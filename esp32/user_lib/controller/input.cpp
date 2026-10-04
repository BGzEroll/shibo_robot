#include "input.h"

#include "action.h"
#include "hw/gamepad.h"
#include "sys_time.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace host
{
    namespace
    {
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        control::remote_input latest_input;
        vision_measurement latest_vision;

        /**
         * @brief 读取上位机帧中的小端有符号 16 位值
         *
         * @param[in] data 数据首地址
         *
         * @return 解码后的有符号数值
         */
        int16_t word(const uint8_t *data)
        {
            return static_cast<int16_t>(data[0] | static_cast<uint16_t>(data[1]) << 8);
        }

        /**
         * @brief 从 UART0 提取完整校验帧，错误时逐字节重新同步
         */
        void task(void *)
        {
            uint8_t buffer[256];
            size_t used = 0;
            control::remote_input input;
            input.stream_id = 1;
            vision_measurement vision;

            while(true)
            {
                const int32_t received = uart_read_bytes(UART_NUM_0,
                    buffer + used, sizeof(buffer) - used, pdMS_TO_TICKS(10));
                if(received > 0){used += received;}

                size_t offset = 0;
                while(used - offset >= 5)
                {
                    const uint8_t *frame = buffer + offset;
                    if(frame[0] != 0xFF || frame[1] != 0xAA ||
                       (frame[2] != 1 && frame[2] != 2) ||
                       (frame[2] == 1 && frame[3] != 14) ||
                       (frame[2] == 2 && frame[3] != 4))
                    {
                        offset++;
                        continue;
                    }

                    const size_t size = frame[3] + 5;
                    if(used - offset < size){break;}
                    uint8_t checksum = 0;
                    const size_t start = frame[2] == 1 ? 4 : 0;
                    for(size_t i = start; i < size - 1; i++){checksum += frame[i];}
                    if(checksum != frame[size - 1])
                    {
                        offset++;
                        continue;
                    }
                    const uint8_t *payload = frame + 4;

                    const uint64_t now = sys_time::get_us_tick();
                    if(frame[2] == 1)
                    {
                        const uint16_t buttons = static_cast<uint16_t>(word(payload));
                        const uint16_t pressed = buttons & ~input.buttons;
                        for(uint32_t i = 0; i < 16; i++)
                        {
                            if(pressed & (1 << i)){input.press_count[i]++;}
                        }

                        for(uint32_t i = 0; i < 6; i++)
                        {
                            input.axes[i] = std::clamp(word(payload + 2 + i * 2) * 0.001f,
                                i < 4 ? -1.0f : 0.0f, 1.0f);
                        }

                        input.buttons = buttons;
                        input.timestamp_us = static_cast<uint32_t>(now);
                        input.valid = true;
                        portENTER_CRITICAL(&lock);
                        latest_input = input;
                        portEXIT_CRITICAL(&lock);
                    }
                    else
                    {
                        vision.dx = word(payload);
                        vision.dy = word(payload + 2);
                        vision.timestamp_us = now;
                        vision.sequence++;
                        vision.valid = vision.dx != 32767 && vision.dy != 32767;
                        portENTER_CRITICAL(&lock);
                        latest_vision = vision;
                        portEXIT_CRITICAL(&lock);
                    }
                    offset += size;
                }

                memmove(buffer, buffer + offset, used - offset);
                used -= offset;
                if(used == sizeof(buffer)){used = 0;}
            }
        }
    }

    /**
     * @brief 初始化上位机 UART0 并启动遥控、视觉接收任务
     *
     * @return true 串口和接收任务初始化成功
     * @return false UART 配置或任务创建失败
     */
    bool init()
    {
        uart_config_t uart{};
        uart.baud_rate = 115200;
        uart.data_bits = UART_DATA_8_BITS;
        uart.parity = UART_PARITY_DISABLE;
        uart.stop_bits = UART_STOP_BITS_1;
        uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart.source_clk = UART_SCLK_DEFAULT;
        return uart_param_config(UART_NUM_0, &uart) == ESP_OK &&
            uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
            uart_driver_install(UART_NUM_0, 512, 0, 0, nullptr, 0) == ESP_OK &&
            xTaskCreatePinnedToCore(task, "host", 3072, nullptr, 2, nullptr, 0) == pdPASS;
    }

    /**
     * @brief 获取最新上位机遥控输入
     *
     * @param[out] out 遥控输入快照
     *
     * @return true 已收到有效遥控帧
     * @return false 尚无有效遥控帧
     */
    bool get_input(control::remote_input &out)
    {
        portENTER_CRITICAL(&lock);
        out = latest_input;
        portEXIT_CRITICAL(&lock);
        return out.valid;
    }

    /**
     * @brief 获取视觉偏移并检查 350 ms 接收超时
     *
     * @param[out] out 视觉测量快照
     *
     * @return true 目标有效且测量未超时
     * @return false 未检测到目标或测量已超时
     */
    bool get_vision(vision_measurement &out)
    {
        portENTER_CRITICAL(&lock);
        out = latest_vision;
        portEXIT_CRITICAL(&lock);

        const uint64_t now = sys_time::get_us_tick();
        out.valid = out.valid && now >= out.timestamp_us && now - out.timestamp_us <= 350000;
        return out.valid;
    }
}

namespace control::input_router
{
    namespace
    {
        input_source previous_source = input_source::NONE;
        uint32_t previous_stream = 0;
        uint16_t press_count[16] = {};
        bool was_fresh = false;

        /**
         * @brief 去除摇杆死区并重新归一化输入
         *
         * @param[in] value 原始摇杆输入，范围 -1 至 1
         *
         * @return 死区处理后的摇杆输入
         */
        float axis(float value)
        {
            return fabsf(value) <= 0.05f ? 0.0f : copysignf((fabsf(value) - 0.05f) / 0.95f, value);
        }
    }

    /**
     * @brief 初始化输入来源和按键边沿状态
     */
    void init()
    {
        previous_source = input_source::NONE;
        previous_stream = 0;
        memset(press_count, 0, sizeof(press_count));
        was_fresh = false;
    }

    /**
     * @brief 优先读取 Xbox 输入，切换来源或恢复连接时丢弃历史按键
     *
     * @param[in] mode 当前动作模式
     * @param[in] max_linear_vel 最大线速度，单位 m/s
     * @param[in] max_steer_vel 最大偏航角速度，单位 rad/s
     * @param[out] out 本周期控制输入
     */
    void update(action::mode mode, float max_linear_vel,
        float max_steer_vel, control_input &out)
    {
        out = {};
        remote_input snapshot;
        if(gamepad::connected())
        {
            out.source = input_source::XBOX;
            gamepad::get(snapshot);
        }
        else
        {
            out.source = input_source::HOST;
            host::get_input(snapshot);
        }

        const uint32_t now = static_cast<uint32_t>(sys_time::get_us_tick());
        out.timestamp_us = snapshot.timestamp_us;
        out.fresh = snapshot.valid && static_cast<uint32_t>(now - snapshot.timestamp_us) <= 250000;

        uint16_t pressed = 0;
        const bool continuous = was_fresh && out.fresh && previous_source == out.source &&
            previous_stream == snapshot.stream_id;
        for(uint32_t i = 0; i < 16; i++)
        {
            if(continuous && press_count[i] != snapshot.press_count[i]){pressed |= 1 << i;}
            press_count[i] = snapshot.press_count[i];
        }
        previous_source = out.source;
        previous_stream = snapshot.stream_id;
        was_fresh = out.fresh;

        if(!out.fresh){return;}

        const uint16_t held = snapshot.buttons;
        const bool modifier = held & buttons::SELECT;
        out.linear = axis(snapshot.axes[3]) * max_linear_vel;
        if(out.linear < 0.0f){out.linear *= 0.8f;}
        out.yaw = -axis(snapshot.axes[0]) * max_steer_vel;

        if(modifier)
        {
            out.camera_direction = (held & buttons::UP ? 1 : 0) - (held & buttons::DOWN ? 1 : 0);
        }
        else
        {
            out.leg_height_direction = (held & buttons::DOWN ? 1 : 0) - (held & buttons::UP ? 1 : 0);
            out.roll_direction = (held & buttons::RIGHT ? 1 : 0) - (held & buttons::LEFT ? 1 : 0);
        }

        if(pressed & buttons::START)
        {
            out.action = action_request::STOP;
            return;
        }

        if(mode == action::mode::STOP)
        {
            if(pressed & buttons::RB){out.action = action_request::BOOT;}
        }
        else if(mode == action::mode::BALANCE)
        {
            out.reset_leg = (pressed & buttons::LS) && fabsf(out.linear) < max_linear_vel * 0.05f;

            if(modifier)
            {
                if(pressed & buttons::X){out.action = action_request::KICK_PLACE;}
                else if(pressed & buttons::Y){out.action = action_request::KICK_RUN;}
                else if(pressed & buttons::B){out.action = action_request::RESET_BALANCE;}
            }
            else
            {
                if(pressed & buttons::B){out.action = action_request::JUMP_RIGHT;}
                else if(pressed & buttons::X){out.action = action_request::JUMP_LEFT;}
                else if(pressed & buttons::A){out.action = action_request::JUMP_BACKWARD;}
                else if(pressed & buttons::Y){out.action = action_request::JUMP_FORWARD;}
                else if(pressed & buttons::RS){out.action = action_request::JUMP_IN_PLACE;}
                else if(pressed & buttons::LB){out.action = action_request::SIT;}
            }
        }
        else if(mode == action::mode::SIT || mode == action::mode::MIDDLE_CALIBRATION)
        {
            out.disable_leg_torque = held & buttons::LS;
            if(modifier && (pressed & buttons::LB)){out.action = action_request::MIDDLE_CALIBRATION;}
            else if(pressed & buttons::RB){out.action = action_request::EXIT;}
        }
        else if(mode == action::mode::KICK_PLACE || mode == action::mode::KICK_RUN)
        {
            if(modifier && (pressed & buttons::B)){out.action = action_request::KICK_EXIT;}
            else if(modifier && (pressed & buttons::X)){out.action = action_request::KICK_PLACE;}
            else if(modifier && (pressed & buttons::Y)){out.action = action_request::KICK_RUN;}
        }
    }
}
