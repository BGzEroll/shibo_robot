#include "host.h"

#include "sys_time.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
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
