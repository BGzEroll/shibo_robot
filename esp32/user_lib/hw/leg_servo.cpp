#include "leg_servo.h"

#include "sys_time.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace uart
{
    namespace
    {
        constexpr uart_port_t PORT = UART_NUM_2;
        constexpr int TX_PIN = 17;
        constexpr int RX_PIN = 16;
        constexpr int BAUD_RATE = 1000000;

        bool initialized = false;
    }

    /**
     * @brief 初始化舵机 UART2
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init()
    {
        if(initialized){return true;}

        uart_config_t config{};
        config.baud_rate = BAUD_RATE;
        config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE;
        config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        config.source_clk = UART_SCLK_DEFAULT;

        if(uart_param_config(PORT, &config) != ESP_OK ||
           uart_set_pin(PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE) != ESP_OK ||
           uart_driver_install(PORT, 256, 0, 0, nullptr, 0) != ESP_OK)
        {
            return false;
        }

        initialized = true;
        return true;
    }

    /**
     * @brief 查询舵机 UART 是否已初始化
     *
     * @return true 已初始化
     * @return false 未初始化
     */
    bool ready()
    {
        return initialized;
    }

    /**
     * @brief 发送舵机 UART 数据并等待发送完成
     *
     * @param[in] data 数据缓冲区
     * @param[in] size 数据长度
     *
     * @return true 已发送完成
     * @return false UART 未初始化或发送失败
     */
    bool write(const uint8_t *data, size_t size)
    {
        return initialized && uart_write_bytes(PORT, data, size) == size &&
            uart_wait_tx_done(PORT, pdMS_TO_TICKS(2)) == ESP_OK;
    }

    /**
     * @brief 读取舵机 UART 数据
     *
     * @param[out] data 接收缓冲区
     * @param[in] size 最大读取长度
     *
     * @return 实际读取长度
     */
    int read(uint8_t *data, size_t size)
    {
        return uart_read_bytes(PORT, data, size, pdMS_TO_TICKS(1));
    }

    /**
     * @brief 清空舵机 UART 接收缓冲区
     */
    void flush()
    {
        uart_flush_input(PORT);
    }
}

namespace leg_servo
{
    namespace
    {
        constexpr uint8_t LEFT_ID = 1;
        constexpr uint8_t RIGHT_ID = 2;
        constexpr uint8_t BROADCAST_ID = 0xFE;
        constexpr uint8_t SYNC_READ = 0x82;
        constexpr uint8_t SYNC_WRITE = 0x83;
        constexpr uint8_t WRITE = 0x03;
        constexpr uint8_t TORQUE_ENABLE = 40;
        constexpr uint8_t MIDDLE_CALIBRATION = 128;
        constexpr uint8_t ACCELERATION = 41;
        constexpr uint8_t PRESENT_POSITION = 56;
        constexpr uint8_t FEEDBACK_SIZE = 15;
        constexpr uint64_t READ_TIMEOUT_US = 5000;

        /**
         * @brief 读取 STS 小端 16 位值
         *
         * @param[in] data 数据首地址
         *
         * @return 16 位值
         */
        uint16_t read_word(const uint8_t *data)
        {
            return static_cast<uint16_t>(data[0] | (data[1] << 8));
        }

        /**
         * @brief 写入 STS 小端 16 位值
         *
         * @param[out] data 数据首地址
         * @param[in] value 16 位值
         */
        void write_word(uint8_t *data, uint16_t value)
        {
            data[0] = static_cast<uint8_t>(value);
            data[1] = static_cast<uint8_t>(value >> 8);
        }

        /**
         * @brief 发送完整协议帧
         *
         * @param[in] frame 帧缓冲区
         * @param[in] size 帧长度
         *
         * @return true 已写入 UART 并发送完成
         */
        bool send(uint8_t *frame, size_t size)
        {
            uint8_t sum = 0;
            for(size_t i = 2; i < size - 1; i++){sum += frame[i];}
            frame[size - 1] = static_cast<uint8_t>(~sum);

            return uart::write(frame, size);
        }

        /**
         * @brief 解析单只舵机反馈
         *
         * @param[in] frame 完整回复帧
         * @param[out] output 舵机反馈
         */
        void decode(const uint8_t *frame, state &output)
        {
            const uint8_t *data = frame + 5;
            const uint16_t position = read_word(data);
            const uint16_t speed = read_word(data + 2);
            const uint16_t load = read_word(data + 4);
            const uint16_t current = read_word(data + 13);

            output.position = position & 0x8000 ?
                -static_cast<int16_t>(position & 0x7FFF) :
                static_cast<int16_t>(position);
            output.speed = speed & 0x8000 ?
                -static_cast<int16_t>(speed & 0x7FFF) :
                static_cast<int16_t>(speed);
            output.load = load & 0x400 ?
                -static_cast<int16_t>(load & 0x3FF) :
                static_cast<int16_t>(load & 0x3FF);
            output.voltage = data[6];
            output.temperature = data[7];
            output.moving = data[10];
            output.current = current & 0x8000 ?
                -static_cast<int16_t>(current & 0x7FFF) :
                static_cast<int16_t>(current);
            output.status = frame[4];
            output.timestamp_us = sys_time::get_us_tick();
            output.valid = true;
        }
    }

    /**
     * @brief 初始化两只 STS3032 共用的 UART2
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init()
    {
        return uart::init();
    }

    /**
     * @brief 同步设置左右腿舵机的目标位置
     *
     * @param[in] left 左侧目标
     * @param[in] right 右侧目标
     *
     * @return true 已发送命令
     * @return false UART 未初始化、位置越界或发送失败
     */
    bool set_target(const command &left, const command &right)
    {
        if(!uart::ready() || left.position < 0 || left.position > 4095 ||
           right.position < 0 || right.position > 4095)
        {
            return false;
        }

        uint8_t frame[24] =
        {
            0xFF, 0xFF, BROADCAST_ID, 20, SYNC_WRITE,
            ACCELERATION, 7,
            LEFT_ID, left.acceleration, 0, 0, 0, 0, 0, 0,
            RIGHT_ID, right.acceleration, 0, 0, 0, 0, 0, 0,
            0
        };

        write_word(frame + 9, static_cast<uint16_t>(left.position));
        write_word(frame + 13, left.speed);
        write_word(frame + 17, static_cast<uint16_t>(right.position));
        write_word(frame + 21, right.speed);
        return send(frame, sizeof(frame));
    }

    /**
     * @brief 主动读取左右腿舵机反馈
     *
     * @param[in, out] left 左侧反馈
     * @param[in, out] right 右侧反馈
     *
     * @return true 两侧均收到有效回复
     * @return false 至少一侧未收到有效回复
     */
    bool read_feedback(state &left, state &right)
    {
        left.valid = false;
        right.valid = false;
        if(!uart::ready()){return false;}

        uart::flush();
        uint8_t request[10] =
        {
            0xFF, 0xFF, BROADCAST_ID, 6, SYNC_READ,
            PRESENT_POSITION, FEEDBACK_SIZE, LEFT_ID, RIGHT_ID, 0
        };
        if(!send(request, sizeof(request))){return false;}

        uint8_t buffer[64];
        size_t used = 0;
        const uint64_t start_us = sys_time::get_us_tick();

        while(sys_time::get_us_tick() - start_us < READ_TIMEOUT_US &&
              (!left.valid || !right.valid))
        {
            const int received = uart::read(
                buffer + used, sizeof(buffer) - used);
            if(received <= 0){continue;}
            used += received;

            size_t offset = 0;
            while(used - offset >= 4)
            {
                if(buffer[offset] != 0xFF || buffer[offset + 1] != 0xFF)
                {
                    offset++;
                    continue;
                }

                const size_t frame_size = static_cast<size_t>(buffer[offset + 3]) + 4;
                if(frame_size != FEEDBACK_SIZE + 6)
                {
                    offset++;
                    continue;
                }
                if(used - offset < frame_size){break;}

                uint8_t sum = 0;
                for(size_t i = offset + 2; i < offset + frame_size - 1; i++)
                {
                    sum += buffer[i];
                }

                if(static_cast<uint8_t>(~sum) == buffer[offset + frame_size - 1])
                {
                    if(buffer[offset + 2] == LEFT_ID)
                    {
                        decode(buffer + offset, left);
                    }
                    else if(buffer[offset + 2] == RIGHT_ID)
                    {
                        decode(buffer + offset, right);
                    }
                }
                offset += frame_size;
            }

            for(size_t i = offset; i < used; i++){buffer[i - offset] = buffer[i];}
            used -= offset;
        }

        return left.valid && right.valid;
    }

    /**
     * @brief 分别设置左右腿舵机的扭矩使能
     *
     * @param[in] left_enabled 左侧使能
     * @param[in] right_enabled 右侧使能
     *
     * @return true 已发送命令
     * @return false UART 未初始化或发送失败
     */
    bool set_torque(bool left_enabled, bool right_enabled)
    {
        if(!uart::ready()){return false;}

        uint8_t frame[12] =
        {
            0xFF, 0xFF, BROADCAST_ID, 8, SYNC_WRITE,
            TORQUE_ENABLE, 1,
            LEFT_ID, static_cast<uint8_t>(left_enabled),
            RIGHT_ID, static_cast<uint8_t>(right_enabled),
            0
        };
        return send(frame, sizeof(frame));
    }

    /**
     * @brief 将指定腿舵机当前机械位置校准为中位
     *
     * @param[in] target 左侧或右侧舵机
     *
     * @return true 校准命令已发送
     * @return false UART 未初始化、参数无效或发送失败
     */
    bool calibrate_middle(side target)
    {
        if(!uart::ready()){return false;}

        uint8_t id = 0;
        if(target == side::left){id = LEFT_ID;}
        else if(target == side::right){id = RIGHT_ID;}
        else{return false;}

        uint8_t frame[8] =
        {
            0xFF, 0xFF, id, 4, WRITE,
            TORQUE_ENABLE, MIDDLE_CALIBRATION, 0
        };
        return send(frame, sizeof(frame));
    }
}
