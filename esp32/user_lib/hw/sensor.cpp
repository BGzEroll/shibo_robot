#include "sensor.h"

#include "sys_time.h"
#include <math.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace i2c
{
    constexpr uint32_t LEFT_DONE = (1U << 0);
    constexpr uint32_t RIGHT_DONE = (1U << 1);

    namespace
    {
        constexpr i2c_port_num_t LEFT_PORT = 0;
        constexpr i2c_port_num_t RIGHT_PORT = 1;

        constexpr gpio_num_t LEFT_SDA = GPIO_NUM_19;
        constexpr gpio_num_t LEFT_SCL = GPIO_NUM_18;

        constexpr gpio_num_t RIGHT_SDA = GPIO_NUM_23;
        constexpr gpio_num_t RIGHT_SCL = GPIO_NUM_5;

        constexpr uint32_t LEFT_FREQ_HZ = 400000;
        constexpr uint32_t RIGHT_FREQ_HZ = 400000;

        constexpr uint16_t DEFAULT_ADDRESS = 0x36;

        struct context
        {
            i2c_master_bus_handle_t bus = nullptr;
            i2c_master_dev_handle_t dev = nullptr;

            uint32_t notify_bit = 0;
            volatile uint64_t completion_time_us = 0;
        };

        context left;
        context right;

        TaskHandle_t task_handle = nullptr;

        /**
         * @brief I2C 异步事务完成回调
         *
         * @param[in] arg I2C 上下文
         *
         * @return true 已唤醒高优先级任务
         * @return false 未唤醒高优先级任务
         */
        bool done_callback(
            i2c_master_dev_handle_t,
            const i2c_master_event_data_t *,
            void *arg)
        {
            auto *ctx = static_cast<context *>(arg);

            ctx->completion_time_us = sys_time::get_us_tick();

            BaseType_t task_woken = pdFALSE;

            xTaskNotifyFromISR(
                task_handle,
                ctx->notify_bit,
                eSetBits,
                &task_woken);

            if(task_woken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }

            return task_woken == pdTRUE;
        }

        /**
         * @brief 初始化 I2C 总线
         *
         * @param[in, out] ctx I2C 上下文
         * @param[in] port I2C 控制器
         * @param[in] sda SDA 引脚
         * @param[in] scl SCL 引脚
         * @param[in] frequency 总线频率
         * @param[in] notify_bit 任务通知位
         */
        void init_bus(
            context &ctx,
            i2c_port_num_t port,
            gpio_num_t sda,
            gpio_num_t scl,
            uint32_t frequency,
            uint32_t notify_bit)
        {
            ctx.notify_bit = notify_bit;

            i2c_master_bus_config_t bus_config{};
            bus_config.i2c_port = port;
            bus_config.sda_io_num = sda;
            bus_config.scl_io_num = scl;
            bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            bus_config.glitch_ignore_cnt = 7;
            bus_config.trans_queue_depth = 1;

            ESP_ERROR_CHECK(
                i2c_new_master_bus(
                    &bus_config,
                    &ctx.bus));

            i2c_device_config_t dev_config{};
            dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_config.device_address = DEFAULT_ADDRESS;
            dev_config.scl_speed_hz = frequency;

            ESP_ERROR_CHECK(
                i2c_master_bus_add_device(
                    ctx.bus,
                    &dev_config,
                    &ctx.dev));

            i2c_master_event_callbacks_t callbacks{};
            callbacks.on_trans_done = done_callback;

            ESP_ERROR_CHECK(
                i2c_master_register_event_callbacks(
                    ctx.dev,
                    &callbacks,
                    &ctx));
        }
    }

    /**
     * @brief 初始化传感器 I2C
     *
     * @param[in] sensor_task Sensor 任务句柄
     */
    void init(TaskHandle_t sensor_task)
    {
        task_handle = sensor_task;

        init_bus(
            left,
            LEFT_PORT,
            LEFT_SDA,
            LEFT_SCL,
            LEFT_FREQ_HZ,
            LEFT_DONE);

        init_bus(
            right,
            RIGHT_PORT,
            RIGHT_SDA,
            RIGHT_SCL,
            RIGHT_FREQ_HZ,
            RIGHT_DONE);
    }

    /**
     * @brief 修改右侧 I2C 设备地址
     *
     * @param[in] address 设备地址
     */
    void set_right_address(uint16_t address)
    {
        ESP_ERROR_CHECK(
            i2c_master_device_change_address(
                right.dev,
                address,
                -1));
    }

    /**
     * @brief 发起左侧 I2C 异步读取
     *
     * @param[in] reg 寄存器地址
     * @param[out] data 接收缓冲区
     * @param[in] size 读取长度
     */
    void read_left(const uint8_t *reg, uint8_t *data, size_t size)
    {
        ESP_ERROR_CHECK(
            i2c_master_transmit_receive(
                left.dev,
                reg,
                1,
                data,
                size,
                -1));
    }

    /**
     * @brief 发起右侧 I2C 异步读取
     *
     * @param[in] reg 寄存器地址
     * @param[out] data 接收缓冲区
     * @param[in] size 读取长度
     */
    void read_right(const uint8_t *reg, uint8_t *data, size_t size)
    {
        ESP_ERROR_CHECK(
            i2c_master_transmit_receive(
                right.dev,
                reg,
                1,
                data,
                size,
                -1));
    }

    /**
     * @brief 同步写入右侧 I2C 单寄存器
     *
     * @param[in] reg 寄存器地址
     * @param[in] value 寄存器值
     *
     * @note 仅用于 IMU 初始化
     */
    void write_right_sync(uint8_t reg, uint8_t value)
    {
        uint8_t data[2] = {reg, value};

        ESP_ERROR_CHECK(
            i2c_master_transmit(
                right.dev,
                data,
                sizeof(data),
                -1));

        uint32_t notification = 0;

        do
        {
            xTaskNotifyWait(
                0,
                RIGHT_DONE,
                &notification,
                portMAX_DELAY);
        }
        while((notification & RIGHT_DONE) == 0);
    }

    /**
     * @brief 获取左侧 I2C 完成时间
     *
     * @return 事务完成时间，单位 us
     */
    uint64_t left_completion_time_us()
    {
        return left.completion_time_us;
    }

    /**
     * @brief 获取右侧 I2C 完成时间
     *
     * @return 事务完成时间，单位 us
     */
    uint64_t right_completion_time_us()
    {
        return right.completion_time_us;
    }
}

namespace as5600
{
    constexpr uint16_t ADDRESS = 0x36;

    namespace
    {
        constexpr uint8_t REG_RAW_ANGLE = 0x0C;

        constexpr int32_t RESOLUTION = 4096;
        constexpr int32_t HALF_RESOLUTION = RESOLUTION / 2;

        /*
         * count/us -> mrad/s
         *
         * 2π / 4096 * 1000 * 1000000
         * ≈ 1533980.788
         */
        constexpr int32_t SPEED_SCALE = 1533981;
        constexpr int32_t SPEED_FILTER_US = 3000;       // 速度低通时间常数：3 ms

        struct state
        {
            uint8_t reg = REG_RAW_ANGLE;
            uint8_t raw[2] = {};

            bool first_sample = true;

            uint16_t last_raw = 0;
            uint64_t last_time_us = 0;

            int32_t full_count = 0;
            int32_t speed_mrad_s = 0;
        };

        state left;
        state right;

        /**
         * @brief 处理一次 AS5600 采样
         *
         * @param[in, out] encoder 编码器状态
         * @param[in] now_us 采样完成时间
         *
         * @return 编码器数据
         */
        sensor::encoder_data process(state &encoder, uint64_t now_us)
        {
            const uint16_t raw =
                static_cast<uint16_t>(
                    ((static_cast<uint16_t>(encoder.raw[0]) & 0x0F) << 8) |
                    encoder.raw[1]);

            if(encoder.first_sample)
            {
                encoder.first_sample = false;

                encoder.last_raw = raw;
                encoder.last_time_us = now_us;

                encoder.full_count = raw;
                encoder.speed_mrad_s = 0;
            }
            else
            {
                int32_t delta =
                    static_cast<int32_t>(raw) -
                    static_cast<int32_t>(encoder.last_raw);

                if(delta > HALF_RESOLUTION)
                {
                    delta -= RESOLUTION;
                }
                else if(delta < -HALF_RESOLUTION)
                {
                    delta += RESOLUTION;
                }

                encoder.full_count += delta;

                const uint64_t dt_us =
                    now_us -
                    encoder.last_time_us;

                if(dt_us != 0)
                {
                    const int64_t numerator =
                        static_cast<int64_t>(encoder.speed_mrad_s) *
                        SPEED_FILTER_US +
                        static_cast<int64_t>(delta) *
                        SPEED_SCALE;

                    encoder.speed_mrad_s =
                        static_cast<int32_t>(numerator /
                            (SPEED_FILTER_US +
                            static_cast<int64_t>(dt_us)));
                }

                encoder.last_raw = raw;
                encoder.last_time_us = now_us;
            }

            sensor::encoder_data data;
            data.timestamp_us = now_us;
            data.full_count = encoder.full_count;
            data.speed_mrad_s = encoder.speed_mrad_s;

            return data;
        }
    }

    /**
     * @brief 发起左编码器读取
     */
    void start_left_read()
    {
        i2c::read_left(
            &left.reg,
            left.raw,
            sizeof(left.raw));
    }

    /**
     * @brief 发起右编码器读取
     */
    void start_right_read()
    {
        i2c::read_right(
            &right.reg,
            right.raw,
            sizeof(right.raw));
    }

    /**
     * @brief 处理左编码器数据
     *
     * @param[in] now_us 采样完成时间
     *
     * @return 左编码器数据
     */
    sensor::encoder_data process_left(uint64_t now_us)
    {
        return process(
            left,
            now_us);
    }

    /**
     * @brief 处理右编码器数据
     *
     * @param[in] now_us 采样完成时间
     *
     * @return 右编码器数据
     */
    sensor::encoder_data process_right(uint64_t now_us)
    {
        return process(
            right,
            now_us);
    }
}

namespace mpu6050
{
    constexpr uint16_t ADDRESS = 0x68;

    namespace
    {
        constexpr uint8_t REG_SMPLRT_DIV = 0x19;
        constexpr uint8_t REG_CONFIG = 0x1A;
        constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
        constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
        constexpr uint8_t REG_DATA = 0x3B;
        constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;

        constexpr uint32_t CALIBRATION_SAMPLES = 1000;

        constexpr float PI = 3.14159265358979323846f;
        constexpr float DEG2RAD = PI / 180.0f;
        constexpr float ACC_COEF = 0.02f;

        uint8_t reg = REG_DATA;
        uint8_t raw[14] = {};

        int64_t gyro_offset_sum[3] = {};
        float gyro_offset[3] = {};

        uint32_t calibration_count = 0;
        uint64_t last_time_us = 0;

        float angle[3] = {};

        /**
         * @brief 读取大端有符号 16 位数据
         *
         * @param[in] data 原始数据
         *
         * @return 转换后的数值
         */
        int16_t read_i16(const uint8_t *data)
        {
            return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                static_cast<uint16_t>(data[1]));
        }

        /**
         * @brief 处理陀螺仪零偏校准
         *
         * @param[in] raw_gyro 原始陀螺仪数据
         *
         * @return true 校准完成
         * @return false 校准进行中
         */
        bool process_calibration(const int16_t raw_gyro[3])
        {
            if(calibration_count >= CALIBRATION_SAMPLES)
            {
                return true;
            }

            for(uint8_t i = 0; i < 3; i++)
            {
                gyro_offset_sum[i] += raw_gyro[i];
            }

            calibration_count++;

            if(calibration_count < CALIBRATION_SAMPLES){return false;}

            for(uint8_t i = 0; i < 3; i++)
            {
                const float raw_offset = static_cast<float>(gyro_offset_sum[i]) /
                    static_cast<float>(CALIBRATION_SAMPLES);

                gyro_offset[i] =
                    raw_offset /
                    65.5f *
                    DEG2RAD;
            }

            last_time_us = 0;

            return false;
        }
    }

    /**
     * @brief 初始化 MPU6050
     */
    void init()
    {
        i2c::set_right_address(ADDRESS);

        /*
         * PLL X gyro clock
         */
        i2c::write_right_sync(REG_PWR_MGMT_1, 0x01);

        sys_time::delay_ms(10);

        /*
         * DLPF_CFG = 3
         *
         * Gyro/Accel bandwidth ≈ 44 Hz
         * Gyro internal output rate = 1 kHz
         */
        i2c::write_right_sync(REG_CONFIG, 0x03);

        /*
         * 1000 / (1 + 4)
         * = 200 Hz
         */
        i2c::write_right_sync(REG_SMPLRT_DIV, 0x04);

        /*
         * ±500 deg/s
         * 65.5 LSB/(deg/s)
         */
        i2c::write_right_sync(REG_GYRO_CONFIG, 0x08);

        /*
         * ±2 g
         * 16384 LSB/g
         */
        i2c::write_right_sync(REG_ACCEL_CONFIG, 0x00);

        i2c::set_right_address(as5600::ADDRESS);
    }

    /**
     * @brief 发起 MPU6050 数据读取
     */
    void start_read()
    {
        i2c::read_right(
            &reg,
            raw,
            sizeof(raw));
    }

    /**
     * @brief 处理一次 MPU6050 采样
     *
     * @param[in] now_us 采样完成时间
     * @param[out] data IMU 数据
     *
     * @return true 数据有效
     * @return false 陀螺仪仍在校准
     */
    bool process(uint64_t now_us, sensor::imu_data &data)
    {
        int16_t raw_acc[3];
        int16_t raw_gyro[3];

        raw_acc[0] = read_i16(&raw[0]);
        raw_acc[1] = read_i16(&raw[2]);
        raw_acc[2] = read_i16(&raw[4]);

        const int16_t raw_temperature = read_i16(&raw[6]);

        raw_gyro[0] = read_i16(&raw[8]);
        raw_gyro[1] = read_i16(&raw[10]);
        raw_gyro[2] = read_i16(&raw[12]);

        if(!process_calibration(raw_gyro))
        {
            return false;
        }

        data.timestamp_us = now_us;

        data.temperature = 
            static_cast<float>(raw_temperature) /
            340.0f +
            36.53f;

        for(uint8_t i = 0; i < 3; i++)
        {
            data.acc[i] =
                static_cast<float>(raw_acc[i]) /
                16384.0f;

            data.gyro[i] =
                static_cast<float>(raw_gyro[i]) /
                65.5f *
                DEG2RAD -
                gyro_offset[i];
        }

        const float acc_roll = atan2f(data.acc[1], data.acc[2]);

        const float acc_pitch =
            atan2f(
                -data.acc[0],
                sqrtf(
                    data.acc[1] * data.acc[1] +
                    data.acc[2] * data.acc[2]));

        if(last_time_us == 0)
        {
            angle[0] = acc_roll;
            angle[1] = acc_pitch;
            angle[2] = 0.0f;
        }
        else
        {
            const float dt = static_cast<float>(now_us - last_time_us) * 1.0e-6f;

            angle[0] =
                (1.0f - ACC_COEF) *
                (
                    angle[0] +
                    data.gyro[0] * dt
                ) +
                ACC_COEF *
                acc_roll;

            angle[1] =
                (1.0f - ACC_COEF) *
                (
                    angle[1] +
                    data.gyro[1] * dt
                ) +
                ACC_COEF *
                acc_pitch;

            angle[2] += data.gyro[2] * dt;

            if(angle[2] > PI)
            {
                angle[2] -= 2.0f * PI;
            }
            else if(angle[2] < -PI)
            {
                angle[2] += 2.0f * PI;
            }
        }

        last_time_us = now_us;

        for(uint8_t i = 0; i < 3; i++)
        {
            data.angle[i] = angle[i];
        }

        return true;
    }
}

namespace sensor
{
    namespace
    {
        constexpr uint64_t MPU_PERIOD_US = 5000;

        constexpr uint32_t TASK_STACK = 4096;
        constexpr UBaseType_t TASK_PRIORITY = 5;
        constexpr BaseType_t TASK_CORE = 1;

        portMUX_TYPE package_lock = portMUX_INITIALIZER_UNLOCKED;

        package latest_package;
        bool right_reading_imu = false;
        uint64_t next_imu_time_us = 0;
        bool started = false;

        /**
         * @brief 发布左编码器数据
         *
         * @param[in] data 左编码器数据
         */
        void publish_left(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.left_encoder = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布右编码器数据
         *
         * @param[in] data 右编码器数据
         */
        void publish_right(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.right_encoder = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布 IMU 数据
         *
         * @param[in] data IMU 数据
         */
        void publish_imu(const imu_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.imu = data;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 处理左编码器事务完成
         */
        void handle_left_encoder()
        {
            publish_left(as5600::process_left(i2c::left_completion_time_us()));
            as5600::start_left_read();
        }

        /**
         * @brief 处理右编码器事务完成
         */
        void handle_right_encoder()
        {
            const uint64_t now_us = i2c::right_completion_time_us();
            publish_right(as5600::process_right(now_us));

            // MPU6050 到时间后插队一次
            if(now_us >= next_imu_time_us)
            {
                do
                {
                    next_imu_time_us += MPU_PERIOD_US;
                }
                while(now_us >= next_imu_time_us);

                right_reading_imu = true;
                i2c::set_right_address(mpu6050::ADDRESS);
                mpu6050::start_read();
                return;
            }

            as5600::start_right_read();
        }

        /**
         * @brief 处理 MPU6050 事务完成
         */
        void handle_imu()
        {
            imu_data data;
            if(mpu6050::process(i2c::right_completion_time_us(), data))
            {
                publish_imu(data);
            }

            // MPU 完成后立即切回 AS5600
            i2c::set_right_address(as5600::ADDRESS);
            right_reading_imu = false;
            as5600::start_right_read();
        }

        /**
         * @brief Sensor 任务入口
         */
        void task(void *)
        {
            i2c::init(xTaskGetCurrentTaskHandle());

            mpu6050::init();
            next_imu_time_us = sys_time::get_us_tick() + MPU_PERIOD_US;

            as5600::start_left_read();
            as5600::start_right_read();

            while(true)
            {
                uint32_t notification = 0;

                xTaskNotifyWait(
                    0,
                    UINT32_MAX,
                    &notification,
                    portMAX_DELAY);

                if(notification & i2c::LEFT_DONE)
                {
                    handle_left_encoder();
                }

                if(notification & i2c::RIGHT_DONE)
                {
                    if(right_reading_imu)
                    {
                        handle_imu();
                    }
                    else
                    {
                        handle_right_encoder();
                    }
                }
            }
        }
    }

    /**
     * @brief 初始化传感器模块
     *
     * @return true 初始化成功
     * @return false Sensor 任务创建失败
     */
    bool init()
    {
        if(started){return true;}

        if(xTaskCreatePinnedToCore(
            task,
            "sensor",
            TASK_STACK,
            nullptr,
            TASK_PRIORITY,
            nullptr,
            TASK_CORE) != pdPASS)
        {
            return false;
        }

        started = true;
        return true;
    }

    /**
     * @brief 获取最新传感器数据
     *
     * @param[out] snapshot 传感器数据快照
     *
     * @return true 数据有效
     * @return false IMU 尚未完成校准
     */
    bool get_package(package &snapshot)
    {
        portENTER_CRITICAL(&package_lock);
        snapshot = latest_package;
        portEXIT_CRITICAL(&package_lock);
        return snapshot.imu.timestamp_us != 0;
    }
}
