#include "sensor.h"

#include "sys_time.h"
#include <math.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace i2c
{
    namespace
    {
        constexpr i2c_port_num_t LEFT_PORT = 0;
        constexpr i2c_port_num_t RIGHT_PORT = 1;

        constexpr int LEFT_SDA = 19;
        constexpr int LEFT_SCL = 18;
        constexpr int RIGHT_SDA = 23;
        constexpr int RIGHT_SCL = 5;

        constexpr uint32_t LEFT_FREQ_HZ = 400000;
        constexpr uint32_t RIGHT_FREQ_HZ = 400000;

        constexpr uint16_t AS5600_ADDRESS = 0x36;

        constexpr uint32_t NOTIFY_LEFT = (1U << 0);
        constexpr uint32_t NOTIFY_RIGHT = (1U << 1);

        struct context
        {
            i2c_master_bus_handle_t bus = nullptr;
            i2c_master_dev_handle_t dev = nullptr;

            uint32_t notify_bit = 0;
        };

        context left;
        context right;

        TaskHandle_t sensor_task_handle = nullptr;

        /**
         * @brief I2C 异步事务完成回调
         */
        bool done_callback(
            i2c_master_dev_handle_t,
            const i2c_master_event_data_t *,
            void *arg)
        {
            auto *ctx = static_cast<context *>(arg);

            BaseType_t task_woken = pdFALSE;

            xTaskNotifyFromISR(
                sensor_task_handle,
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
         * @brief 初始化一条固定 I2C 总线
         */
        void init_bus(
            context &ctx,
            i2c_port_num_t port,
            int sda,
            int scl,
            uint32_t frequency,
            uint32_t notify_bit)
        {
            ctx.notify_bit = notify_bit;

            i2c_master_bus_config_t bus_config{};
            bus_config.i2c_port = port;
            bus_config.sda_io_num = static_cast<gpio_num_t>(sda);
            bus_config.scl_io_num = static_cast<gpio_num_t>(scl);
            bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            bus_config.glitch_ignore_cnt = 7;
            bus_config.intr_priority = 0;
            bus_config.trans_queue_depth = 1;       // 启用 asynchronous transaction
            bus_config.flags.enable_internal_pullup = false;        // 硬件使用外部上拉

            ESP_ERROR_CHECK(
                i2c_new_master_bus(
                    &bus_config,
                    &ctx.bus));

            i2c_device_config_t dev_config{};
            dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_config.device_address = AS5600_ADDRESS;
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

    constexpr uint32_t left_notify_bit()
    {
        return NOTIFY_LEFT;
    }

    constexpr uint32_t right_notify_bit()
    {
        return NOTIFY_RIGHT;
    }

    /**
     * @brief 初始化两条 I2C 总线
     */
    void init(TaskHandle_t task_handle)
    {
        sensor_task_handle = task_handle;

        init_bus(
            left,
            LEFT_PORT,
            LEFT_SDA,
            LEFT_SCL,
            LEFT_FREQ_HZ,
            NOTIFY_LEFT);

        init_bus(
            right,
            RIGHT_PORT,
            RIGHT_SDA,
            RIGHT_SCL,
            RIGHT_FREQ_HZ,
            NOTIFY_RIGHT);
    }

    /**
     * @brief 等待指定 I2C 事务完成
     *
     * @note 仅供启动初始化阶段使用
     */
    void wait(uint32_t notify_bit)
    {
        uint32_t notification = 0;

        while((notification & notify_bit) == 0)
        {
            xTaskNotifyWait(
                0,
                notify_bit,
                &notification,
                portMAX_DELAY);
        }
    }

    /**
     * @brief 修改右侧 I2C 当前设备地址
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
     * @brief 左侧 I2C 异步寄存器读取
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
     * @brief 右侧 I2C 异步寄存器读取
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
     * @brief 右侧 I2C 异步寄存器写入
     */
    void write_right(const uint8_t *data, size_t size)
    {
        ESP_ERROR_CHECK(
            i2c_master_transmit(
                right.dev,
                data,
                size,
                -1));
    }
}

namespace as5600
{
    namespace
    {
        constexpr uint16_t ADDRESS = 0x36;
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


    constexpr uint16_t address()
    {
        return ADDRESS;
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
     * @brief 处理左编码器新数据
     */
    sensor::encoder_data process_left(uint64_t now_us)
    {
        return process(
            left,
            now_us);
    }

    /**
     * @brief 处理右编码器新数据
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
    namespace
    {
        constexpr const char *TAG = "mpu6050";

        constexpr uint16_t ADDRESS = 0x68;

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
        bool calibrated = false;

        uint64_t last_time_us = 0;

        float angle[3] = {};

        /**
         * @brief 读取大端 int16
         */
        int16_t read_i16(const uint8_t *data)
        {
            return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                static_cast<uint16_t>(data[1]));
        }

        /**
         * @brief 写入一个 MPU6050 寄存器
         */
        void write_register(uint8_t address, uint8_t value)
        {
            uint8_t tx[2] = {address, value};
            i2c::write_right(tx, sizeof(tx));

            // tx 位于栈上
            // 必须等异步事务真正结束后才能返回
            i2c::wait(i2c::right_notify_bit());
        }

        /**
         * @brief 处理陀螺仪启动校准
         */
        bool process_calibration(const int16_t raw_gyro[3])
        {
            if(calibrated){return true;}

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

            calibrated = true;
            last_time_us = 0;

            ESP_LOGI(
                TAG,
                "gyro calibration finished");

            return false;
        }
    }


    constexpr uint16_t address()
    {
        return ADDRESS;
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
        write_register(REG_PWR_MGMT_1, 0x01);

        sys_time::delay_ms(10);

        /*
         * DLPF_CFG = 3
         *
         * Gyro/Accel bandwidth ≈ 44 Hz
         * Gyro internal output rate = 1 kHz
         */
        write_register(REG_CONFIG, 0x03);

        /*
         * 1000 / (1 + 4)
         * = 200 Hz
         */
        write_register(REG_SMPLRT_DIV, 0x04);

        /*
         * ±500 deg/s
         * 65.5 LSB/(deg/s)
         */
        write_register(REG_GYRO_CONFIG, 0x08);

        /*
         * ±2 g
         * 16384 LSB/g
         */
        write_register(REG_ACCEL_CONFIG, 0x00);

        i2c::set_right_address(as5600::address());
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
     * @brief 处理一帧 MPU6050 数据
     *
     * @return true 已产生有效 IMU 数据
     * @return false 尚处于启动校准阶段
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
                    data.acc[1] *
                    data.acc[1] +
                    data.acc[2] *
                    data.acc[2]));

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

            angle[2] +=
                data.gyro[2] * dt;

            if(angle[2] > PI)
            {
                angle[2] -=
                    2.0f * PI;
            }
            else if(angle[2] < -PI)
            {
                angle[2] +=
                    2.0f * PI;
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
        constexpr const char *TAG = "sensor";

        constexpr uint64_t MPU6050_PERIOD_US = 5000;

        constexpr uint32_t TASK_STACK = 4096;
        constexpr UBaseType_t TASK_PRIORITY = 20;
        constexpr BaseType_t TASK_CORE = 1;

        package latest_package;

        bool left_valid = false;
        bool right_valid = false;
        bool imu_valid = false;

        bool started = false;

        portMUX_TYPE package_lock = portMUX_INITIALIZER_UNLOCKED;

        uint64_t next_imu_time_us = 0;

        enum class right_phase : uint8_t
        {
            encoder,
            imu
        };

        right_phase phase = right_phase::encoder;


        /**
         * @brief 发布左编码器数据
         */
        void publish_left(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.left_encoder = data;
            left_valid = true;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布右编码器数据
         */
        void publish_right(const encoder_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.right_encoder = data;
            right_valid = true;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 发布 IMU 数据
         */
        void publish_imu(const imu_data &data)
        {
            portENTER_CRITICAL(&package_lock);
            latest_package.imu = data;
            imu_valid = true;
            portEXIT_CRITICAL(&package_lock);
        }

        /**
         * @brief 推进 MPU6050 的 200 Hz deadline
         */
        void update_imu_deadline(uint64_t now_us)
        {
            do
            {
                next_imu_time_us += MPU6050_PERIOD_US;
            }
            while(now_us >= next_imu_time_us);
        }

        /**
         * @brief 左侧 I2C 完成
         */
        void handle_left_i2c()
        {
            const uint64_t now_us = sys_time::get_us_tick();
            publish_left(as5600::process_left(now_us));
            as5600::start_left_read();
        }

        /**
         * @brief 右 AS5600 完成
         */
        void handle_right_encoder()
        {
            const uint64_t now_us = sys_time::get_us_tick();
            publish_right(as5600::process_right(now_us));

            // MPU6050 到时间后插队一次
            if(now_us >= next_imu_time_us)
            {
                update_imu_deadline(now_us);
                phase = right_phase::imu;
                i2c::set_right_address(mpu6050::address());
                mpu6050::start_read();
                return;
            }

            as5600::start_right_read();
        }

        /**
         * @brief MPU6050 完成
         */
        void handle_imu()
        {
            const uint64_t now_us = sys_time::get_us_tick();

            imu_data data;
            if(mpu6050::process(now_us, data))
            {
                publish_imu(data);
            }

            // MPU 完成后立即切回 AS5600
            i2c::set_right_address(as5600::address());
            phase = right_phase::encoder;
            as5600::start_right_read();
        }

        /**
         * @brief 右侧 I2C 完成
         */
        void handle_right_i2c()
        {
            switch(phase)
            {
                case right_phase::encoder:
                    handle_right_encoder();
                    break;

                case right_phase::imu:
                    handle_imu();
                    break;
            }
        }

        /**
         * @brief Sensor task
         */
        void task(void *)
        {
            i2c::init(xTaskGetCurrentTaskHandle());     // sensor task 是唯一 I2C owner

            mpu6050::init();
            next_imu_time_us = sys_time::get_us_tick() + MPU6050_PERIOD_US;

            as5600::start_left_read();
            as5600::start_right_read();

            ESP_LOGI(TAG, "sensor started");

            while(true)
            {
                uint32_t notification = 0;

                xTaskNotifyWait(
                    0,
                    UINT32_MAX,
                    &notification,
                    portMAX_DELAY);

                if(notification & i2c::left_notify_bit())
                {
                    handle_left_i2c();
                }

                if(notification & i2c::right_notify_bit())
                {
                    handle_right_i2c();
                }
            }
        }
    }

    bool init()
    {
        if(started){return true;}

        const BaseType_t result =
            xTaskCreatePinnedToCore(
                task,
                "sensor",
                TASK_STACK,
                nullptr,
                TASK_PRIORITY,
                nullptr,
                TASK_CORE);

        if(result != pdPASS){return false;}
        started = true;

        return true;
    }

    bool ready()
    {
        bool value;

        portENTER_CRITICAL(&package_lock);

        value =
            left_valid &&
            right_valid &&
            imu_valid;

        portEXIT_CRITICAL(&package_lock);

        return value;
    }

    bool get_package(package &snapshot)
    {
        bool valid;

        portENTER_CRITICAL(&package_lock);

        valid =
            left_valid &&
            right_valid &&
            imu_valid;

        if(valid){snapshot = latest_package;}

        portEXIT_CRITICAL(&package_lock);

        return valid;
    }
}
