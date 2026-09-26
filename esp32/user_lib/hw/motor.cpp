#include "motor.h"

#include "hw/sensor.h"
#include "sys_time.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace motor
{
    namespace
    {
        constexpr int32_t Q15_ONE = 32768;
        constexpr int32_t OUTPUT_LIMIT = 18919;
        constexpr int32_t POLE_PAIRS = 7;
        constexpr uint32_t PWM_PEAK = 1600;     // 80 MHz / (2 * 25 kHz)
        constexpr uint64_t ENCODER_TIMEOUT_US = 5000;
        constexpr uint64_t COMMAND_TIMEOUT_US = 20000;
        constexpr uint32_t MAX_PREDICT_US = 1000;
        constexpr uint32_t DIRECTION_STEPS = 500;
        constexpr int32_t DIRECTION_MIN_COUNT = 41;
        constexpr uint32_t ZERO_SAMPLES = 32;

        // 浮点仅用于编译期生成整数系数。
        constexpr double PHASE_RESISTANCE = 12.27166;
        constexpr double KT_KE = 0.0796;
        constexpr double BUS_VOLTAGE = 7.4;
        constexpr float ALIGNMENT_VOLTAGE = 1.7f;
        constexpr int32_t ALIGNMENT_UQ = static_cast<int32_t>(
            ALIGNMENT_VOLTAGE / BUS_VOLTAGE * Q15_ONE + 0.5);
        constexpr int32_t TORQUE_GAIN_Q10 = static_cast<int32_t>(
            PHASE_RESISTANCE / KT_KE / BUS_VOLTAGE * Q15_ONE / 1000.0 * 1024.0 + 0.5);
        constexpr int32_t BEMF_GAIN_Q14 = static_cast<int32_t>(
            KT_KE / BUS_VOLTAGE * Q15_ONE / 1000.0 * 16384.0 + 0.5);

        struct context
        {
            gpio_num_t enable_pin;
            gpio_num_t pwm_pins[3];
            sensor::encoder_data sensor::package::*encoder;
            mcpwm_cmpr_handle_t comparators[3] = {};
            int8_t direction = 0;
            uint16_t zero_phase = 0;
            uint64_t last_encoder_us = 0;
            bool enabled = false;
        };

        context left{GPIO_NUM_22, {GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25},
            &sensor::package::left_encoder};
        context right{GPIO_NUM_12, {GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14},
            &sensor::package::right_encoder};

        struct command
        {
            int32_t left_mNm = 0;
            int32_t right_mNm = 0;
            uint64_t timestamp_us = 0;
            bool enabled = false;
        };

        portMUX_TYPE command_lock = portMUX_INITIALIZER_UNLOCKED;
        command target;
        mcpwm_timer_handle_t timer = nullptr;
        bool started = false;

        /**
         * @brief Q15 正弦查表与线性插值
         *
         * @param[in] phase 16 位电角度
         *
         * @return Q15 正弦值
         */
        int32_t lookup_sin(uint16_t phase)
        {
            static constexpr uint16_t table[65] =
            {
                0, 804, 1608, 2411, 3212, 4011, 4808, 5602,
                6393, 7180, 7962, 8740, 9512, 10279, 11039, 11793,
                12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531,
                18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595,
                23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791,
                27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957,
                30274, 30572, 30853, 31114, 31357, 31581, 31786, 31972,
                32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758,
                32768
            };

            const uint16_t index = phase >> 8;
            const int32_t fraction = phase & 0xFF;
            int32_t a, b;

            if(index < 64)
            {
                a = table[index];
                b = table[index + 1];
            }
            else if(index < 128)
            {
                a = table[128 - index];
                b = table[127 - index];
            }
            else if(index < 192)
            {
                a = -table[index - 128];
                b = -table[index - 127];
            }
            else
            {
                a = -table[256 - index];
                b = -table[255 - index];
            }

            return a + (((b - a) * fraction) >> 8);
        }

        /**
         * @brief 逆 Park、逆 Clarke 和零序注入并更新三相 PWM
         *
         * @param[in, out] motor 电机上下文
         * @param[in] uq Q15 归一化 q 轴电压
         * @param[in] phase 16 位电角度
         */
        void output(context &motor, int32_t uq, uint16_t phase)
        {
            if(uq > OUTPUT_LIMIT){uq = OUTPUT_LIMIT;}
            else if(uq < -OUTPUT_LIMIT){uq = -OUTPUT_LIMIT;}

            const int32_t alpha = -(lookup_sin(phase) * uq >> 15);
            const int32_t beta = lookup_sin(static_cast<uint16_t>(phase + 0x4000)) * uq >> 15;
            const int32_t phase_b = -(alpha >> 1) + (28378 * beta >> 15);
            const int32_t phase_c = -(alpha >> 1) - (28378 * beta >> 15);

            int32_t minimum = alpha;
            int32_t maximum = alpha;
            if(phase_b < minimum){minimum = phase_b;}
            if(phase_c < minimum){minimum = phase_c;}
            if(phase_b > maximum){maximum = phase_b;}
            if(phase_c > maximum){maximum = phase_c;}

            const int32_t offset = Q15_ONE / 2 - ((maximum + minimum) >> 1);
            const int32_t values[3] = {alpha + offset, phase_b + offset, phase_c + offset};

            for(uint32_t i = 0; i < 3; i++)
            {
                int32_t duty = values[i];
                if(duty < 0){duty = 0;}
                else if(duty > Q15_ONE){duty = Q15_ONE;}

                // 生成器在零点置高，向上比较时置低，向下比较时置高。
                uint32_t compare = (static_cast<uint32_t>(duty) * PWM_PEAK + 16384) >> 15;
                if(compare == 0){compare = 1;}
                else if(compare == PWM_PEAK){compare = PWM_PEAK - 1;}
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor.comparators[i], compare));
            }
        }

        /**
         * @brief 关闭驱动并将三相占空比恢复到中点
         *
         * @param[in, out] motor 电机上下文
         */
        void disable(context &motor)
        {
            if(!motor.enabled){return;}
            gpio_set_level(motor.enable_pin, 0);
            motor.enabled = false;
            output(motor, 0, 0);
        }

        /**
         * @brief 初始化双电机共用的 25 kHz 中心对齐 PWM
         *
         * @return true 初始化成功
         * @return false PWM 资源初始化失败
         */
        bool init_pwm()
        {
            gpio_set_level(left.enable_pin, 0);
            gpio_set_level(right.enable_pin, 0);
            gpio_config_t pins{};
            pins.pin_bit_mask =
                (static_cast<uint64_t>(1) << left.enable_pin) |
                (static_cast<uint64_t>(1) << right.enable_pin);
            pins.mode = GPIO_MODE_OUTPUT;
            if(gpio_config(&pins) != ESP_OK){return false;}

            mcpwm_timer_config_t timer_config{};
            timer_config.group_id = 0;
            timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
            timer_config.resolution_hz = 80000000;
            timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
            timer_config.period_ticks = PWM_PEAK * 2;
            if(mcpwm_new_timer(&timer_config, &timer) != ESP_OK){return false;}

            mcpwm_operator_config_t operator_config{};
            operator_config.group_id = 0;
            mcpwm_comparator_config_t compare_config{};
            compare_config.flags.update_cmp_on_tez = true;

            for(uint32_t phase = 0; phase < 3; phase++)
            {
                mcpwm_oper_handle_t oper = nullptr;
                if(mcpwm_new_operator(&operator_config, &oper) != ESP_OK ||
                   mcpwm_operator_connect_timer(oper, timer) != ESP_OK)
                {
                    return false;
                }

                context *motors[2] = {&left, &right};
                for(context *motor : motors)
                {
                    if(mcpwm_new_comparator(oper, &compare_config,
                            &motor->comparators[phase]) != ESP_OK ||
                       mcpwm_comparator_set_compare_value(
                            motor->comparators[phase], PWM_PEAK / 2) != ESP_OK)
                    {
                        return false;
                    }

                    mcpwm_generator_config_t gen_config{};
                    gen_config.gen_gpio_num = motor->pwm_pins[phase];
                    mcpwm_gen_handle_t gen = nullptr;
                    if(mcpwm_new_generator(oper, &gen_config, &gen) != ESP_OK ||
                       mcpwm_generator_set_action_on_timer_event(gen,
                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
                       mcpwm_generator_set_action_on_compare_event(gen,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                motor->comparators[phase], MCPWM_GEN_ACTION_LOW)) != ESP_OK ||
                       mcpwm_generator_set_action_on_compare_event(gen,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN,
                                motor->comparators[phase], MCPWM_GEN_ACTION_HIGH)) != ESP_OK)
                    {
                        return false;
                    }
                }
            }

            return mcpwm_timer_enable(timer) == ESP_OK &&
                mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) == ESP_OK;
        }

        /**
         * @brief 等待当前电机的新编码器样本
         *
         * @param[in, out] motor 电机上下文
         * @param[out] next 新编码器样本
         * @param[in] timeout_ms 最长等待时间
         *
         * @return true 收到新鲜样本
         * @return false 等待或编码器超时
         */
        bool wait_encoder(context &motor, sensor::encoder_data &next, uint32_t timeout_ms)
        {
            const uint64_t deadline =
                sys_time::get_us_tick() + static_cast<uint64_t>(timeout_ms) * 1000;
            while(sys_time::get_us_tick() < deadline)
            {
                sensor::package snapshot;
                sensor::get_package(snapshot);      // 返回值只表示 IMU 是否就绪。
                next = snapshot.*motor.encoder;
                const uint64_t now_us = sys_time::get_us_tick();
                if(next.timestamp_us != 0 &&
                   next.timestamp_us != motor.last_encoder_us &&
                   now_us >= next.timestamp_us &&
                   now_us - next.timestamp_us <= ENCODER_TIMEOUT_US)
                {
                    motor.last_encoder_us = next.timestamp_us;
                    return true;
                }
                if(motor.enabled && motor.last_encoder_us != 0 &&
                   now_us - motor.last_encoder_us > ENCODER_TIMEOUT_US)
                {
                    return false;
                }
                vTaskDelay(1);
            }
            return false;
        }

        /**
         * @brief 保持当前电角度并持续检查编码器
         *
         * @param[in, out] motor 电机上下文
         * @param[out] encoder 最后一个编码器样本
         * @param[in] duration_ms 保持时间
         *
         * @return true 持续收到新鲜样本
         * @return false 编码器超时
         */
        bool hold_encoder(context &motor, sensor::encoder_data &encoder,
            uint32_t duration_ms)
        {
            const uint64_t deadline =
                sys_time::get_us_tick() + static_cast<uint64_t>(duration_ms) * 1000;
            while(sys_time::get_us_tick() < deadline)
            {
                if(!wait_encoder(motor, encoder, 20)){return false;}
            }
            return true;
        }

        /**
         * @brief 扫描电角度并校准编码器方向与零电角
         *
         * @param[in, out] motor 电机上下文
         *
         * @return true 校准成功
         * @return false 编码器超时或方向扫描失败
         */
        bool calibrate(context &motor)
        {
            sensor::encoder_data encoder;
            if(!wait_encoder(motor, encoder, 100)){return false;}

            output(motor, ALIGNMENT_UQ, 0xC000);
            sys_time::delay_us(50);     // 等待三相比较值在 PWM 零点装载。
            gpio_set_level(motor.enable_pin, 1);
            motor.enabled = true;
            if(!hold_encoder(motor, encoder, 300)){return false;}

            const int32_t start_count = encoder.full_count;
            for(uint32_t step = 0; step <= DIRECTION_STEPS; step++)
            {
                const uint16_t phase = static_cast<uint16_t>(
                    0xC000 + step * 65536 / DIRECTION_STEPS);
                output(motor, ALIGNMENT_UQ, phase);
                vTaskDelay(pdMS_TO_TICKS(2));
                if(!wait_encoder(motor, encoder, 20)){return false;}
            }

            const int32_t forward_count = encoder.full_count;
            for(int32_t step = DIRECTION_STEPS; step >= 0; step--)
            {
                const uint16_t phase = static_cast<uint16_t>(
                    0xC000 + static_cast<uint32_t>(step) * 65536 / DIRECTION_STEPS);
                output(motor, ALIGNMENT_UQ, phase);
                vTaskDelay(pdMS_TO_TICKS(2));
                if(!wait_encoder(motor, encoder, 20)){return false;}
            }

            const int32_t forward_delta = forward_count - start_count;
            const int32_t reverse_delta = encoder.full_count - forward_count;
            if(forward_delta >= DIRECTION_MIN_COUNT && reverse_delta <= -DIRECTION_MIN_COUNT)
            {
                motor.direction = 1;
            }
            else if(forward_delta <= -DIRECTION_MIN_COUNT && reverse_delta >= DIRECTION_MIN_COUNT)
            {
                motor.direction = -1;
            }
            else
            {
                return false;
            }

            output(motor, ALIGNMENT_UQ, 0xC000);
            if(!hold_encoder(motor, encoder, 300)){return false;}

            int64_t sum = 0;
            for(uint32_t i = 0; i < ZERO_SAMPLES; i++)
            {
                if(!wait_encoder(motor, encoder, 20)){return false;}
                sum += static_cast<int64_t>(encoder.full_count) * 16;
            }

            motor.zero_phase = static_cast<uint16_t>(
                static_cast<int32_t>(motor.direction) * POLE_PAIRS * (sum / ZERO_SAMPLES));
            return true;
        }

        /**
         * @brief 根据新鲜编码器样本计算电压并更新当前电机
         *
         * @param[in, out] motor 电机上下文
         * @param[in] encoder 编码器样本
         * @param[in] torque_mNm 目标力矩，单位 mN·m
         * @param[in] now_us 当前时间，单位 us
         */
        void update(context &motor, const sensor::encoder_data &encoder,
            int32_t torque_mNm, uint64_t now_us)
        {
            const uint32_t age_us = static_cast<uint32_t>(
                now_us - encoder.timestamp_us > MAX_PREDICT_US ?
                MAX_PREDICT_US : now_us - encoder.timestamp_us);
            const int32_t advance = static_cast<int32_t>(
                (static_cast<int64_t>(encoder.speed_mrad_s) * age_us * 175) >> 24);
            const uint16_t mechanical = static_cast<uint16_t>(
                (static_cast<uint32_t>(encoder.full_count) << 4) + advance);
            const uint16_t electrical = static_cast<uint16_t>(
                motor.direction * POLE_PAIRS * static_cast<int32_t>(mechanical) - motor.zero_phase);

            int64_t uq =
                (static_cast<int64_t>(torque_mNm) * TORQUE_GAIN_Q10 >> 10) +
                (static_cast<int64_t>(motor.direction) * encoder.speed_mrad_s * BEMF_GAIN_Q14 >> 14);
            if(uq > OUTPUT_LIMIT){uq = OUTPUT_LIMIT;}
            else if(uq < -OUTPUT_LIMIT){uq = -OUTPUT_LIMIT;}

            output(motor, static_cast<int32_t>(uq), electrical);
            if(!motor.enabled)
            {
                sys_time::delay_us(50);
                gpio_set_level(motor.enable_pin, 1);
                motor.enabled = true;
            }
        }

        /**
         * @brief core 1 上持续运行的双电机 FOC 任务
         */
        void task(void *)
        {
            sensor::package snapshot;
            while(true)
            {
                sensor::get_package(snapshot);
                command current;
                portENTER_CRITICAL(&command_lock);
                current = target;
                portEXIT_CRITICAL(&command_lock);

                const uint64_t now_us = sys_time::get_us_tick();
                const bool fresh =
                    snapshot.left_encoder.timestamp_us != 0 &&
                    snapshot.right_encoder.timestamp_us != 0 &&
                    now_us >= snapshot.left_encoder.timestamp_us &&
                    now_us >= snapshot.right_encoder.timestamp_us &&
                    now_us - snapshot.left_encoder.timestamp_us <= ENCODER_TIMEOUT_US &&
                    now_us - snapshot.right_encoder.timestamp_us <= ENCODER_TIMEOUT_US;
                const bool active = fresh && current.enabled &&
                    now_us >= current.timestamp_us &&
                    now_us - current.timestamp_us <= COMMAND_TIMEOUT_US;

                if(active)
                {
                    update(left, snapshot.left_encoder, current.left_mNm, now_us);
                    update(right, snapshot.right_encoder, current.right_mNm, now_us);
                }
                else
                {
                    disable(left);
                    disable(right);
                }

                taskYIELD();
            }
        }
    }

    /**
     * @brief 初始化 PWM、校准双电机并启动 FOC 任务
     *
     * @return true 初始化成功
     * @return false PWM、校准或任务创建失败
     */
    bool init()
    {
        if(started){return true;}
        if(!init_pwm()){return false;}

        const bool left_ok = calibrate(left);
        disable(left);
        if(!left_ok){return false;}

        const bool right_ok = calibrate(right);
        disable(right);
        if(!right_ok){return false;}

        portENTER_CRITICAL(&command_lock);
        target = command{};
        portEXIT_CRITICAL(&command_lock);
        if(xTaskCreatePinnedToCore(task, "motor", 4096, nullptr, 5, nullptr, 1) != pdPASS)
        {return false;}

        started = true;
        return true;
    }

    /**
     * @brief 提交左右电机力矩目标与使能状态
     *
     * @param[in] left_mNm 左电机目标力矩，单位 mN·m
     * @param[in] right_mNm 右电机目标力矩，单位 mN·m
     * @param[in] enabled 是否使能输出
     */
    void set_target(int32_t left_mNm, int32_t right_mNm, bool enabled)
    {
        const uint64_t now_us = sys_time::get_us_tick();
        portENTER_CRITICAL(&command_lock);
        target.left_mNm = left_mNm;
        target.right_mNm = right_mNm;
        target.timestamp_us = now_us;
        target.enabled = enabled;
        portEXIT_CRITICAL(&command_lock);
    }
}
