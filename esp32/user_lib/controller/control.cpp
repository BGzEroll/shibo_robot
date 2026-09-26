#include "control.h"

#include "balance.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "sys_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdint.h>

namespace control
{
    namespace
    {
        constexpr uint32_t PERIOD_MS = 2;
        constexpr uint64_t ENCODER_TIMEOUT_US = 5000;
        constexpr uint64_t IMU_TIMEOUT_US = 15000;
        constexpr float ARM_PITCH_RAD = 0.15f;
        constexpr float TRIP_PITCH_RAD = 0.5f;
        constexpr uint32_t ARM_TICKS = 100;

        balance::config settings;
        motor::directions motor_directions;
        portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
        status latest_status;
        bool started = false;

        /**
         * @brief 发布本周期控制状态
         *
         * @param[in] next 新状态
         */
        void publish_status(const status &next)
        {
            portENTER_CRITICAL(&status_lock);
            latest_status = next;
            portEXIT_CRITICAL(&status_lock);
        }

        /**
         * @brief 在 core 0 上周期计算原地平衡力矩
         *
         * @param[in] arg RTOS 任务参数
         */
        void task(void *)
        {
            TickType_t last_wake = xTaskGetTickCount();
            uint32_t upright_ticks = 0;
            bool engaged = false;
            bool tripped = false;
            arm_state trip_reason = arm_state::tripped_sensor;

            while(true)
            {
                sensor::package snapshot;
                const bool imu_ready = sensor::get_package(snapshot);
                const uint64_t now_us = sys_time::get_us_tick();
                const bool fresh = imu_ready &&
                    snapshot.left_encoder.timestamp_us != 0 &&
                    snapshot.right_encoder.timestamp_us != 0 &&
                    now_us >= snapshot.imu.timestamp_us &&
                    now_us >= snapshot.left_encoder.timestamp_us &&
                    now_us >= snapshot.right_encoder.timestamp_us &&
                    now_us - snapshot.imu.timestamp_us <= IMU_TIMEOUT_US &&
                    now_us - snapshot.left_encoder.timestamp_us <= ENCODER_TIMEOUT_US &&
                    now_us - snapshot.right_encoder.timestamp_us <= ENCODER_TIMEOUT_US;

                const float pitch = snapshot.imu.angle[1];
                const float pitch_rate = snapshot.imu.gyro[1];
                const float yaw_rate = snapshot.imu.gyro[2];
                const float speed = -(
                    static_cast<float>(motor_directions.left) *
                        snapshot.left_encoder.speed_mrad_s +
                    static_cast<float>(motor_directions.right) *
                        snapshot.right_encoder.speed_mrad_s) *
                    0.0005f * settings.wheel_radius_m;
                const bool valid = fresh && isfinite(pitch) &&
                    isfinite(pitch_rate) && isfinite(yaw_rate) &&
                    isfinite(speed);

                if(engaged && (!valid || fabsf(pitch) > TRIP_PITCH_RAD))
                {
                    engaged = false;
                    tripped = true;
                    trip_reason = valid ? arm_state::tripped_pitch :
                        arm_state::tripped_sensor;
                }

                if(!engaged)
                {
                    balance::reset();
                    motor::set_target(0, 0, false);
                    if(!tripped && valid && fabsf(pitch) < ARM_PITCH_RAD)
                    {
                        if(++upright_ticks >= ARM_TICKS){engaged = true;}
                    }
                    else
                    {
                        upright_ticks = 0;
                    }
                }
                else
                {
                    const balance::output torque = balance::step(
                        settings.model_height_m, pitch, pitch_rate, speed, yaw_rate,
                        PERIOD_MS * 0.001f);
                    const float left_mNm = torque.left_nm * 1000.0f *
                        (2.0f / 3.0f);       // lqi 参数不太能匹配上，临时直接调整输出，之后再调
                    const float right_mNm = torque.right_nm * 1000.0f *
                        (2.0f / 3.0f);
                    if(!isfinite(left_mNm) || !isfinite(right_mNm))
                    {
                        engaged = false;
                        tripped = true;
                        trip_reason = arm_state::tripped_output;
                        motor::set_target(0, 0, false);
                    }
                    else
                    {
                        const float left_limited = fmaxf(-settings.max_torque_mNm,
                            fminf(settings.max_torque_mNm, left_mNm));
                        const float right_limited = fmaxf(-settings.max_torque_mNm,
                            fminf(settings.max_torque_mNm, right_mNm));
                        motor::set_target(
                            static_cast<int32_t>(roundf(left_limited)),
                            static_cast<int32_t>(roundf(right_limited)), true);
                    }
                }

                arm_state state = arm_state::arming;
                if(tripped){state = trip_reason;}
                else if(engaged){state = arm_state::active;}
                else if(!valid){state = arm_state::wait_sensor;}
                else if(fabsf(pitch) >= ARM_PITCH_RAD)
                {
                    state = arm_state::wait_pitch;
                }
                publish_status({state, pitch, speed, upright_ticks * PERIOD_MS});

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
            }
        }
    }

    /**
     * @brief 初始化固定模型并启动原地平衡任务
     *
     * @return true 平衡任务已启动
     * @return false 电机方向无效或任务创建失败
     */
    bool init()
    {
        if(started){return true;}
        motor::set_target(0, 0, false);
        motor_directions = motor::get_directions();
        if(motor_directions.left == 0 || motor_directions.right == 0)
        {
            publish_status({arm_state::init_failed});
            return false;
        }

        balance::init(settings);

        if(xTaskCreatePinnedToCore(task, "control", 4096, nullptr, 4,
                nullptr, 0) != pdPASS)
        {
            publish_status({arm_state::init_failed});
            return false;
        }

        started = true;
        return true;
    }

    /**
     * @brief 获取最新平衡启动状态
     *
     * @return 控制任务最近一次发布的状态
     */
    status get_status()
    {
        portENTER_CRITICAL(&status_lock);
        const status snapshot = latest_status;
        portEXIT_CRITICAL(&status_lock);
        return snapshot;
    }
}
