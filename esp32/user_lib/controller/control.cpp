#include "control.h"

#include "balance.h"
#include "hw/leg_servo.h"
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
        constexpr float ARM_SPEED_M_S = 0.1f;
        constexpr uint32_t ARM_TICKS = 100;
        constexpr int16_t LEFT_POSITION = 2088;
        constexpr int16_t RIGHT_POSITION = 2008;
        constexpr float RAD_PER_COUNT = 6.2831853f / 4096.0f;
        constexpr float PI = 3.14159265f;

        balance::config settings;
        motor::directions motor_directions;
        bool started = false;

        /**
         * @brief 根据旧机构标定曲线估计腿长
         *
         * @param[in] position_rad 舵机位置，单位 rad
         *
         * @return 腿长，单位 m
         */
        float leg_height(float position_rad)
        {
            const float distance = fabsf(position_rad - PI) / RAD_PER_COUNT;
            return ((4.6289047954e-12f * distance - 9.3936274976e-08f) *
                distance + 1.5357902969e-04f) * distance +
                4.2041568108e-02f;
        }

        /**
         * @brief 将双腿移动到固定原地平衡姿态
         *
         * @param[out] height_m 到位后的平均腿长，单位 m
         *
         * @return true 双腿已到位
         * @return false 舵机命令或反馈失败
         */
        bool prepare_legs(float &height_m)
        {
            const leg_servo::command left{LEFT_POSITION, 450, 250};
            const leg_servo::command right{RIGHT_POSITION, 450, 250};
            if(!leg_servo::set_target(left, right) ||
               !leg_servo::set_torque(true, true))
            {
                leg_servo::set_torque(false, false);
                return false;
            }

            const uint64_t deadline = sys_time::get_us_tick() + 2000000;
            while(sys_time::get_us_tick() < deadline)
            {
                leg_servo::state left_state;
                leg_servo::state right_state;
                if(leg_servo::read_feedback(left_state, right_state) &&
                   fabsf(left_state.position_rad - LEFT_POSITION * RAD_PER_COUNT) <
                       20.0f * RAD_PER_COUNT &&
                   fabsf(right_state.position_rad - RIGHT_POSITION * RAD_PER_COUNT) <
                       20.0f * RAD_PER_COUNT)
                {
                    height_m = (leg_height(left_state.position_rad) +
                        leg_height(right_state.position_rad)) * 0.5f;
                    return true;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            leg_servo::set_torque(false, false);
            return false;
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
                const float speed = -(
                    static_cast<float>(motor_directions.left) *
                        snapshot.left_encoder.speed_mrad_s +
                    static_cast<float>(motor_directions.right) *
                        snapshot.right_encoder.speed_mrad_s) *
                    0.0005f * settings.wheel_radius_m;
                const bool valid = fresh && isfinite(pitch) &&
                    isfinite(pitch_rate) && isfinite(speed);

                if(engaged && (!valid || fabsf(pitch) > TRIP_PITCH_RAD))
                {
                    engaged = false;
                    tripped = true;
                }

                if(!engaged)
                {
                    balance::reset();
                    motor::set_target(0, 0, false);
                    if(!tripped && valid && fabsf(pitch) < ARM_PITCH_RAD &&
                       fabsf(speed) < ARM_SPEED_M_S)
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
                    const float torque_mNm = balance::step(pitch, pitch_rate,
                        speed, PERIOD_MS * 0.001f) * 1000.0f;
                    if(!isfinite(torque_mNm))
                    {
                        engaged = false;
                        tripped = true;
                        motor::set_target(0, 0, false);
                    }
                    else
                    {
                        const float limited = fmaxf(-settings.max_torque_mNm,
                            fminf(settings.max_torque_mNm, torque_mNm));
                        const int32_t target_mNm = static_cast<int32_t>(roundf(limited));
                        motor::set_target(target_mNm, target_mNm, true);
                    }
                }

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PERIOD_MS));
            }
        }
    }

    /**
     * @brief 准备固定腿姿并启动原地平衡任务
     *
     * @return true 平衡任务已启动
     * @return false 腿部准备或任务创建失败
     */
    bool init()
    {
        if(started){return true;}
        motor::set_target(0, 0, false);
        motor_directions = motor::get_directions();
        if(motor_directions.left == 0 || motor_directions.right == 0)
        {
            return false;
        }

        float height_m = 0.0f;
        if(!prepare_legs(height_m)){return false;}
        balance::init(settings, height_m);

        if(xTaskCreatePinnedToCore(task, "control", 4096, nullptr, 4,
                nullptr, 0) != pdPASS)
        {
            leg_servo::set_torque(false, false);
            return false;
        }

        started = true;
        return true;
    }
}
