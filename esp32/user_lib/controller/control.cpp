#include "control.h"

#include "balance.h"
#include "hw/gamepad.h"
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
        constexpr uint32_t PERIOD_MS = 1;
        constexpr uint64_t ENCODER_TIMEOUT_US = 5000;
        constexpr uint64_t IMU_TIMEOUT_US = 15000;
        constexpr float ARM_PITCH_RAD = 0.15f;
        constexpr float TRIP_PITCH_RAD = 0.5f;

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
            uint32_t last_session = 0;
            uint16_t last_buttons = 0;
            bool last_pad_ready = false;
            bool engaged = false;
            bool tripped = false;
            arm_state trip_reason = arm_state::tripped_sensor;

            while(true)
            {
                sensor::package snapshot;
                const bool imu_ready = sensor::get_package(snapshot);
                gamepad::state pad;
                const bool pad_ready = gamepad::get_state(pad);
                const uint64_t now_us = sys_time::get_us_tick();

                bool rb_pressed = false;
                if(pad_ready)
                {
                    if(last_pad_ready && pad.session == last_session)
                    {
                        rb_pressed = (pad.buttons & gamepad::button::RB) &&
                            !(last_buttons & gamepad::button::RB);
                    }
                    last_session = pad.session;
                    last_buttons = pad.buttons;
                }
                last_pad_ready = pad_ready;

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
                    0.001f *
                    0.5f *
                    settings.wheel_radius_m;

                if(engaged && (!fresh || !pad_ready ||
                    fabsf(pitch) > TRIP_PITCH_RAD))
                {
                    engaged = false;
                    tripped = true;
                    trip_reason = !fresh ? arm_state::tripped_sensor :
                        !pad_ready ? arm_state::tripped_gamepad :
                        arm_state::tripped_pitch;
                }

                if(!engaged)
                {
                    balance::reset();
                    motor::set_target(0, 0, false);

                    if(rb_pressed && fresh && pad_ready &&
                        fabsf(pitch) < ARM_PITCH_RAD)
                    {
                        engaged = true;
                        tripped = false;
                    }
                }
                else
                {
                    const balance::output torque = balance::step(
                        settings.model_height_m,
                        pitch,
                        pitch_rate,
                        speed,
                        yaw_rate,
                        PERIOD_MS * 0.001f);

                    const float temporary_scale = 0.1f * (2.0f / 3.0f); // q_test 临时缩放
                    const float left_mNm = torque.left_Nm * temporary_scale * 1000.0f;
                    const float right_mNm = torque.right_Nm * temporary_scale * 1000.0f;

                    motor::set_target(
                        static_cast<int32_t>(roundf(left_mNm)),
                        static_cast<int32_t>(roundf(right_mNm)),
                        true);
                }

                arm_state state = arm_state::wait_button;
                if(tripped){state = trip_reason;}
                else if(engaged){state = arm_state::active;}
                else if(!fresh){state = arm_state::wait_sensor;}
                else if(!pad_ready){state = arm_state::wait_gamepad;}
                else if(fabsf(pitch) >= ARM_PITCH_RAD)
                {
                    state = arm_state::wait_pitch;
                }

                publish_status({state, pitch, speed});

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

        if(xTaskCreatePinnedToCore(
            task,
            "control",
            4096,
            nullptr,
            4,
            nullptr,
            0) != pdPASS)
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
