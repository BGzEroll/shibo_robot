#include "control.h"

#include "balance.h"
#include "leg.h"
#include "hw/gamepad.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "sys_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <algorithm>
#include <atomic>
#include <math.h>
#include <stdint.h>

namespace control
{
    namespace
    {
        constexpr uint32_t PERIOD_MS = 1;
        constexpr uint64_t ENCODER_TIMEOUT_US = 5000;
        constexpr uint64_t IMU_TIMEOUT_US = 15000;
        constexpr float TRIP_PITCH_RAD = 0.5f;
        constexpr float MAX_LINEAR_M_S = 0.6f;
        constexpr float MAX_YAW_RAD_S = 2.0f;
        constexpr float AXIS_DEADBAND = 0.05f;

        enum class mode : uint8_t {boot, balance, sit, stand, jump, recover, stop};
        enum class phase : uint8_t
        {
            prepare, moving, done, calibrate, push, fly, land
        };

        struct runtime
        {
            mode current = mode::boot;
            phase sit = phase::prepare;
            uint64_t phase_us = 0;
            uint64_t stable_us = 0;
            bool leg_torque_on = false;
            phase jump = phase::prepare;
            int8_t linear_dir = 0;
            int8_t turn_dir = 0;
            float target_yaw = 0.0f;
            bool airborne_seen = false;
            bool calibration_sent = false;
            float linear_ref = 0.0f;
            float yaw_ref = 0.0f;
        };

        // 可选行为只需在此处产生高层目标；电机输出仍由本模块控制。
        struct intent
        {
            float linear_m_s = 0.0f;
            float yaw_rad_s = 0.0f;
        };

        balance::config settings;
        motor::directions motor_directions;
        portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
        status latest_status;
        bool started = false;
        std::atomic<bool> sit_ready{false};
        std::atomic<bool> calibration_requested{false};

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
         * @brief 摇杆死区内归零，死区外重新映射到满量程
         *
         * @param[in] value 摇杆原始归一化值
         *
         * @return 应用死区后的归一化值
         */
        float axis(float value)
        {
            if(fabsf(value) <= AXIS_DEADBAND){return 0.0f;}
            return copysignf((fabsf(value) - AXIS_DEADBAND) /
                (1.0f - AXIS_DEADBAND), value);
        }

        /**
         * @brief 将手柄左摇杆转换为行走和转向目标
         *
         * @param[in] pad 手柄输入快照
         *
         * @return 行走和转向目标
         */
        intent manual_intent(const gamepad::state &pad)
        {
            const float linear = axis(pad.axes[1]);
            return {
                linear * MAX_LINEAR_M_S *
                    (linear < 0.0f ? 0.8f : 1.0f),
                -axis(pad.axes[0]) * MAX_YAW_RAD_S
            };
        }

        /**
         * @brief 将目标偏航角误差归一到半圈内
         *
         * @param[in] target 目标角度，单位 rad
         * @param[in] current 当前角度，单位 rad
         *
         * @return 最短角度误差，单位 rad
         */
        float yaw_error(float target, float current)
        {
            float error = target - current;
            while(error > 3.1415927f){error -= 6.2831853f;}
            while(error < -3.1415927f){error += 6.2831853f;}
            return error;
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
            runtime run;
            bool tripped = false;
            arm_state trip_reason = arm_state::tripped_sensor;
            uint64_t last_leg_feedback_us = 0;

            while(true)
            {
                const uint64_t cycle_us = sys_time::get_us_tick();
                const bool leg_cycle = cycle_us - last_leg_feedback_us >= 20000;
                if(leg_cycle)
                {
                    leg::refresh();
                    last_leg_feedback_us = cycle_us;
                }
                sensor::package snapshot;
                const bool imu_ready = sensor::get_package(snapshot);
                gamepad::state pad;
                const bool pad_ready = gamepad::get_state(pad);
                const uint64_t now_us = sys_time::get_us_tick();

                uint16_t pressed = 0;
                if(pad_ready)
                {
                    if(last_pad_ready && pad.session == last_session)
                    {
                        pressed = pad.buttons & ~last_buttons;
                    }
                    last_session = pad.session;
                    last_buttons = pad.buttons;
                }
                last_pad_ready = pad_ready;

                const bool fresh = imu_ready && leg::ready(now_us) &&
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
                const float acceleration = sqrtf(
                    snapshot.imu.acc[0] * snapshot.imu.acc[0] +
                    snapshot.imu.acc[1] * snapshot.imu.acc[1] +
                    snapshot.imu.acc[2] * snapshot.imu.acc[2]);
                const leg::contact contact = leg::update_contact(acceleration,
                    run.current == mode::balance, now_us);
                const float speed = -(
                    static_cast<float>(motor_directions.left) *
                        snapshot.left_encoder.speed_mrad_s +
                    static_cast<float>(motor_directions.right) *
                        snapshot.right_encoder.speed_mrad_s) *
                    0.001f *
                    0.5f *
                    settings.wheel_radius_m;

                if(run.current != mode::boot && run.current != mode::stop &&
                    (!fresh || !pad_ready ||
                    (run.current != mode::sit && run.current != mode::stand &&
                        fabsf(pitch) > TRIP_PITCH_RAD)))
                {
                    run.current = mode::stop;
                    sit_ready = false;
                    tripped = true;
                    trip_reason = !fresh ? arm_state::tripped_sensor :
                        !pad_ready ? arm_state::tripped_gamepad :
                        arm_state::tripped_pitch;
                }

                if(pressed & gamepad::button::START)
                {
                    run.current = mode::stop;
                    sit_ready = false;
                    tripped = false;
                }

                if(run.current == mode::boot || run.current == mode::stop)
                {
                    balance::reset();
                    run.linear_ref = 0.0f;
                    run.yaw_ref = 0.0f;
                    motor::set_target(0, 0, false);

                    if((pressed & gamepad::button::RB) &&
                        !(pressed & gamepad::button::START) && fresh && pad_ready)
                    {
                        if(!run.leg_torque_on)
                        {
                            leg::torque(true);
                            run.leg_torque_on = true;
                        }
                        leg::reset();
                        leg::pose(2088, 2008, 450, 250);
                        run.current = mode::stand;
                        run.phase_us = now_us;
                        tripped = false;
                    }
                }
                else if(run.current == mode::sit)
                {
                    const uint64_t elapsed = now_us - run.phase_us;
                    if(run.sit == phase::prepare)
                    {
                        motor::set_target(0, 0, false);
                        leg_servo::state left;
                        leg_servo::state right;
                        leg::get_feedback(left, right);
                        if((fabsf(left.position_rad - 3.1415927f) < 0.08f &&
                            fabsf(right.position_rad - 3.1415927f) < 0.08f) ||
                            elapsed >= 800000)
                        {
                            run.sit = phase::moving;
                            run.phase_us = now_us;
                        }
                    }
                    else if(run.sit == phase::moving)
                    {
                        if(fabsf(pitch) >= 0.25f || elapsed >= 1500000)
                        {
                            motor::set_target(0, 0, false);
                            run.sit = phase::done;
                            run.phase_us = now_us;
                            sit_ready = true;
                        }
                        else{motor::set_target(-3, -3, true);}
                    }
                    else if(run.sit == phase::done)
                    {
                        motor::set_target(0, 0, false);
                        if(calibration_requested.exchange(false))
                        {
                            run.sit = phase::calibrate;
                            run.phase_us = now_us;
                            run.calibration_sent = false;
                            sit_ready = false;
                        }
                        if(elapsed >= 10000000 && run.leg_torque_on)
                        {
                            leg::torque(false);
                            run.leg_torque_on = false;
                        }
                        if(run.sit == phase::done &&
                            (pressed & gamepad::button::RB))
                        {
                            sit_ready = false;
                            leg::torque(true);
                            run.leg_torque_on = true;
                            leg::reset();
                            leg::pose(2088, 2008, 450, 250);
                            run.current = mode::stand;
                            run.phase_us = now_us;
                        }
                    }
                    else if(run.sit == phase::calibrate)
                    {
                        motor::set_target(0, 0, false);
                        if(elapsed >= 500000 && run.leg_torque_on)
                        {
                            leg::torque(false);
                            run.leg_torque_on = false;
                        }
                        if(elapsed >= 2000000 && !run.calibration_sent)
                        {
                            leg_servo::calibrate_middle(leg_servo::side::left);
                            leg_servo::calibrate_middle(leg_servo::side::right);
                            run.calibration_sent = true;
                        }
                        if(elapsed >= 2500000)
                        {
                            run.sit = phase::done;
                            run.phase_us = now_us;
                            sit_ready = true;
                        }
                    }
                }
                else if(run.current == mode::stand)
                {
                    motor::set_target(0, 0, false);
                    if(now_us - run.phase_us >= 350000)
                    {
                        balance::reset();
                        run.current = mode::recover;
                        run.phase_us = now_us;
                        run.stable_us = 0;
                    }
                }
                else
                {
                    if(run.current == mode::balance &&
                        (pressed & gamepad::button::LS)){leg::reset();}
                    if(run.current == mode::balance &&
                        (pressed & gamepad::button::LB))
                    {
                        run.current = mode::sit;
                        run.linear_ref = 0.0f;
                        run.yaw_ref = 0.0f;
                        sit_ready = false;
                        run.sit = phase::prepare;
                        run.phase_us = now_us;
                        leg::pose(2088, 2008, 450, 250);
                        motor::set_target(0, 0, false);
                    }
                    else if(run.current == mode::balance &&
                        (pressed & (gamepad::button::RS | gamepad::button::Y |
                            gamepad::button::A | gamepad::button::X |
                            gamepad::button::B)))
                    {
                        run.current = mode::jump;
                        run.linear_ref = 0.0f;
                        run.yaw_ref = 0.0f;
                        run.jump = phase::prepare;
                        run.phase_us = now_us;
                        run.linear_dir = (pressed & gamepad::button::Y) ? 1 :
                            (pressed & gamepad::button::A) ? -1 : 0;
                        run.turn_dir = (pressed & gamepad::button::X) ? 1 :
                            (pressed & gamepad::button::B) ? -1 : 0;
                        run.target_yaw = snapshot.imu.angle[2] +
                            run.turn_dir * 1.5707963f;
                        run.airborne_seen = false;
                        balance::reset();
                        leg::pose(2148, 1948, 450, 250);
                    }
                    if(run.current != mode::sit)
                    {
                        if(leg_cycle && (run.current == mode::balance ||
                            run.current == mode::recover))
                        {
                            leg::control(snapshot.imu.angle[0],
                                run.current == mode::balance ? pad.buttons : 0);
                        }
                        if(run.current == mode::jump)
                        {
                            const uint64_t elapsed = now_us - run.phase_us;
                            if(contact == leg::contact::airborne)
                            {
                                run.airborne_seen = true;
                            }
                            if(run.jump == phase::prepare)
                            {
                                run.jump = phase::push;
                                run.phase_us = now_us;
                            }
                            else if(run.jump == phase::push &&
                                ((elapsed >= 80000 &&
                                    contact == leg::contact::airborne) ||
                                elapsed >= static_cast<uint64_t>(run.linear_dir > 0 ?
                                    650000 : run.linear_dir < 0 ? 700000 : 200000)))
                            {
                                run.airborne_seen = contact == leg::contact::airborne;
                                leg::pose(2518, 1578, 0, 0);
                                run.jump = phase::fly;
                                run.phase_us = now_us;
                            }
                            else if(run.jump == phase::fly &&
                                ((run.airborne_seen && elapsed >= 40000 &&
                                    contact == leg::contact::ground) ||
                                elapsed >= 130000))
                            {
                                leg::pose(2148, 1948, 0, 0);
                                run.jump = phase::land;
                                run.phase_us = now_us;
                            }
                            else if(run.jump == phase::land &&
                                ((elapsed >= 100000 && fabsf(pitch) < 0.18f &&
                                    fabsf(pitch_rate) < 1.6f) ||
                                elapsed >= 260000))
                            {
                                balance::reset();
                                run.current = mode::recover;
                                run.phase_us = now_us;
                                run.stable_us = 0;
                            }
                        }
                        balance::reference target;
                        if(run.current == mode::balance)
                        {
                            const intent command = manual_intent(pad);
                            const float linear_step = std::clamp(
                                (command.linear_m_s - run.linear_ref) * 0.041f,
                                -0.0016f, 0.0016f);
                            run.linear_ref = command.linear_m_s == 0.0f ?
                                0.0f : run.linear_ref + linear_step;
                            run.yaw_ref += (command.yaw_rad_s - run.yaw_ref) *
                                0.105f;
                            target.linear_m_s = run.linear_ref;
                            target.yaw_rad_s = run.yaw_ref;
                        }
                        else if(run.current == mode::jump)
                        {
                            target.linear_feedback = run.jump == phase::push &&
                                run.linear_dir != 0;
                            target.yaw_feedback = run.turn_dir != 0 ||
                                run.linear_dir != 0;
                            if(run.jump == phase::push && run.linear_dir)
                            {
                                const float speed = run.linear_dir > 0 ? 0.4f : 0.34f;
                                const float ramp_us = run.linear_dir > 0 ?
                                    160000.0f : 240000.0f;
                                target.linear_m_s = run.linear_dir * speed *
                                    std::clamp((now_us - run.phase_us) / ramp_us,
                                        0.0f, 1.0f);
                            }
                            if(target.yaw_feedback)
                            {
                                float kp = run.turn_dir ? 1.0f : 3.0f;
                                float feedforward = 0.0f;
                                float limit = run.turn_dir ? 0.6f : 1.8f;
                                if(run.turn_dir && run.jump == phase::push)
                                {
                                    kp = 1.4f;
                                    feedforward = 1.2f;
                                    limit = 1.8f;
                                }
                                else if(run.turn_dir && run.jump == phase::fly)
                                {
                                    kp = 2.0f;
                                    feedforward = 6.4f;
                                    limit = 6.4f;
                                }
                                else if(run.turn_dir && run.jump == phase::land)
                                {
                                    kp = 0.35f;
                                    limit = 0.4f;
                                }
                                target.yaw_rad_s = std::clamp(
                                    run.turn_dir * feedforward + kp * yaw_error(
                                        run.target_yaw, snapshot.imu.angle[2]),
                                    -limit, limit);
                            }
                        }
                        const balance::output torque = balance::step(
                            leg::height_m(),
                            pitch,
                            pitch_rate,
                            speed,
                            yaw_rate,
                            PERIOD_MS * 0.001f, target);

                        const float temporary_scale = 0.1f * (2.0f / 3.0f); // q_test 临时缩放
                        const float blend = run.current == mode::recover ?
                            std::clamp((now_us - run.phase_us) / 220000.0f,
                                0.0f, 1.0f) : 1.0f;
                        const float left_mNm = torque.left_Nm * temporary_scale * blend * 1000.0f;
                        const float right_mNm = torque.right_Nm * temporary_scale * blend * 1000.0f;

                        motor::set_target(
                            static_cast<int32_t>(roundf(left_mNm)),
                            static_cast<int32_t>(roundf(right_mNm)),
                            true);

                        if(run.current == mode::recover)
                        {
                            if(fabsf(pitch) < 0.16f && fabsf(pitch_rate) < 1.2f)
                            {
                                if(!run.stable_us){run.stable_us = now_us;}
                            }
                            else{run.stable_us = 0;}
                            if(run.stable_us && now_us - run.stable_us >= 140000)
                            {
                                balance::reset();
                                run.linear_ref = 0.0f;
                                run.yaw_ref = 0.0f;
                                run.current = mode::balance;
                            }
                            else if(now_us - run.phase_us >= 2500000)
                            {
                                run.current = mode::stop;
                                motor::set_target(0, 0, false);
                            }
                        }
                    }
                }

                arm_state state = arm_state::wait_button;
                if(tripped){state = trip_reason;}
                else if(run.current != mode::boot && run.current != mode::stop)
                {
                    state = arm_state::active;
                }
                else if(!fresh){state = arm_state::wait_sensor;}
                else if(!pad_ready){state = arm_state::wait_gamepad;}
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

    /**
     * @brief 在坐下并停止轮电机后请求左右腿舵机中位校准
     *
     * @return true 已提交请求
     * @return false 当前不在坐下稳定阶段
     */
    bool request_leg_calibration()
    {
        if(!sit_ready){return false;}
        calibration_requested = true;
        return true;
    }
}
