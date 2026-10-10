#include "control.h"

#include "action.h"
#include "balance.h"
#include "input.h"
#include "config.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "leg.h"
#include "hw/battery.h"
#include "sys_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

namespace control
{
    namespace
    {
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        status latest;
        bool maintenance = false;
        motor::directions directions;

        /**
         * @brief 检查反馈时间戳是否处于有效窗口
         *
         * @param[in] timestamp 反馈时间戳，单位 us
         * @param[in] now 当前时间，单位 us
         * @param[in] timeout 允许的反馈间隔，单位 us
         *
         * @return true 反馈有效且未超时
         * @return false 时间戳无效或反馈超时
         */
        bool fresh(uint64_t timestamp, uint64_t now, uint64_t timeout)
        {
            return timestamp != 0 && now >= timestamp && now - timestamp <= timeout;
        }

        /**
         * @brief 执行 1 ms 平衡循环和 10 ms 输入、动作更新
         */
        void task(void *)
        {
            const config::settings &settings = config::get();

            action::init();
            input_router::init();
            action::output actions;
            control_input input;

            uint64_t previous_us = sys_time::get_us_tick();
            uint64_t action_time_us = previous_us;
            uint64_t upright_since_us = 0;

            bool faulted = false;
            arm_state fault_reason = arm_state::tripped_sensor;
            bool was_enabled = false;
            TickType_t wake = xTaskGetTickCount();

            while(true)
            {
                const uint64_t now = sys_time::get_us_tick();
                const float dt = (now - previous_us) * 1.0e-6f;
                previous_us = now;
                const bool timed_out = dt <= 0.0f || dt > 0.005f;
                const bool action_due = now - action_time_us >= 10000;
                if(action_due){input_router::update(input);}

                sensor::package snapshot;
                const bool imu_ready = sensor::get_package(snapshot);
                const leg::package legs = leg::get();
                const battery::state battery = battery::get();

                status next;
                feedback &measured = next.measured;
                measured.pitch_rad = snapshot.imu.angle[1];
                measured.pitch_rate = snapshot.imu.gyro[1];
                measured.yaw_angle = snapshot.imu.angle[2];
                measured.yaw_rate = snapshot.imu.gyro[2];
                measured.roll_angle = snapshot.imu.angle[0];
                measured.speed_m_s = (settings.left_wheel_direction * directions.left *
                    snapshot.left_encoder.speed_mrad_s + settings.right_wheel_direction *
                    directions.right * snapshot.right_encoder.speed_mrad_s) * 0.0005f *
                    settings.balance.wheel_radius_m;
                measured.avg_leg_height = legs.height_m;

                const bool sensors_ready = imu_ready && fresh(snapshot.imu.timestamp_us, now, 15000) &&
                    fresh(snapshot.left_encoder.timestamp_us, now, 5000) &&
                    fresh(snapshot.right_encoder.timestamp_us, now, 5000);

                const bool leg_ready = legs.left.valid && legs.right.valid &&
                    fresh(legs.left.timestamp_us, now, 100000) && fresh(legs.right.timestamp_us, now, 100000) &&
                    !legs.io_failed && legs.left.status_bits == 0 && legs.right.status_bits == 0;
                const bool input_ready = input.fresh &&
                    static_cast<uint32_t>(now - input.timestamp_us) <= 250000;

                const bool upright_now = sensors_ready && !timed_out &&
                    fabsf(measured.pitch_rad - settings.balance.pitch_offset_rad) < settings.arm_pitch_rad;
                if(!upright_now){upright_since_us = 0;}
                else if(upright_since_us == 0){upright_since_us = now;}
                next.upright_ms = upright_now ?
                    std::min<uint64_t>((now - upright_since_us) / 1000, 100) : 0;
                const bool pitch_trip =
                    fabsf(measured.pitch_rad - settings.balance.pitch_offset_rad) > settings.trip_pitch_rad;

                portENTER_CRITICAL(&lock);
                const bool configuring = maintenance;
                portEXIT_CRITICAL(&lock);
                const bool ready = !configuring && input_ready && sensors_ready && leg_ready &&
                    battery.valid && !timed_out && !(was_enabled && pitch_trip);
                if(!ready && was_enabled)
                {
                    faulted = true;
                    fault_reason = pitch_trip ? arm_state::tripped_pitch : arm_state::tripped_sensor;
                }

                if(action_due)
                {
                    const uint32_t tick_ms = (now - action_time_us) / 1000;
                    action_time_us = now;
                    if(actions.current_mode == mode::STOP && (input.pressed & buttons::RB))
                    {
                        // 启动预留和配置锁定共用锁，避免停止检查与起身请求交错。
                        portENTER_CRITICAL(&lock);
                        const bool can_start = ready && !maintenance && !battery.low &&
                            next.upright_ms == 100 && !(input.pressed & buttons::START);
                        if(can_start)
                        {
                            faulted = false;
                            latest.mode = mode::BOOT;
                        }
                        else{input.pressed &= ~buttons::RB;}
                        portEXIT_CRITICAL(&lock);
                    }
                    if(ready && !faulted)
                    {
                        host::vision_measurement vision;
                        host::get_vision(vision);
                        actions = action::step(input, measured, legs, vision, battery.low, tick_ms);
                    }
                }

                if(!ready || faulted)
                {
                    if(actions.current_mode != mode::STOP){actions = action::stop();}
                    actions.command = {};
                }

                const bool enable = ready && !faulted && !pitch_trip &&
                    actions.command.mode != balance_mode::OFF && (was_enabled || next.upright_ms == 100);
                balance::output torque;
                if(enable)
                {
                    torque = balance::step(measured.avg_leg_height, measured.pitch_rad, measured.pitch_rate,
                        measured.speed_m_s, measured.yaw_rate, dt, actions.command, settings.motor.torque_limit_Nm);
                    actions.command.reset_reference = false;
                    if(!std::isfinite(torque.left_Nm) || !std::isfinite(torque.right_Nm))
                    {
                        faulted = true;
                        fault_reason = arm_state::tripped_sensor;
                        actions = action::stop();
                        balance::reset();
                        torque = {};
                    }
                }
                else{balance::reset();}

                next.enabled = enable && !faulted;
                next.left_torque_Nm = torque.left_Nm;
                next.right_torque_Nm = torque.right_Nm;
                next.mode = actions.current_mode;
                next.phase = actions.phase;
                next.calibration_sent = legs.calibration_sent;

                if(faulted){next.state = fault_reason;}
                else if(!input_ready){next.state = arm_state::wait_input;}
                else if(!sensors_ready || !leg_ready){next.state = arm_state::wait_sensor;}
                else if(!battery.valid || battery.low){next.state = arm_state::low_battery;}
                else if(next.enabled){next.state = arm_state::active;}
                else if(actions.current_mode == mode::STOP){next.state = arm_state::stopped;}
                else if(!upright_now){next.state = arm_state::wait_pitch;}
                else{next.state = arm_state::arming;}

                portENTER_CRITICAL(&lock);
                if(maintenance){next.enabled = false;}
                motor::set_target(settings.left_wheel_direction *
                    static_cast<int32_t>(roundf(torque.left_Nm * 1.0e6f)), settings.right_wheel_direction *
                    static_cast<int32_t>(roundf(torque.right_Nm * 1.0e6f)), next.enabled);
                latest = next;
                portEXIT_CRITICAL(&lock);

                was_enabled = next.enabled;
                vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
            }
        }
    }

    /**
     * @brief 初始化平衡控制并启动控制任务
     *
     * @param[in] hardware_ready 控制所需硬件是否初始化成功
     *
     * @return true 控制任务已启动
     * @return false 硬件未就绪、电机方向无效或任务创建失败
     */
    bool init(bool hardware_ready)
    {
        directions = motor::get_directions();
        if(!hardware_ready || directions.left == 0 || directions.right == 0)
        {
            latest.state = arm_state::init_failed;
            return false;
        }

        balance::init(config::get().balance);
        const bool started = xTaskCreatePinnedToCore(task, "control", 6144, nullptr, 4, nullptr, 0) == pdPASS;
        if(!started){latest.state = arm_state::init_failed;}
        return started;
    }

    /**
     * @brief 获取最新控制状态
     *
     * @return 控制任务最近一次发布的状态
     */
    status get_status()
    {
        portENTER_CRITICAL(&lock);
        const status snapshot = latest;
        portEXIT_CRITICAL(&lock);
        return snapshot;
    }

    /**
     * @brief 在小车停止时锁定控制，供参数保存和重启使用
     *
     * @return true 小车已停止并进入配置状态
     * @return false 小车尚未停止或仍在初始化
     */
    bool begin_configuration()
    {
        portENTER_CRITICAL(&lock);
        const bool stopped = latest.state != arm_state::preparing && !latest.enabled &&
            latest.mode == mode::STOP;
        if(stopped){maintenance = true;}
        portEXIT_CRITICAL(&lock);
        return stopped;
    }

    /**
     * @brief 解除配置锁定，允许响应新的启动请求
     */
    void end_configuration()
    {
        portENTER_CRITICAL(&lock);
        maintenance = false;
        portEXIT_CRITICAL(&lock);
    }
}
