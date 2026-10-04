#include "control.h"

#include "action.h"
#include "balance.h"
#include "input.h"
#include "config.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "leg.h"
#include "hw/battery.h"
#include "hw/aux_servo.h"
#include "io/host.h"
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
        bool started = false;
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
         * @brief 根据腿部位置计算质心到轮轴的距离
         *
         * @param[in] position_rad 腿部舵机位置，单位 rad
         *
         * @return 经质心修正的模型高度，单位 m
         */
        float height(float position_rad)
        {
            const config::settings &settings = config::get();
            const float count = fabsf(position_rad * (4096.0f / 6.28318530718f) - 2048.0f);
            const float *poly = settings.height_poly;
            const float leg_height = ((poly[0] * count + poly[1]) * count + poly[2]) * count + poly[3];
            return leg_height * settings.height_com_scale + settings.height_com_offset_m;
        }

        /**
         * @brief 执行 1 ms 平衡循环和 10 ms 输入、动作更新
         */
        void task(void *)
        {
            const config::settings &settings = config::get();
            action::state actions;
            action::leg_runtime legs;
            action::init(actions);
            input_router::init();
            balance_command command;
            control_input input;
            uint32_t ticks = 0;
            uint32_t upright = 0;
            uint64_t previous_us = sys_time::get_us_tick();
            float camera_angle = 90.0f;
            bool faulted = false;
            arm_state fault_reason = arm_state::tripped_sensor;
            bool was_enabled = false;
            TickType_t wake = xTaskGetTickCount();

            while(true)
            {
                const uint64_t now = sys_time::get_us_tick();
                const float dt = (now - previous_us) * 1.0e-6f;
                previous_us = now;

                sensor::package snapshot;
                const bool imu_ready = sensor::get_package(snapshot);
                const leg::package leg_state = leg::get();
                const battery::state battery_state = battery::get();

                status next;
                next.pitch_rad = snapshot.imu.angle[1];
                next.pitch_rate = snapshot.imu.gyro[1];
                next.yaw_angle = snapshot.imu.angle[2];
                next.yaw_rate = snapshot.imu.gyro[2];
                next.roll_angle = snapshot.imu.angle[0];
                next.speed_m_s = -(directions.left * snapshot.left_encoder.speed_mrad_s +
                    directions.right * snapshot.right_encoder.speed_mrad_s) * 0.0005f *
                    settings.balance.wheel_radius_m;
                next.avg_leg_height = settings.height_feedback ?
                    0.5f * (height(leg_state.left.position_rad) + height(leg_state.right.position_rad)) :
                    settings.balance.model_height_m;

                const bool sensors_ready = imu_ready && fresh(snapshot.imu.timestamp_us, now, 15000) &&
                    fresh(snapshot.left_encoder.timestamp_us, now, 5000) &&
                    fresh(snapshot.right_encoder.timestamp_us, now, 5000) &&
                    std::isfinite(next.pitch_rad) && std::isfinite(next.pitch_rate) &&
                    std::isfinite(next.yaw_rate) && std::isfinite(next.yaw_angle) &&
                    std::isfinite(next.roll_angle) && std::isfinite(next.avg_leg_height);

                const bool leg_ready = leg_state.left.valid && leg_state.right.valid &&
                    fresh(leg_state.left.timestamp_us, now, 100000) &&
                    fresh(leg_state.right.timestamp_us, now, 100000) &&
                    !leg_state.io_failed &&
                    leg_state.left.status_bits == 0 && leg_state.right.status_bits == 0;

                const bool upright_now = sensors_ready &&
                    fabsf(next.pitch_rad - settings.balance.pitch_offset_rad) < settings.arm_pitch_rad;
                upright = upright_now ? std::min<uint32_t>(upright + 1, 100) : 0;
                next.upright_ms = upright;

                portENTER_CRITICAL(&lock);
                const bool configuring = maintenance;
                portEXIT_CRITICAL(&lock);

                if((ticks++ % 10) == 0)
                {
                    input_router::update(actions.current_mode,
                        settings.max_linear_m_s, settings.max_yaw_rad_s, input);
                    if(input.action == action_request::BOOT && sensors_ready && leg_ready &&
                       upright == 100 && battery_state.valid && !battery_state.low && !configuring)
                    {
                        faulted = false;
                    }
                    else if(input.action == action_request::BOOT){input.action = action_request::NONE;}
                    if(battery_state.valid && battery_state.low &&
                       actions.current_mode == action::mode::BALANCE)
                    {
                        input.action = action_request::SIT;
                    }

                    host::vision_measurement vision;
                    const bool vision_ready = host::get_vision(vision);
                    action::context context{input, next, legs, settings.max_linear_m_s,
                        settings.max_yaw_rad_s, battery_state.valid, battery_state.low,
                        static_cast<int16_t>(leg_state.left.position_rad * (4096.0f / 6.28318530718f)),
                        static_cast<int16_t>(leg_state.right.position_rad * (4096.0f / 6.28318530718f)),
                        vision_ready, vision.dx, vision.dy, vision.sequence};

                    portENTER_CRITICAL(&lock);
                    const bool action_allowed = !maintenance && !faulted && input.fresh &&
                        sensors_ready && leg_ready && battery_state.valid;
                    if(action_allowed && input.action == action_request::BOOT)
                    {
                        latest.mode = static_cast<uint8_t>(action::mode::BOOT);
                    }
                    portEXIT_CRITICAL(&lock);
                    if(action_allowed)
                    {
                        command = action::step(actions, context, 10);
                    }

                    if(actions.current_mode != action::mode::KICK_PLACE &&
                       actions.current_mode != action::mode::KICK_RUN)
                    {
                        camera_angle = std::clamp(camera_angle + input.camera_direction * 1.2f, 0.0f, 180.0f);
                        aux_servo::set_camera(static_cast<uint16_t>(camera_angle));
                    }
                }

                const bool pitch_trip =
                    fabsf(next.pitch_rad - settings.balance.pitch_offset_rad) > settings.trip_pitch_rad;
                const bool timed_out = dt <= 0.0f || dt > 0.005f;
                // 在每个 1 ms 周期检查输入过期，不能依赖动作更新频率。
                const bool input_ready = input.fresh &&
                    static_cast<uint32_t>(now - input.timestamp_us) <= 250000;
                if(configuring || !input_ready || !sensors_ready || !leg_ready || timed_out ||
                   !battery_state.valid || (was_enabled && pitch_trip))
                {
                    if(was_enabled)
                    {
                        faulted = true;
                        fault_reason = pitch_trip ? arm_state::tripped_pitch : arm_state::tripped_sensor;
                    }
                    actions.current_mode = action::mode::STOP;
                    command = {};
                    leg::set_torque(0, 0);
                    balance::reset();
                }

                if(faulted){command = {};}

                const bool enable = !configuring && !faulted && input_ready && sensors_ready && leg_ready &&
                    battery_state.valid && !timed_out && !pitch_trip && command.mode != balance_mode::OFF &&
                    (was_enabled || upright == 100);

                balance::output torque;
                if(enable)
                {
                    torque = balance::step(next.avg_leg_height, next.pitch_rad, next.pitch_rate,
                        next.speed_m_s, next.yaw_rate, dt, command, settings.motor.torque_limit_Nm);
                    command.reset_reference = false;
                    command.reset_yaw_integral = false;
                    if(!std::isfinite(torque.left_Nm) || !std::isfinite(torque.right_Nm))
                    {
                        faulted = true;
                        torque = {};
                    }
                }
                else{balance::reset();}

                next.enabled = enable && !faulted;
                next.left_torque_Nm = torque.left_Nm;
                next.right_torque_Nm = torque.right_Nm;
                next.mode = static_cast<uint8_t>(actions.current_mode);
                next.phase = actions.phase;
                next.calibration_success = leg_state.calibrated;

                if(faulted){next.state = fault_reason;}
                else if(!input_ready){next.state = arm_state::wait_input;}
                else if(!sensors_ready || !leg_ready){next.state = arm_state::wait_sensor;}
                else if(!battery_state.valid || battery_state.low){next.state = arm_state::low_battery;}
                else if(next.enabled){next.state = arm_state::active;}
                else if(actions.current_mode == action::mode::STOP){next.state = arm_state::stopped;}
                else if(!upright_now){next.state = arm_state::wait_pitch;}
                else{next.state = arm_state::arming;}

                // 配置接口和最后一次使能决策共用锁，防止停止检查与启动交错。
                portENTER_CRITICAL(&lock);
                if(maintenance){next.enabled = false;}
                motor::set_target(static_cast<int32_t>(roundf(torque.left_Nm * 1.0e6f)),
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
        if(started){return true;}

        directions = motor::get_directions();
        if(!hardware_ready || directions.left == 0 || directions.right == 0)
        {
            latest.state = arm_state::init_failed;
            return false;
        }

        balance::init(config::get().balance);
        started = xTaskCreatePinnedToCore(task, "control", 6144, nullptr, 4, nullptr, 0) == pdPASS;
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
            latest.mode == static_cast<uint8_t>(action::mode::STOP);
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
