#include "action.h"

#include "config.h"
#include "hw/leg.h"
#include "hw/aux_servo.h"
#include "sys_time.h"
#include <algorithm>
#include <cmath>

namespace control::action
{
    namespace
    {
        constexpr float PI = 3.14159265359f;

        /**
         * @brief 计算最短方向的偏航角误差
         *
         * @param[in] target 目标角度，单位 rad
         * @param[in] current 当前角度，单位 rad
         *
         * @return 归一化到 ±π 范围的角度误差，单位 rad
         */
        float angle_error(float target, float current)
        {
            return remainderf(target - current, 2.0f * PI);
        }

        /**
         * @brief 切换动作模式并重置阶段计时
         *
         * @param[in,out] value 动作状态
         * @param[in] next 目标模式
         */
        void enter(state &value, mode next)
        {
            value.current_mode = next;
            value.phase = 0;
            value.timer_ms = 0;
            value.stable_ms = 0;

            if(next == mode::KICK_PLACE || next == mode::KICK_RUN)
            {
                value.camera_deg = 45.0f;
                value.vision_sequence = 0;
                value.vision_time_ms = 0;
                value.cooldown_ms = 0;
                value.post_kick_ms = 0;
                aux_servo::set_camera(45);
            }
        }

        /**
         * @brief 根据固定腿部几何和横滚 PID 更新舵机目标
         *
         * @param[in,out] ctx 本周期动作上下文
         * @param[in] tick_ms 动作周期，单位 ms
         * @param[in] offset 腿部弯曲位置偏移，单位编码器计数
         */
        void update_leg(context &ctx, uint32_t tick_ms, float offset = 0.0f)
        {
            const config::settings &settings = config::get();
            const float dt = tick_ms * 0.001f;
            leg_runtime &value = ctx.leg;
            if(ctx.input.reset_leg){value = {};}

            value.height_base = std::clamp(value.height_base + ctx.input.leg_height_direction * 25.0f * dt,
                -10.0f, 52.0f);
            value.roll_target_deg = std::clamp(value.roll_target_deg + ctx.input.roll_direction * 25.0f * dt,
                -15.0f, 15.0f);

            value.roll_filtered_deg += dt / (0.3f + dt) *
                (ctx.status.roll_angle * 180.0f / PI - value.roll_filtered_deg);

            const float error = value.roll_filtered_deg - value.roll_target_deg;
            value.roll_integral = std::clamp(value.roll_integral + settings.roll_i * error * dt,
                -settings.roll_limit_count, settings.roll_limit_count);
            const float correction = std::clamp(settings.roll_p * error + value.roll_integral +
                settings.roll_d * (error - value.roll_error) / dt,
                -settings.roll_limit_count, settings.roll_limit_count);
            value.roll_error = error;

            const float bend = 8.4f * (30.0f - value.height_base) + offset;
            leg::set_pose(static_cast<int16_t>(std::clamp(2048.0f + bend - correction, 2088.0f, 2398.0f)),
                static_cast<int16_t>(std::clamp(2048.0f - bend - correction, 1698.0f, 2008.0f)), 1000, 0);
        }

        /**
         * @brief 逐步恢复平衡，稳定后进入行驶模式，超时后停止
         *
         * @param[in,out] value 动作状态
         * @param[in,out] ctx 本周期动作上下文
         * @param[in] tick_ms 动作周期，单位 ms
         *
         * @return 本周期平衡指令
         */
        balance_command recover(state &value, context &ctx, uint32_t tick_ms)
        {
            balance_command command;
            command.mode = balance_mode::RECOVER;
            command.recover_blend = std::min(value.timer_ms / 220.0f, 1.0f);

            update_leg(ctx, tick_ms);

            const bool stable =
                fabsf(ctx.status.pitch_rad - config::get().balance.pitch_offset_rad) < 0.16f &&
                fabsf(ctx.status.pitch_rate) < 1.2f;
            value.stable_ms = stable ? value.stable_ms + tick_ms : 0;
            if(value.stable_ms >= 140)
            {
                enter(value, mode::BALANCE);
                command.reset_reference = true;
            }
            else if(value.timer_ms >= 2500)
            {
                enter(value, mode::STOP);
                leg::set_torque(0, 0);
                return {};
            }

            return command;
        }

        /**
         * @brief 执行跳跃的蓄力、伸展、落地和恢复阶段
         *
         * @param[in,out] value 动作状态
         * @param[in] ctx 本周期动作上下文
         *
         * @return 本周期平衡指令
         */
        balance_command jump(state &value, context &ctx)
        {
            balance_command command;
            command.mode = balance_mode::BALANCE;
            command.steering = value.jump_turn != 0 || value.jump_linear != 0;
            command.yaw_feedback = command.steering;
            command.yaw_integral = command.steering;

            const uint32_t push_ms = value.jump_linear > 0 ? 650 : value.jump_linear < 0 ? 700 : 200;
            if(value.phase == 0)
            {
                if(value.jump_linear != 0)
                {
                    const float ramp_ms = value.jump_linear > 0 ? 160.0f : 240.0f;
                    command.linear_vel = value.jump_linear * std::min(ctx.max_linear_vel,
                        value.jump_linear > 0 ? 0.40f : 0.34f) * std::min(value.timer_ms / ramp_ms, 1.0f);
                }

                if(value.timer_ms >= push_ms)
                {
                    leg::set_pose(2518, 1578, 0, 0);
                    value.phase = 1;
                    value.timer_ms = 0;
                }
            }
            else if(value.phase == 1 && value.timer_ms >= 130)
            {
                leg::set_pose(2148, 1948, 0, 0);
                value.phase = 2;
                value.timer_ms = 0;
            }
            else if(value.phase == 2 && value.timer_ms >= 260)
            {
                value.phase = 3;
                value.timer_ms = 0;
            }
            else if(value.phase == 3 && value.timer_ms >= 350)
            {
                enter(value, mode::BALANCE);
                command.reset_reference = true;
            }

            command.linear_feedback = value.jump_linear != 0 && value.phase == 0;
            if(command.steering)
            {
                const float error = angle_error(value.target_yaw, ctx.status.yaw_angle);
                const float feed = value.jump_turn == 0 ? 0.0f :
                    value.phase == 0 ? 1.2f : value.phase == 1 ? 6.4f : 0.0f;
                const float kp = value.jump_turn == 0 ? 3.0f :
                    value.phase == 0 ? 1.4f : value.phase == 1 ? 2.0f : value.phase == 2 ? 0.35f : 0.8f;
                const float limit = value.jump_turn == 0 ? 1.8f :
                    value.phase == 0 ? 1.8f : value.phase == 1 ? 6.4f : value.phase == 2 ? 0.4f : 0.5f;
                command.yaw_rate = std::clamp(value.jump_turn * feed + kp * error, -limit, limit);
            }

            return command;
        }

        /**
         * @brief 执行视觉瞄准、追球、踢球及踢后退回，仅对新视觉帧调整摄像头
         *
         * @param[in,out] value 动作状态
         * @param[in,out] ctx 本周期动作上下文
         * @param[in,out] command 本周期平衡指令
         * @param[in] tick_ms 动作周期，单位 ms
         */
        void kick(state &value, context &ctx, balance_command &command, uint32_t tick_ms)
        {
            value.cooldown_ms = value.cooldown_ms > tick_ms ? value.cooldown_ms - tick_ms : 0;
            if(ctx.input.action == action_request::KICK_EXIT)
            {
                value.phase = 1;
                value.timer_ms = 0;
                aux_servo::set_frontier(0);
            }

            if(value.phase == 1)
            {
                if(value.timer_ms >= 500)
                {
                    ctx.leg = {};
                    enter(value, mode::BALANCE);
                }
                return;
            }

            if(value.post_kick_ms != 0)
            {
                value.post_kick_ms += tick_ms;
                if(value.post_kick_ms < 700)
                {
                    const float error = angle_error(value.target_yaw, ctx.status.yaw_angle);
                    if(fabsf(error) < 10.0f * PI / 180.0f)
                    {
                        if(fabsf(ctx.input.linear) < ctx.max_linear_vel * 0.05f){command.linear_vel = -0.12f;}
                    }
                    else{command.yaw_rate = std::clamp(ctx.input.yaw + 1.2f * error, -0.9f, 0.9f);}
                    return;
                }
                value.post_kick_ms = 0;
            }

            if(!ctx.vision_valid)
            {
                value.camera_deg = 45.0f;
                value.vision_time_ms = 0;
                aux_servo::set_camera(45);
                aux_servo::set_frontier(0);
                return;
            }

            if(value.vision_sequence != ctx.vision_sequence)
            {
                value.vision_sequence = ctx.vision_sequence;
                const uint32_t now = static_cast<uint32_t>(sys_time::get_us_tick() / 1000);
                const uint32_t dt_ms = value.vision_time_ms == 0 ? 10 :
                    std::max<uint32_t>(now - value.vision_time_ms, 1);
                const float derivative = value.vision_time_ms == 0 ? 0.0f :
                    0.5f * (ctx.vision_dy - value.last_dy) / dt_ms;
                if(abs(ctx.vision_dy) > 10)
                {
                    value.camera_deg = std::clamp(value.camera_deg -
                        std::clamp(0.07f * ctx.vision_dy + derivative, -10.0f, 10.0f), 0.0f, 180.0f);
                    aux_servo::set_camera(static_cast<uint16_t>(value.camera_deg));
                }
                value.last_dy = ctx.vision_dy;
                value.vision_time_ms = now;
            }

            const float yaw = abs(ctx.vision_dx) < 40 ? 0.0f :
                std::clamp(ctx.vision_dx * 0.0063f, -0.9f, 0.9f);
            command.yaw_rate = std::clamp(ctx.input.yaw + yaw, -ctx.max_steer_vel, ctx.max_steer_vel);

            const bool running = value.current_mode == mode::KICK_RUN;
            const bool close = value.camera_deg < (running ? 10.0f : 20.0f) &&
                ctx.vision_dy > (running ? -5 : -10) && ctx.vision_dy < 120;
            if(close && value.cooldown_ms == 0)
            {
                aux_servo::set_frontier(0);
                value.cooldown_ms = 2000;
                if(running){value.post_kick_ms = 1;}
            }
            else if(!close){aux_servo::set_frontier(100);}

            if(running && fabsf(ctx.input.linear) < ctx.max_linear_vel * 0.05f && !close)
            {
                command.linear_vel = std::clamp((120 - ctx.vision_dy) * 0.002f,
                    0.0f, std::min(ctx.max_linear_vel, 0.25f));
            }
        }
    }

    /**
     * @brief 初始化动作状态，默认保持停止
     *
     * @param[out] value 动作状态
     */
    void init(state &value)
    {
        value = {};
    }

    /**
     * @brief 根据遥控请求和当前反馈更新小车动作状态
     *
     * @param[in,out] value 动作状态
     * @param[in,out] ctx 本周期动作上下文
     * @param[in] tick_ms 动作周期，单位 ms
     *
     * @return 本周期平衡指令
     */
    balance_command step(state &value, context &ctx, uint32_t tick_ms)
    {
        value.timer_ms += tick_ms;
        if(ctx.input.action == action_request::STOP)
        {
            enter(value, mode::STOP);
            leg::set_torque(0, 0);
            aux_servo::set_frontier(180);
            return {};
        }

        if(value.current_mode == mode::STOP)
        {
            if(ctx.input.action == action_request::BOOT && ctx.battery_valid && !ctx.battery_low)
            {
                enter(value, mode::BOOT);
                leg::set_pose(2088, 2008, 450, 250);
                leg::set_torque(1, 1);
                ctx.leg = {};
            }

            return {};
        }

        if(value.current_mode == mode::BOOT)
        {
            if(value.phase == 0)
            {
                if(value.timer_ms >= 350)
                {
                    value.phase = 1;
                    value.timer_ms = 0;
                }

                return {};
            }

            return recover(value, ctx, tick_ms);
        }

        if(value.current_mode == mode::SIT || value.current_mode == mode::MIDDLE_CALIBRATION)
        {
            if(ctx.input.action == action_request::EXIT && ctx.battery_valid &&
               !ctx.battery_low && value.phase >= 2)
            {
                enter(value, mode::BOOT);
                leg::set_pose(2088, 2008, 450, 250);
                leg::set_torque(1, 1);
                ctx.leg = {};
                return {};
            }

            if(ctx.input.action == action_request::MIDDLE_CALIBRATION && value.current_mode == mode::SIT)
            {
                enter(value, mode::MIDDLE_CALIBRATION);
                value.phase = 2;
                leg::set_torque(0, 0);
            }

            balance_command command;
            if(value.phase == 0)
            {
                command.mode = balance_mode::BALANCE;
                command.steering = true;
                if(abs(ctx.servo_left_position - 2048) <= 50 && abs(ctx.servo_right_position - 2048) <= 50)
                {
                    leg::set_torque(2, 2);
                    value.phase = 1;
                    value.timer_ms = 0;
                }
                else if(value.timer_ms >= 2000)
                {
                    enter(value, mode::STOP);
                    leg::set_torque(0, 0);
                    return {};
                }
            }

            if(value.phase == 1)
            {
                command.mode = balance_mode::DIRECT;
                command.direct_left = -0.005f;
                command.direct_right = -0.005f;
                if(fabsf(ctx.status.pitch_rad) >= 0.25f || value.timer_ms >= 1000)
                {
                    value.phase = 2;
                    value.timer_ms = 0;
                    command = {};
                }
            }

            if(value.phase == 2)
            {
                if(ctx.input.disable_leg_torque || value.timer_ms >= 10000){leg::set_torque(0, 0);}
                if(value.current_mode == mode::MIDDLE_CALIBRATION)
                {
                    leg::set_torque(0, 0);
                    if(value.timer_ms >= 2000)
                    {
                        leg::set_torque(128, 128);
                        value.phase = 3;
                    }
                }
            }

            return command;
        }

        if(value.current_mode == mode::JUMP){return jump(value, ctx);}

        if(ctx.input.action == action_request::SIT)
        {
            enter(value, mode::SIT);
            leg::set_pose(2088, 2008, 450, 250);
        }
        else if(ctx.input.action >= action_request::JUMP_IN_PLACE &&
                ctx.input.action <= action_request::JUMP_RIGHT)
        {
            enter(value, mode::JUMP);
            value.jump_linear = ctx.input.action == action_request::JUMP_FORWARD ? 1 :
                ctx.input.action == action_request::JUMP_BACKWARD ? -1 : 0;
            value.jump_turn = ctx.input.action == action_request::JUMP_LEFT ? 1 :
                ctx.input.action == action_request::JUMP_RIGHT ? -1 : 0;
            value.target_yaw = ctx.status.yaw_angle + value.jump_turn * PI * 0.5f;
            leg::set_pose(2148, 1948, 450, 250);

            balance_command command;
            command.mode = balance_mode::BALANCE;
            command.reset_reference = true;
            return command;
        }
        else if(ctx.input.action == action_request::KICK_PLACE ||
                ctx.input.action == action_request::KICK_RUN)
        {
            enter(value, ctx.input.action == action_request::KICK_PLACE ? mode::KICK_PLACE : mode::KICK_RUN);
            value.target_yaw = ctx.status.yaw_angle;
        }

        balance_command command;
        command.mode = balance_mode::BALANCE;
        command.steering = true;
        command.linear_vel = ctx.input.linear;
        command.yaw_rate = ctx.input.yaw;

        if(ctx.input.action == action_request::RESET_BALANCE){command.reset_reference = true;}

        if(value.current_mode == mode::KICK_PLACE || value.current_mode == mode::KICK_RUN)
        {
            update_leg(ctx, tick_ms, 50.0f);
            kick(value, ctx, command, tick_ms);
        }
        else if(value.current_mode == mode::BALANCE){update_leg(ctx, tick_ms);}

        return command;
    }
}
