#include "action.h"

#include "config.h"
#include "hw/aux_servo.h"
#include "sys_time.h"
#include <algorithm>
#include <cmath>

namespace control::action
{
    namespace
    {
        constexpr float PI = 3.14159265359f;

        struct runtime
        {
            mode current_mode = mode::STOP;
            uint8_t phase = 0;
            uint32_t timer_ms = 0;
            uint32_t stable_ms = 0;

            int8_t jump_linear = 0;
            int8_t jump_turn = 0;
            float target_yaw = 0.0f;

            float camera_deg = 90.0f;
            int16_t last_dy = 0;
            uint32_t vision_sequence = 0;
            uint32_t vision_time_ms = 0;
            uint32_t cooldown_ms = 0;
            uint32_t post_kick_ms = 0;
        };

        runtime value;

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
         * @brief 清空动作状态并切换模式，保留相机实际目标角度
         *
         * @param[in] next 目标模式
         */
        void enter(mode next)
        {
            const float camera = value.camera_deg;
            value = {};
            value.current_mode = next;
            value.camera_deg = camera;

            if(next == mode::KICK_PLACE || next == mode::KICK_RUN)
            {
                value.camera_deg = 45.0f;
                aux_servo::set_camera(45);
            }
        }

        /**
         * @brief 设置起身姿态并进入启动等待阶段
         */
        void start()
        {
            enter(mode::BOOT);
            leg::set_pose(leg::LEG_LEFT_MIN, leg::LEG_RIGHT_MIN, 450, 250);
            leg::set_torque(1, 1);
            leg::reset();
        }

        /**
         * @brief 执行起身等待与平衡恢复，稳定后进入行驶模式，超时后停止
         *
         * @param[in] input 速度和按键快照
         * @param[in] measured 当前姿态反馈
         * @param[in] tick_ms 动作周期，单位 ms
         *
         * @return 本周期平衡指令
         */
        balance_command boot(const control_input &input, const feedback &measured, uint32_t tick_ms)
        {
            enum : uint8_t
            {
                WAIT_LEG,
                RECOVER
            };
            if(value.phase == WAIT_LEG)
            {
                if(value.timer_ms >= 350)
                {
                    value.phase = RECOVER;
                    value.timer_ms = 0;
                }
                return {};
            }

            balance_command command;
            command.mode = balance_mode::RECOVER;
            command.recover_blend = std::min(value.timer_ms / 220.0f, 1.0f);
            leg::update(measured.roll_angle, input.held, false, tick_ms);

            const bool stable =
                fabsf(measured.pitch_rad - config::get().balance.pitch_offset_rad) < 0.16f &&
                fabsf(measured.pitch_rate) < 1.2f;
            value.stable_ms = stable ? value.stable_ms + tick_ms : 0;
            if(value.stable_ms >= 140)
            {
                enter(mode::BALANCE);
                command.reset_reference = true;
            }
            else if(value.timer_ms >= 2500){return stop().command;}

            return command;
        }

        /**
         * @brief 执行坐下、阻尼、中位校准和重新起身
         *
         * @param[in] input 速度和按键快照
         * @param[in] measured 当前姿态反馈
         * @param[in] legs 当前舵机反馈
         * @param[in] battery_low 电池是否低电
         *
         * @return 本周期平衡指令
         */
        balance_command sit(const control_input &input, const feedback &measured,
            const leg::package &legs, bool battery_low)
        {
            enum : uint8_t
            {
                LOWER,
                DAMP,
                SEATED,
                CALIBRATED
            };
            const bool calibration = (input.held & buttons::SELECT) && (input.pressed & buttons::LB);
            if(!calibration && (input.pressed & buttons::RB) && !battery_low && value.phase >= SEATED)
            {
                start();
                return {};
            }
            if(calibration && value.current_mode == mode::SIT)
            {
                enter(mode::MIDDLE_CALIBRATION);
                value.phase = SEATED;
                leg::set_torque(0, 0);
            }

            balance_command command;
            if(value.phase == LOWER)
            {
                command.mode = balance_mode::BALANCE;
                command.yaw_feedback = true;
                const float count_per_rad = 4096.0f / (2.0f * PI);
                if(abs(static_cast<int16_t>(legs.left.position_rad * count_per_rad) - 2048) <= 50 &&
                   abs(static_cast<int16_t>(legs.right.position_rad * count_per_rad) - 2048) <= 50)
                {
                    leg::set_torque(2, 2);
                    value.phase = DAMP;
                    value.timer_ms = 0;
                }
                else if(value.timer_ms >= 2000){return stop().command;}
            }
            if(value.phase == DAMP)
            {
                command.mode = balance_mode::DIRECT;
                command.direct_left = -0.005f;
                command.direct_right = -0.005f;
                if(fabsf(measured.pitch_rad) >= 0.25f || value.timer_ms >= 1000)
                {
                    value.phase = SEATED;
                    value.timer_ms = 0;
                    command = {};
                }
            }
            if(value.phase == SEATED)
            {
                if((input.held & buttons::LS) || value.timer_ms >= 10000){leg::set_torque(0, 0);}
                if(value.current_mode == mode::MIDDLE_CALIBRATION)
                {
                    leg::set_torque(0, 0);
                    if(value.timer_ms >= 2000)
                    {
                        leg::set_torque(128, 128);
                        value.phase = CALIBRATED;
                    }
                }
            }

            return command;
        }

        /**
         * @brief 执行跳跃的蓄力、伸展、落地和恢复阶段
         *
         * @param[in] measured 当前姿态反馈
         *
         * @return 本周期平衡指令
         */
        balance_command jump(const feedback &measured)
        {
            enum : uint8_t
            {
                PREPARE,
                PUSH,
                LAND,
                SETTLE
            };
            const config::settings &settings = config::get();
            const int8_t jump_linear = value.jump_linear;
            const int8_t jump_turn = value.jump_turn;
            const float target_yaw = value.target_yaw;
            balance_command command;
            command.mode = balance_mode::BALANCE;
            command.yaw_feedback = jump_turn != 0 || jump_linear != 0;

            const uint32_t push_ms = jump_linear > 0 ? 650 : jump_linear < 0 ? 700 : 200;
            if(value.phase == PREPARE)
            {
                if(jump_linear != 0)
                {
                    const float ramp_ms = jump_linear > 0 ? 160.0f : 240.0f;
                    command.linear_vel = jump_linear * std::min(settings.max_linear_m_s,
                        jump_linear > 0 ? 0.40f : 0.34f) * std::min(value.timer_ms / ramp_ms, 1.0f);
                }

                if(value.timer_ms >= push_ms)
                {
                    leg::set_pose(leg::LEG_LEFT_MAX + 20, leg::LEG_RIGHT_MAX - 20, 0, 0);
                    value.phase = PUSH;
                    value.timer_ms = 0;
                }
            }
            else if(value.phase == PUSH && value.timer_ms >= 130)
            {
                leg::set_pose(leg::LEG_LEFT_CROUCH, leg::LEG_RIGHT_CROUCH, 0, 0);
                value.phase = LAND;
                value.timer_ms = 0;
            }
            else if(value.phase == LAND && value.timer_ms >= 260)
            {
                value.phase = SETTLE;
                value.timer_ms = 0;
            }
            else if(value.phase == SETTLE && value.timer_ms >= 350)
            {
                enter(mode::BALANCE);
                command.reset_reference = true;
            }

            command.linear_feedback = jump_linear != 0 && value.phase == PREPARE;
            if(command.yaw_feedback)
            {
                const float error = angle_error(target_yaw, measured.yaw_angle);
                const float feed = jump_turn == 0 ? 0.0f :
                    value.phase == PREPARE ? 1.2f : value.phase == PUSH ? 6.4f : 0.0f;
                const float kp = jump_turn == 0 ? 3.0f :
                    value.phase == PREPARE ? 1.4f : value.phase == PUSH ? 2.0f : value.phase == LAND ? 0.35f : 0.8f;
                const float limit = jump_turn == 0 ? 1.8f :
                    value.phase == PREPARE ? 1.8f : value.phase == PUSH ? 6.4f : value.phase == LAND ? 0.4f : 0.5f;
                command.yaw_rate = std::clamp(jump_turn * feed + kp * error, -limit, limit);
            }

            return command;
        }

        /**
         * @brief 执行视觉瞄准、追球、踢球及踢后退回，仅对新视觉帧调整摄像头
         *
         * @param[in] input 速度和按键快照
         * @param[in] measured 当前姿态反馈
         * @param[in] vision 最新视觉测量
         * @param[in,out] command 本周期平衡指令
         * @param[in] exit_requested 是否请求退出当前踢球模式
         * @param[in] tick_ms 动作周期，单位 ms
         */
        void kick(const control_input &input, const feedback &measured,
            const host::vision_measurement &vision, balance_command &command,
            bool exit_requested, uint32_t tick_ms)
        {
            enum : uint8_t
            {
                TRACK,
                EXIT
            };
            const config::settings &settings = config::get();
            value.cooldown_ms = value.cooldown_ms > tick_ms ? value.cooldown_ms - tick_ms : 0;
            if(exit_requested)
            {
                value.phase = EXIT;
                value.timer_ms = 0;
                aux_servo::set_frontier(0);
            }

            if(value.phase == EXIT)
            {
                if(value.timer_ms >= 500)
                {
                    leg::reset();
                    enter(mode::BALANCE);
                }
                return;
            }

            if(value.post_kick_ms != 0)
            {
                value.post_kick_ms += tick_ms;
                if(value.post_kick_ms < 700)
                {
                    const float error = angle_error(value.target_yaw, measured.yaw_angle);
                    if(fabsf(error) < 10.0f * PI / 180.0f)
                    {
                        if(fabsf(input.linear) < settings.max_linear_m_s * 0.05f){command.linear_vel = -0.12f;}
                    }
                    else{command.yaw_rate = std::clamp(input.yaw + 1.2f * error, -0.9f, 0.9f);}
                    return;
                }
                value.post_kick_ms = 0;
            }

            if(!vision.valid)
            {
                value.camera_deg = 45.0f;
                value.vision_time_ms = 0;
                aux_servo::set_camera(45);
                aux_servo::set_frontier(0);
                return;
            }

            if(value.vision_sequence != vision.sequence)
            {
                value.vision_sequence = vision.sequence;
                const uint32_t now = static_cast<uint32_t>(sys_time::get_us_tick() / 1000);
                const uint32_t dt_ms = value.vision_time_ms == 0 ? 10 :
                    std::max<uint32_t>(now - value.vision_time_ms, 1);
                const float derivative = value.vision_time_ms == 0 ? 0.0f :
                    0.5f * (vision.dy - value.last_dy) / dt_ms;
                if(abs(vision.dy) > 10)
                {
                    value.camera_deg = std::clamp(value.camera_deg -
                        std::clamp(0.07f * vision.dy + derivative, -10.0f, 10.0f), 0.0f, 180.0f);
                    aux_servo::set_camera(static_cast<uint16_t>(value.camera_deg));
                }
                value.last_dy = vision.dy;
                value.vision_time_ms = now;
            }

            const float yaw = abs(vision.dx) < 40 ? 0.0f :
                std::clamp(vision.dx * 0.0063f, -0.9f, 0.9f);
            command.yaw_rate = std::clamp(input.yaw + yaw, -settings.max_yaw_rad_s, settings.max_yaw_rad_s);

            const bool running = value.current_mode == mode::KICK_RUN;
            const bool close = value.camera_deg < (running ? 10.0f : 20.0f) &&
                vision.dy > (running ? -5 : -10) && vision.dy < 120;
            if(close && value.cooldown_ms == 0)
            {
                aux_servo::set_frontier(0);
                value.cooldown_ms = 2000;
                if(running){value.post_kick_ms = 1;}
            }
            else if(!close){aux_servo::set_frontier(100);}

            if(running && fabsf(input.linear) < settings.max_linear_m_s * 0.05f && !close)
            {
                command.linear_vel = std::clamp((120 - vision.dy) * 0.002f,
                    0.0f, std::min(settings.max_linear_m_s, 0.25f));
            }
        }

        /**
         * @brief 处理行驶和踢球模式中的按键及腿部姿态
         *
         * @param[in] input 速度和按键快照
         * @param[in] measured 当前姿态反馈
         * @param[in] vision 最新视觉测量
         * @param[in] battery_low 电池是否低电
         * @param[in] tick_ms 动作周期，单位 ms
         *
         * @return 本周期平衡指令
         */
        balance_command drive(const control_input &input, const feedback &measured,
            const host::vision_measurement &vision, bool battery_low, uint32_t tick_ms)
        {
            const config::settings &settings = config::get();
            const bool modifier = input.held & buttons::SELECT;
            const bool was_kicking = value.current_mode == mode::KICK_PLACE || value.current_mode == mode::KICK_RUN;
            const bool exit_requested = was_kicking && modifier && (input.pressed & buttons::B);
            const bool reset_pose = !was_kicking && (input.pressed & buttons::LS) &&
                fabsf(input.linear) < settings.max_linear_m_s * 0.05f;
            balance_command command;
            command.mode = balance_mode::BALANCE;
            command.yaw_feedback = true;
            command.linear_vel = input.linear;
            command.yaw_rate = input.yaw;

            if(value.current_mode == mode::BALANCE)
            {
                if(!battery_low && !modifier &&
                   (input.pressed & (buttons::B | buttons::X | buttons::A | buttons::Y | buttons::RS)))
                {
                    enter(mode::JUMP);
                    if(input.pressed & buttons::B){value.jump_turn = -1;}
                    else if(input.pressed & buttons::X){value.jump_turn = 1;}
                    else if(input.pressed & buttons::A){value.jump_linear = -1;}
                    else if(input.pressed & buttons::Y){value.jump_linear = 1;}
                    value.target_yaw = measured.yaw_angle + value.jump_turn * PI * 0.5f;
                    leg::set_pose(leg::LEG_LEFT_CROUCH, leg::LEG_RIGHT_CROUCH, 450, 250);
                    command.linear_vel = 0.0f;
                    command.yaw_rate = 0.0f;
                    command.yaw_feedback = false;
                    command.reset_reference = true;
                    return command;
                }
                if(battery_low || (!modifier && (input.pressed & buttons::LB)))
                {
                    enter(mode::SIT);
                    leg::set_pose(leg::LEG_LEFT_MIN, leg::LEG_RIGHT_MIN, 450, 250);
                    return command;
                }
                command.reset_reference = modifier && (input.pressed & buttons::B) &&
                    !(input.pressed & (buttons::X | buttons::Y));
            }
            if(modifier && (input.pressed & (buttons::X | buttons::Y)) &&
               !(was_kicking && (input.pressed & buttons::B)))
            {
                enter(input.pressed & buttons::X ? mode::KICK_PLACE : mode::KICK_RUN);
                value.target_yaw = measured.yaw_angle;
            }

            const bool kicking = value.current_mode == mode::KICK_PLACE || value.current_mode == mode::KICK_RUN;
            leg::update(measured.roll_angle, input.held, reset_pose, tick_ms, kicking ? 50.0f : 0.0f);
            if(kicking){kick(input, measured, vision, command, exit_requested, tick_ms);}
            return command;
        }
    }

    /**
     * @brief 初始化私有动作状态，默认保持停止
     */
    void init()
    {
        value = {};
        stop();
        aux_servo::set_camera(90);
    }

    /**
     * @brief 清空动作阶段和计时，关闭腿部力矩并恢复前挡板
     *
     * @return 停止模式及零轮力矩指令
     */
    output stop()
    {
        enter(mode::STOP);
        leg::reset();
        leg::set_torque(0, 0);
        aux_servo::set_frontier(180);
        return {};
    }

    /**
     * @brief 根据按键和反馈更新动作，统一维护相机目标角度
     *
     * @param[in] input 速度和按键快照
     * @param[in] measured 当前姿态反馈
     * @param[in] legs 当前舵机反馈
     * @param[in] vision 最新视觉测量
     * @param[in] battery_low 电池是否低电
     * @param[in] tick_ms 距上次动作更新的时间，单位 ms
     *
     * @return 本周期平衡指令及动作模式、阶段
     */
    output step(const control_input &input, const feedback &measured,
        const leg::package &legs, const host::vision_measurement &vision,
        bool battery_low, uint32_t tick_ms)
    {
        if(input.pressed & buttons::START){return stop();}

        value.timer_ms += tick_ms;
        balance_command command;
        switch(value.current_mode)
        {
            case mode::STOP:
                if((input.pressed & buttons::RB) && !battery_low){start();}
                break;
            case mode::BOOT:
                command = boot(input, measured, tick_ms);
                break;
            case mode::SIT:
            case mode::MIDDLE_CALIBRATION:
                command = sit(input, measured, legs, battery_low);
                break;
            case mode::JUMP:
                command = jump(measured);
                break;
            case mode::BALANCE:
            case mode::KICK_PLACE:
            case mode::KICK_RUN:
                command = drive(input, measured, vision, battery_low, tick_ms);
                break;
        }

        if(value.current_mode != mode::KICK_PLACE && value.current_mode != mode::KICK_RUN)
        {
            const int8_t direction = input.held & buttons::SELECT ?
                (input.held & buttons::UP ? 1 : 0) - (input.held & buttons::DOWN ? 1 : 0) : 0;
            value.camera_deg = std::clamp(value.camera_deg + direction * 120.0f * tick_ms * 0.001f,
                0.0f, 180.0f);
            aux_servo::set_camera(static_cast<uint16_t>(value.camera_deg));
        }

        return {command, value.current_mode, value.phase};
    }
}
