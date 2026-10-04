#include "controller/action.h"
#include "controller/balance.h"
#include "controller/input.h"
#include "config.h"
#include "hw/gamepad.h"
#include "controller/leg.h"
#include "hw/aux_servo.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>

namespace
{
    config::settings settings;
    uint64_t clock_us = 1000000;
    uint8_t leg_mode = 0;
    int16_t left_pose = 0;
    int16_t right_pose = 0;
    uint16_t camera = 0;
    uint32_t camera_updates = 0;
    uint16_t frontier = 0;
    bool gamepad_connected = false;
    control::remote_input gamepad_input;
    control::remote_input host_input;
}

namespace config
{
    /**
     * @brief 提供测试配置
     *
     * @return 测试配置的只读引用
     */
    const settings &get()
    {
        return ::settings;
    }
}

namespace sys_time
{
    /**
     * @brief 获取测试时钟
     *
     * @return 测试时钟，单位 us
     */
    uint64_t get_us_tick()
    {
        return clock_us;
    }
}

namespace leg
{
    /**
     * @brief 记录测试中的腿部目标位置
     *
     * @param[in] left 左腿目标位置
     * @param[in] right 右腿目标位置
     */
    void set_pose(int16_t left, int16_t right, uint16_t, uint8_t)
    {
        left_pose = left;
        right_pose = right;
    }

    /**
     * @brief 替代动作测试中的腿部 PID 状态复位
     */
    void reset()
    {
    }

    /**
     * @brief 替代动作测试中的连续腿部姿态更新
     */
    void update(float, uint16_t, bool, uint32_t, float)
    {
    }

    /**
     * @brief 记录测试中的腿部力矩模式
     *
     * @param[in] left 左腿模式
     */
    void set_torque(uint8_t left, uint8_t)
    {
        leg_mode = left;
    }
}

namespace aux_servo
{
    /**
     * @brief 记录摄像头角度及更新次数
     *
     * @param[in] value 目标角度，单位 °
     */
    void set_camera(uint16_t value)
    {
        camera = value;
        camera_updates++;
    }

    /**
     * @brief 记录前挡板角度
     *
     * @param[in] value 目标角度，单位 °
     */
    void set_frontier(uint16_t value)
    {
        frontier = value;
    }
}

namespace gamepad
{
    /**
     * @brief 提供测试用手柄连接状态
     *
     * @return true 已连接；false 未连接
     */
    bool connected()
    {
        return gamepad_connected;
    }

    /**
     * @brief 提供测试用遥控输入
     *
     * @param[out] out 遥控输入快照
     *
     * @return true 输入有效；false 输入无效
     */
    bool get(control::remote_input &out)
    {
        out = gamepad_input;
        return out.valid;
    }
}

namespace host
{
    /**
     * @brief 提供测试用遥控输入
     *
     * @param[out] out 遥控输入快照
     *
     * @return true 输入有效；false 输入无效
     */
    bool get_input(control::remote_input &out)
    {
        out = host_input;
        return out.valid;
    }
}

/**
 * @brief 验证动作阶段、LQR 力矩、手柄报告和输入边沿
 *
 * @return 0 全部断言通过
 */
int32_t main()
{
    using namespace control;
    feedback measured;
    control_input input;
    input.fresh = true;
    leg::package legs;
    legs.left.position_rad = 2088 * (6.28318530718f / 4096);
    legs.right.position_rad = 2008 * (6.28318530718f / 4096);
    host::vision_measurement vision;
    bool battery_low = false;
    action::output actions;
    action::init();

    const auto step = [&]()
    {
        clock_us += 10000;
        actions = action::step(input, measured, legs, vision, battery_low, 10);
        input.pressed = 0;
        return actions.command;
    };
    const auto advance = [&](uint32_t count)
    {
        for(uint32_t i = 0; i < count; i++){step();}
    };
    const auto start = [&]()
    {
        input.pressed = buttons::RB;
        assert(step().mode == balance_mode::OFF);
        assert(actions.current_mode == mode::BOOT && leg_mode == 1);
        advance(60);
        assert(actions.current_mode == mode::BALANCE);
    };

    assert(step().mode == balance_mode::OFF);
    start();
    input.pressed = buttons::START;
    assert(step().mode == balance_mode::OFF && leg_mode == 0);
    assert(actions.current_mode == mode::STOP && actions.phase == 0 && frontier == 180);
    advance(300);
    assert(actions.current_mode == mode::STOP);

    // 起身未稳定时等待超时必须停止。
    input.pressed = buttons::RB;
    step();
    measured.pitch_rad = 0.3f;
    advance(300);
    assert(actions.current_mode == mode::STOP && leg_mode == 0);
    measured.pitch_rad = 0;
    start();

    // 五种跳跃共用阶段流程，完成后回到平衡。
    const uint16_t jump_buttons[] = {buttons::Y, buttons::A, buttons::X, buttons::B, buttons::RS};
    const uint32_t push_ticks[] = {65, 70, 20, 20, 20};
    for(uint32_t i = 0; i < 5; i++)
    {
        input.pressed = jump_buttons[i];
        assert(step().reset_reference && actions.current_mode == mode::JUMP);
        assert(left_pose == 2148 && right_pose == 1948);
        advance(push_ticks[i]);
        assert(left_pose == 2518 && right_pose == 1578 && actions.phase == 1);
        advance(13);
        assert(left_pose == 2148 && actions.phase == 2);
        advance(61);
        assert(actions.current_mode == mode::BALANCE);
    }

    // 跳跃伸展中停止，阶段和计时不能残留到下一次启动。
    input.pressed = buttons::Y;
    step();
    advance(65);
    input.pressed = buttons::START;
    assert(step().mode == balance_mode::OFF);
    assert(actions.current_mode == mode::STOP && actions.phase == 0 && leg_mode == 0);
    start();

    // 同时按下多个动作键时保留原有按键优先级。
    input.pressed = buttons::B | buttons::LB;
    step();
    assert(actions.current_mode == mode::JUMP && step().yaw_rate < 0);
    input.pressed = buttons::START;
    step();
    start();

    input.pressed = buttons::LB;
    step();
    assert(actions.current_mode == mode::SIT);
    assert(step().mode == balance_mode::DIRECT && leg_mode == 2);
    measured.pitch_rad = 0.3f;
    assert(step().mode == balance_mode::OFF && actions.phase == 2);
    measured.pitch_rad = 0;

    // 低电坐下后不响应起身，校准必须通过显式按键触发。
    battery_low = true;
    input.pressed = buttons::RB;
    assert(step().mode == balance_mode::OFF && actions.current_mode == mode::SIT);
    battery_low = false;
    input.held = buttons::SELECT;
    input.pressed = buttons::LB | buttons::RB;
    step();
    assert(actions.current_mode == mode::MIDDLE_CALIBRATION);
    input.held = 0;
    advance(200);
    assert(actions.phase == 3 && leg_mode == 128);
    start();

    for(uint16_t button : {buttons::X, buttons::Y})
    {
        input.held = buttons::SELECT;
        input.pressed = button | buttons::B;
        assert(!step().reset_reference);
        input.held = 0;
        assert(actions.current_mode == (button == buttons::X ? mode::KICK_PLACE : mode::KICK_RUN));
        vision.valid = true;
        vision.dx = 100;
        vision.dy = 40;
        vision.sequence++;
        auto command = step();
        assert(command.yaw_rate > 0);
        if(button == buttons::Y){assert(command.linear_vel > 0);}
        const uint32_t updates = camera_updates;
        step();
        assert(camera_updates == updates);
        vision.valid = false;
        command = step();
        assert(command.linear_vel == 0 && command.yaw_rate == 0 && camera == 45 && frontier == 0);

        // 退出视觉后保留最后角度，手动调节从该角度继续。
        vision.valid = true;
        vision.sequence++;
        step();
        const uint16_t tracked = camera;
        input.held = buttons::SELECT;
        input.pressed = buttons::B | buttons::Y;
        step();
        assert(actions.current_mode == (button == buttons::X ? mode::KICK_PLACE : mode::KICK_RUN));
        advance(50);
        assert(actions.current_mode == mode::BALANCE && camera == tracked);
        input.held |= buttons::UP;
        step();
        assert(camera > tracked && camera <= tracked + 2);
        input.held = 0;
    }

    // 低电请求优先坐下，Start 始终执行停止清理。
    battery_low = true;
    input.pressed = buttons::Y;
    step();
    assert(actions.current_mode == mode::SIT);
    input.pressed = buttons::START;
    assert(step().mode == balance_mode::OFF && actions.phase == 0 && frontier == 180);
    battery_low = false;

    // 两轮共模俯仰、差模偏航、实际高度调度和力矩限幅。
    balance::init(settings.balance);
    balance_command command;
    command.mode = balance_mode::BALANCE;
    command.yaw_feedback = true;
    const auto low = balance::step(0.031f, 0.01f, 0, 0, 0, 0.001f, command);
    balance::reset();
    const auto high = balance::step(0.082f, 0.01f, 0, 0, 0, 0.001f, command);
    assert(low.left_Nm == low.right_Nm && low.left_Nm < 0 && low.left_Nm != high.left_Nm);
    balance::reset();
    const auto turn = balance::step(0.048f, 0, 0, 0, 0.1f, 0.001f, command);
    assert(turn.left_Nm < 0 && turn.right_Nm > 0 && fabsf(turn.left_Nm + turn.right_Nm) < 1e-7f);
    const auto saturated = balance::step(0.048f, 1, 10, 10, 10, 0.001f, command);
    assert(fabsf(saturated.left_Nm) <= 0.025f && fabsf(saturated.right_Nm) <= 0.025f);

    uint8_t report[16] = {};
    for(uint32_t i = 0; i < 4; i++){report[i * 2 + 1] = 128;}
    report[13] = 0x81; // A + RB
    report[14] = 0x08; // Start
    report[12] = 2;    // Up + Right
    remote_input decoded;
    assert(gamepad::parse_report(report, sizeof(report), decoded));
    assert(decoded.buttons == (buttons::A | buttons::RB | buttons::START | buttons::UP | buttons::RIGHT));
    assert(decoded.press_count[0] == 1 && decoded.axes[0] == 0);
    assert(gamepad::parse_report(report, sizeof(report), decoded) && decoded.press_count[0] == 1);
    assert(!gamepad::parse_report(report, 15, decoded));

    // 新连接按住 RB 不产生边沿；来源切换和重连也不能重放历史按键。
    input_router::init();
    gamepad_connected = true;
    gamepad_input = decoded;
    gamepad_input.timestamp_us = static_cast<uint32_t>(clock_us);
    gamepad_input.stream_id = 1;
    input_router::update(input);
    assert(input.fresh && input.pressed == 0);
    gamepad_input.press_count[9]++;
    input_router::update(input);
    assert(input.pressed == buttons::RB);
    gamepad_input.stream_id++;
    gamepad_input.press_count[5]++;
    input_router::update(input);
    assert(input.pressed == 0);
    gamepad_connected = false;
    host_input = gamepad_input;
    input_router::update(input);
    assert(input.source == input_source::HOST && input.pressed == 0);
    host_input.press_count[5]++;
    input_router::update(input);
    assert(input.pressed == buttons::START);
    clock_us += 250001;
    input_router::update(input);
    assert(!input.fresh && input.pressed == 0);
    puts("control tests passed: actions, camera handoff, interruption, LQR, input edges");
}
