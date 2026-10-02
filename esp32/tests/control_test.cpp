#include "controller/action.h"
#include "controller/balance.h"
#include "controller/input.h"
#include "config.h"
#include "hw/gamepad.h"
#include "hw/leg.h"
#include "hw/aux_servo.h"
#include "io/host.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
    config::settings settings;
    uint64_t clock_us = 1000000;
    uint8_t leg_mode = 0;
    int left_pose = 0;
    int right_pose = 0;
    int camera = 0;
    int camera_updates = 0;
    int frontier = 0;
    bool gamepad_connected = false;
    control::remote_input gamepad_input;
    control::remote_input host_input;
}
namespace config {const settings &get(){return ::settings;}}
namespace sys_time {uint64_t get_us_tick(){return clock_us;}}
namespace leg
{
    void set_pose(int16_t left, int16_t right, uint16_t, uint8_t)
    {left_pose = left; right_pose = right;}
    void set_torque(uint8_t left, uint8_t){leg_mode = left;}
}
namespace aux_servo
{
    void set_camera(uint16_t value){camera = value; camera_updates++;}
    void set_frontier(uint16_t value){frontier = value;}
}
namespace gamepad
{
    bool connected(){return gamepad_connected;}
    bool get(control::remote_input &out){out = gamepad_input; return out.valid;}
}
namespace host {bool get_input(control::remote_input &out){out = host_input; return out.valid;}}

int main()
{
    using namespace control;
    action::state actions;
    action::leg_runtime legs;
    status sensor;
    control_input input;
    input.fresh = true;
    action::context ctx{input,sensor,legs,.6f,2.f,true,false,2088,2008,false,0,0,0};
    action::init(actions);
    assert(action::step(actions,ctx,10).mode == balance_mode::OFF);
    input.action = action_request::BOOT;
    assert(action::step(actions,ctx,10).mode == balance_mode::OFF);
    assert(leg_mode == 1 && actions.current_mode == action::mode::BOOT);
    input.action = action_request::NONE;
    for(int i = 0; i < 60; i++){clock_us += 10000; action::step(actions,ctx,10);}
    assert(actions.current_mode == action::mode::BALANCE);
    input.action = action_request::STOP;
    assert(action::step(actions,ctx,10).mode == balance_mode::OFF && leg_mode == 0);
    input.action = action_request::NONE;
    for(int i = 0; i < 300; i++){action::step(actions,ctx,10);}
    assert(actions.current_mode == action::mode::STOP);

    // 起身未稳定时等待超时必须停止。
    input.action = action_request::BOOT;
    action::step(actions,ctx,10);
    input.action = action_request::NONE;
    sensor.pitch_rad = .3f;
    for(int i = 0; i < 300; i++){action::step(actions,ctx,10);}
    assert(actions.current_mode == action::mode::STOP && leg_mode == 0);
    sensor.pitch_rad = 0;

    actions.current_mode = action::mode::BALANCE;
    input.action = action_request::JUMP_FORWARD;
    action::step(actions,ctx,10);
    assert(left_pose == 2148 && right_pose == 1948);
    input.action = action_request::NONE;
    for(int i = 0; i < 65; i++){action::step(actions,ctx,10);}
    assert(left_pose == 2518 && right_pose == 1578 && actions.phase == 1);
    for(int i = 0; i < 13; i++){action::step(actions,ctx,10);}
    assert(left_pose == 2148 && actions.phase == 2);
    for(int i = 0; i < 61; i++){action::step(actions,ctx,10);}
    assert(actions.current_mode == action::mode::BALANCE);

    // 低电坐下后不响应起身，校准必须通过显式动作触发。
    actions.current_mode = action::mode::SIT;
    actions.phase = 2;
    ctx.battery_low = true;
    input.action = action_request::EXIT;
    assert(action::step(actions,ctx,10).mode == balance_mode::OFF);
    assert(actions.current_mode == action::mode::SIT);
    ctx.battery_low = false;
    input.action = action_request::MIDDLE_CALIBRATION;
    action::step(actions,ctx,10);
    input.action = action_request::NONE;
    for(int i = 0; i < 200; i++){action::step(actions,ctx,10);}
    assert(actions.phase == 3 && leg_mode == 128);

    actions.current_mode = action::mode::BALANCE;
    input.action = action_request::KICK_RUN;
    action::step(actions,ctx,10);
    input.action = action_request::NONE;
    ctx.vision_valid = true;
    ctx.vision_dx = 100;
    ctx.vision_dy = 40;
    ctx.vision_sequence = 1;
    auto command = action::step(actions,ctx,10);
    assert(command.linear_vel > 0 && command.yaw_rate > 0);
    const int updates = camera_updates;
    action::step(actions,ctx,10);
    assert(camera_updates == updates);
    ctx.vision_valid = false;
    command = action::step(actions,ctx,10);
    assert(command.linear_vel == 0 && command.yaw_rate == 0 && camera == 45 && frontier == 0);

    // 两轮共模俯仰、差模偏航、实际高度调度和力矩限幅。
    balance::init(settings.balance);
    command = {};
    command.mode = balance_mode::BALANCE;
    command.steering = true;
    const auto low = balance::step(.031f,.01f,0,0,0,.001f,command);
    balance::reset();
    const auto high = balance::step(.082f,.01f,0,0,0,.001f,command);
    assert(low.left_Nm == low.right_Nm && low.left_Nm < 0 && low.left_Nm != high.left_Nm);
    balance::reset();
    const auto turn = balance::step(.048f,0,0,0,.1f,.001f,command);
    assert(turn.left_Nm < 0 && turn.right_Nm > 0 && fabsf(turn.left_Nm + turn.right_Nm) < 1e-7f);
    const auto saturated = balance::step(.048f,1,10,10,10,.001f,command);
    assert(fabsf(saturated.left_Nm) <= .025f && fabsf(saturated.right_Nm) <= .025f);
    const auto invalid = balance::step(.048f,NAN,0,0,0,.001f,command);
    assert(invalid.left_Nm == 0 && invalid.right_Nm == 0);

    uint8_t report[16] = {};
    for(int i = 0; i < 4; i++){report[i*2+1] = 128;}
    report[13] = 0x81; // A + RB
    report[14] = 0x08; // Start
    report[12] = 2; // Up + Right
    remote_input decoded;
    assert(gamepad::parse_report(report,sizeof(report),decoded));
    assert(decoded.buttons == (buttons::A|buttons::RB|buttons::START|buttons::UP|buttons::RIGHT));
    assert(decoded.press_count[0] == 1 && decoded.axes[0] == 0);
    assert(gamepad::parse_report(report,sizeof(report),decoded) && decoded.press_count[0] == 1);
    assert(!gamepad::parse_report(report,15,decoded));

    // 新连接按住 RB 不启动；释放后再次按下才产生动作。
    input_router::init();
    gamepad_connected = true;
    gamepad_input = decoded;
    gamepad_input.timestamp_us = static_cast<uint32_t>(clock_us);
    gamepad_input.stream_id = 1;
    input_router::update(action::mode::STOP,.6f,2.f,input);
    assert(input.fresh && input.action == action_request::NONE);
    gamepad_input.press_count[9]++;
    input_router::update(action::mode::STOP,.6f,2.f,input);
    // report 中 Start 也在按住，但未出现新边沿，RB 才是新请求。
    assert(input.action == action_request::BOOT);
    clock_us += 250001;
    input_router::update(action::mode::STOP,.6f,2.f,input);
    assert(!input.fresh && input.action == action_request::NONE);
    puts("control tests passed");
}
