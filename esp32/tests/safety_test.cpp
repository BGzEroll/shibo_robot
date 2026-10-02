#include "controller/control.h"
#include "config.h"
#include "hw/sensor.h"
#include "hw/motor.h"
#include "hw/leg.h"
#include "hw/aux_servo.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "io/host.h"
#include "freertos/task.h"

#include <cassert>
#include <cstdio>

namespace
{
    struct finished {};
    config::settings settings;
    uint64_t clock_us = 1000000;
    uint32_t tick_ms = 0;
    void (*control_task)(void *) = nullptr;
    bool enabled = false;
    control::remote_input input;
}
namespace config {const settings &get(){return ::settings;}}
namespace sys_time {uint64_t get_us_tick(){return clock_us;}}
namespace sensor
{
    bool get_package(package &out)
    {
        out = {};
        out.imu.timestamp_us = clock_us;
        if(tick_ms >= 740 && tick_ms < 760){out.imu.timestamp_us -= 15001;}
        out.left_encoder.timestamp_us = clock_us;
        out.right_encoder.timestamp_us = clock_us;
        return true;
    }
}
namespace motor
{
    directions get_directions(){return {1,-1};}
    void set_target(int32_t, int32_t, bool active){enabled = active;}
}
namespace leg
{
    package get()
    {
        package out;
        out.left.valid = true;
        out.right.valid = true;
        out.left.timestamp_us = clock_us;
        out.right.timestamp_us = clock_us;
        out.left.position_rad = 2088 * (6.28318530718f / 4096);
        out.right.position_rad = 2008 * (6.28318530718f / 4096);
        return out;
    }
    void set_pose(int16_t, int16_t, uint16_t, uint8_t){}
    void set_torque(uint8_t, uint8_t){}
}
namespace aux_servo {void set_camera(uint16_t){} void set_frontier(uint16_t){}}
namespace battery {state get(){return {8.0f,true,false};}}
namespace gamepad
{
    bool connected(){return true;}
    bool get(control::remote_input &out)
    {
        input.valid = tick_ms < 2600;
        input.stream_id = 1;
        input.timestamp_us = static_cast<uint32_t>(clock_us);
        out = input;
        return true;
    }
}
namespace host
{
    bool get_input(control::remote_input &out){out = {}; return false;}
    bool get_vision(vision_measurement &out){out = {}; return false;}
}
TickType_t xTaskGetTickCount(){return tick_ms;}
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *, uint32_t,
    void *, int, TaskHandle_t *, int){control_task = task; return pdPASS;}
void vTaskDelayUntil(TickType_t *, TickType_t)
{
    tick_ms++;
    clock_us += 1000;
    if(tick_ms == 140){assert(!enabled); input.press_count[9]++; input.buttons = control::buttons::RB;}
    if(tick_ms == 150){input.buttons = 0;}
    if(tick_ms == 700){assert(enabled);}
    if(tick_ms == 741)
    {
        assert(!enabled && control::get_status().state == control::arm_state::tripped_sensor);
    }
    if(tick_ms == 940)
    {
        assert(!enabled); // 反馈恢复后不自动启动。
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 950){input.buttons = 0;}
    if(tick_ms == 1500){assert(enabled); input.press_count[5]++; input.buttons = control::buttons::START;}
    if(tick_ms == 1510){input.buttons = 0;}
    if(tick_ms == 1520){assert(!enabled && control::begin_configuration());}
    if(tick_ms == 1540){input.press_count[9]++; input.buttons = control::buttons::RB;}
    if(tick_ms == 1550){input.buttons = 0;}
    if(tick_ms == 1900){assert(!enabled); control::end_configuration();}
    if(tick_ms == 1910){input.press_count[9]++; input.buttons = control::buttons::RB;}
    if(tick_ms == 1920){input.buttons = 0;}
    if(tick_ms == 2500){assert(enabled);}
    // 只更新控制任务的时钟，模拟手柄长时间无新报告。
    if(tick_ms >= 2600){input.valid = false;}
    if(tick_ms == 2900){assert(!enabled); throw finished{};}
}
int main()
{
    assert(!control::begin_configuration()); // 启动校准期间不能重启保存。
    assert(control::init());
    try{control_task(nullptr);}catch(const finished &){}
    assert(!enabled);
    puts("safety tests passed: stale IMU, explicit rearm, Start, config lock, input loss");
}
