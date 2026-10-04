#include "controller/control.h"
#include "config.h"
#include "hw/sensor.h"
#include "hw/motor.h"
#include "controller/leg.h"
#include "hw/aux_servo.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "controller/input.h"
#include "freertos/task.h"
#include <cassert>
#include <cstdio>

namespace
{
    struct finished
    {
    };

    config::settings settings;
    uint64_t clock_us = 1000000;
    uint32_t tick_ms = 0;
    void (*control_task)(void *) = nullptr;
    bool enabled = false;
    control::remote_input input;
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

namespace sensor
{
    /**
     * @brief 提供传感器快照并在指定时段注入 IMU 超时
     *
     * @param[out] out 传感器快照
     *
     * @return true 快照已生成
     */
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
    /**
     * @brief 提供测试用的左右电机方向
     *
     * @return 左轮为正向、右轮为反向
     */
    directions get_directions()
    {
        return {1, -1};
    }

    /**
     * @brief 记录控制任务的电机使能状态
     *
     * @param[in] active 是否使能输出
     */
    void set_target(int32_t, int32_t, bool active)
    {
        enabled = active;
    }
}

namespace leg
{
    /**
     * @brief 提供有效的左右腿反馈
     *
     * @return 测试用腿部反馈
     */
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

    /**
     * @brief 替代测试中无需执行的舵机位置输出
     */
    void set_pose(int16_t, int16_t, uint16_t, uint8_t)
    {
    }

    /**
     * @brief 替代测试中无需执行的舵机力矩输出
     */
    void set_torque(uint8_t, uint8_t)
    {
    }
}

namespace aux_servo
{
    /**
     * @brief 替代测试中无需执行的摄像头输出
     */
    void set_camera(uint16_t)
    {
    }

    /**
     * @brief 替代测试中无需执行的前挡板输出
     */
    void set_frontier(uint16_t)
    {
    }
}

namespace battery
{
    /**
     * @brief 提供电压正常的电池状态
     *
     * @return 有效且未低电的电池状态
     */
    state get()
    {
        return {8.0f, true, false};
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
        return true;
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
        input.valid = tick_ms < 2600;
        input.stream_id = 1;
        input.timestamp_us = static_cast<uint32_t>(clock_us);
        out = input;
        return true;
    }
}

namespace host
{
    /**
     * @brief 模拟测试中没有上位机遥控输入
     *
     * @param[out] out 遥控输入快照
     *
     * @return false 无有效输入
     */
    bool get_input(control::remote_input &out)
    {
        out = {};
        return false;
    }

    /**
     * @brief 模拟测试中没有视觉目标
     *
     * @param[out] out 视觉快照
     *
     * @return false 无有效目标
     */
    bool get_vision(vision_measurement &out)
    {
        out = {};
        return false;
    }
}

/**
 * @brief 获取模拟的 RTOS 时钟
 *
 * @return 模拟时钟，单位 ms
 */
TickType_t xTaskGetTickCount()
{
    return tick_ms;
}

/**
 * @brief 记录控制任务入口供测试直接执行
 *
 * @param[in] task 控制任务入口
 *
 * @return pdPASS
 */
BaseType_t xTaskCreatePinnedToCore(
    void (*task)(void *), const char *, uint32_t, void *, int32_t, TaskHandle_t *, int32_t)
{
    control_task = task;
    return pdPASS;
}

/**
 * @brief 推进模拟时钟，注入按键和失联，并检查控制状态
 */
void vTaskDelayUntil(TickType_t *, TickType_t)
{
    tick_ms++;
    clock_us += 1000;
    if(tick_ms == 140)
    {
        assert(!enabled);
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
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
    if(tick_ms == 1500)
    {
        assert(enabled);
        input.press_count[5]++;
        input.buttons = control::buttons::START;
    }
    if(tick_ms == 1510){input.buttons = 0;}
    if(tick_ms == 1520){assert(!enabled && control::begin_configuration());}
    if(tick_ms == 1540)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 1550){input.buttons = 0;}
    if(tick_ms == 1900)
    {
        assert(!enabled);
        control::end_configuration();
    }
    if(tick_ms == 1910)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 1920){input.buttons = 0;}
    if(tick_ms == 2500){assert(enabled);}
    // 只更新控制任务的时钟，模拟手柄长时间无新报告。
    if(tick_ms >= 2600){input.valid = false;}
    if(tick_ms == 2900)
    {
        assert(!enabled);
        throw finished{};
    }
}

/**
 * @brief 验证真实控制任务的故障停机、手动重启和配置锁定
 *
 * @return 0 全部断言通过
 */
int32_t main()
{
    assert(!control::begin_configuration()); // 启动校准期间不能重启保存。
    assert(control::init());
    try
    {
        control_task(nullptr);
    }
    catch(const finished &)
    {
    }
    assert(!enabled);
    puts("safety tests passed: stale IMU, explicit rearm, Start, config lock, input loss");
}
