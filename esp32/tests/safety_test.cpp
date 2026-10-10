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
#include <cmath>
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
    int32_t left_foc_uNm = 0;
    int32_t right_foc_uNm = 0;
    uint32_t pose_updates = 0;
    uint32_t pose_before_trip = 0;
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
     * @brief 提供传感器快照并在指定时段注入超时、倾倒和异常角速度
     *
     * @param[out] out 传感器快照
     *
     * @return true 快照已生成
     */
    bool get_package(package &out)
    {
        out = {};
        out.imu.timestamp_us = clock_us;
        if(tick_ms >= 3500 && tick_ms < 3510){out.imu.angle[1] = 0.6f;}
        if(tick_ms >= 740 && tick_ms < 760){out.imu.timestamp_us -= 15001;}
        if(tick_ms == 4690){out.imu.gyro[1] = NAN;}
        out.left_encoder.timestamp_us = clock_us;
        out.right_encoder.timestamp_us = clock_us;
        if(tick_ms == 4000)
        {
            out.imu.gyro[1] = 0.1f;
            out.left_encoder.speed_mrad_s = settings.left_wheel_direction * 1000;
            out.right_encoder.speed_mrad_s = -settings.right_wheel_direction * 2000;
        }
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
     * @param[in] left_uNm 左轮目标力矩，单位 μN·m
     * @param[in] right_uNm 右轮目标力矩，单位 μN·m
     * @param[in] active 是否使能输出
     */
    void set_target(int32_t left_uNm, int32_t right_uNm, bool active)
    {
        assert(std::abs(left_uNm) <= 25000 && std::abs(right_uNm) <= 25000);
        enabled = active;
        left_foc_uNm = left_uNm;
        right_foc_uNm = right_uNm;
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
        pose_updates++;
    }

    /**
     * @brief 替代测试中的腿部 PID 状态复位
     */
    void reset()
    {
    }

    /**
     * @brief 记录动作是否提交了腿部更新
     */
    void update(float, int8_t, int8_t, bool, uint32_t, float)
    {
        pose_updates++;
    }

    /**
     * @brief 替代测试中无需执行的舵机模式切换
     */
    void set_mode(uint8_t)
    {
    }

    /**
     * @brief 替代测试中无需执行的舵机中位校准
     */
    void calibrate_middle()
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
        input.valid = tick_ms < 2600 || tick_ms >= 2920;
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
 * @param[in] priority 任务优先级
 * @param[in] core 任务核心
 *
 * @return pdPASS
 */
BaseType_t xTaskCreatePinnedToCore(
    void (*task)(void *), const char *, uint32_t, void *, int32_t priority, TaskHandle_t *, int32_t core)
{
    assert(priority == 4 && core == 0);
    control_task = task;
    return pdPASS;
}

/**
 * @brief 推进模拟时钟，注入按键和失联，并检查控制状态
 */
void vTaskDelayUntil(TickType_t *, TickType_t)
{
    const control::status current = control::get_status();
    assert(left_foc_uNm == settings.left_wheel_direction *
        static_cast<int32_t>(roundf(current.left_torque_Nm * 1.0e6f)));
    assert(right_foc_uNm == settings.right_wheel_direction *
        static_cast<int32_t>(roundf(current.right_torque_Nm * 1.0e6f)));

    tick_ms++;
    clock_us += tick_ms <= 100 ? 2000 : 1000;
    if(tick_ms == 20){assert(control::get_status().upright_ms >= 30);}
    if(tick_ms == 80){assert(control::get_status().upright_ms == 100);}
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
    if(tick_ms == 2900){assert(!enabled);}
    if(tick_ms == 2930)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 2940){input.buttons = 0;}
    if(tick_ms == 3500)
    {
        assert(enabled);
        pose_before_trip = pose_updates;
        input.press_count[3]++;
        input.buttons = control::buttons::Y;
    }
    if(tick_ms == 3501)
    {
        assert(!enabled && pose_updates == pose_before_trip);
        const control::status state = control::get_status();
        assert(state.state == control::arm_state::tripped_pitch);
        assert(state.mode == control::mode::STOP && state.phase == 0);
    }
    if(tick_ms == 3510){input.buttons = 0;}
    if(tick_ms == 3540)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 3550){input.buttons = 0;}
    if(tick_ms == 3560){assert(!enabled && control::get_status().mode == control::mode::STOP);}
    if(tick_ms == 3630)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 3640){input.buttons = 0;}
    if(tick_ms == 4100){assert(enabled);}
    if(tick_ms == 4001)
    {
        assert(fabsf(current.measured.speed_m_s - settings.balance.wheel_radius_m * 1.5f) < 1e-7f);
        assert(current.left_torque_Nm != 0 && current.right_torque_Nm != 0);
    }
    if(tick_ms == 4110)
    {
        clock_us += 7000;
        pose_before_trip = pose_updates;
    }
    if(tick_ms == 4111)
    {
        assert(!enabled && pose_updates == pose_before_trip);
        assert(control::get_status().state == control::arm_state::tripped_sensor);
    }
    if(tick_ms == 4200){assert(!enabled);}
    if(tick_ms == 4240)
    {
        input.press_count[9]++;
        input.buttons = control::buttons::RB;
    }
    if(tick_ms == 4250){input.buttons = 0;}
    if(tick_ms == 4690){assert(enabled);}
    if(tick_ms == 4691)
    {
        const control::status state = control::get_status();
        assert(!enabled && state.state == control::arm_state::tripped_sensor);
        assert(state.mode == control::mode::STOP && state.phase == 0);
        assert(state.left_torque_Nm == 0 && state.right_torque_Nm == 0);
    }
    if(tick_ms == 4800)
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
    const int8_t directions[] = {-1, 1};
    for(int8_t left : directions)
    {
        for(int8_t right : directions)
        {
            settings.left_wheel_direction = left;
            settings.right_wheel_direction = right;
            clock_us = 1000000;
            tick_ms = 0;
            pose_updates = 0;
            pose_before_trip = 0;
            input = {};

            assert(control::init());
            try
            {
                control_task(nullptr);
            }
            catch(const finished &)
            {
            }
            assert(!enabled);
        }
    }
    puts("safety tests passed: wheel directions, elapsed time, stale IMU, rearm, config lock, input loss, pitch, deadline, torque guard");
}
