#include "controller/leg.h"
#include "controller/input.h"
#include "config.h"
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
    void (*leg_task)(void *) = nullptr;
    leg_servo::command left_target;
    leg_servo::command right_target;
    uint8_t torque_mode = 0;
    uint32_t calibrations = 0;
    uint32_t writes = 0;
    uint32_t tick_ms = 0;
}

namespace config
{
    /**
     * @brief 提供可修改的测试配置
     *
     * @return 测试配置的只读引用
     */
    const settings &get()
    {
        return ::settings;
    }
}

namespace leg_servo
{
    /**
     * @brief 模拟舵机串口初始化成功
     *
     * @return true
     */
    bool init()
    {
        return true;
    }

    /**
     * @brief 记录实际腿部任务提交的左右目标
     *
     * @param[in] left 左腿目标
     * @param[in] right 右腿目标
     *
     * @return true
     */
    bool set_target(const command &left, const command &right)
    {
        left_target = left;
        right_target = right;
        writes++;
        return true;
    }

    /**
     * @brief 提供固定起身姿态的舵机反馈
     *
     * @param[out] left 左腿反馈
     * @param[out] right 右腿反馈
     *
     * @return true
     */
    bool read_feedback(state &left, state &right)
    {
        left.valid = right.valid = true;
        left.position_rad = 2088 * (6.28318530718f / 4096);
        right.position_rad = 2008 * (6.28318530718f / 4096);
        left.timestamp_us = right.timestamp_us = tick_ms * 1000 + 1;
        return true;
    }

    /**
     * @brief 记录腿部力矩模式
     *
     * @param[in] left 左腿模式
     * @param[in] right 右腿模式
     *
     * @return true
     */
    bool set_torque_mode(uint8_t left, uint8_t right)
    {
        assert(left == right);
        torque_mode = left;
        return true;
    }

    /**
     * @brief 记录中位校准次数
     *
     * @return true
     */
    bool calibrate_middle(side)
    {
        calibrations++;
        return true;
    }
}

/**
 * @brief 获取模拟的 RTOS 时钟
 *
 * @return 模拟时间，单位 ms
 */
TickType_t xTaskGetTickCount()
{
    return tick_ms;
}

/**
 * @brief 保存真实腿部任务入口并检查调度配置
 *
 * @param[in] task 腿部任务入口
 * @param[in] priority 任务优先级
 * @param[in] core 任务核心
 *
 * @return pdPASS
 */
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *, uint32_t,
    void *, int32_t priority, TaskHandle_t *, int32_t core)
{
    assert(priority == 2 && core == 0);
    leg_task = task;
    return pdPASS;
}

/**
 * @brief 在真实腿部任务的周期间提交目标并检查输出
 */
void vTaskDelayUntil(TickType_t *, TickType_t period)
{
    assert(period == 10);
    tick_ms += period;
    switch(tick_ms)
    {
        case 10:
            assert(left_target.position == 2132 && right_target.position == 1964);
            assert(left_target.speed == 1000 && torque_mode == 0);
            leg::update(0, control::buttons::DOWN, false, 10);
            break;
        case 20:
            assert(left_target.position == 2129 && right_target.position == 1966);
            leg::update(0, control::buttons::RIGHT, true, 10);
            break;
        case 30:
            assert(left_target.position == 2134 && right_target.position == 1966);
            leg::set_torque(128, 128);
            break;
        case 40:
            assert(calibrations == 2 && leg::get().calibrated);
            break;
        case 50:
            assert(calibrations == 2 && writes == 3);
            leg::set_pose(3000, 1000, 450, 250);
            break;
        case 60:
            assert(left_target.position == 2518 && right_target.position == 1578);
            assert(left_target.speed == 450 && right_target.acceleration == 250);
            settings.height_feedback = true;
            settings.height_poly[0] = settings.height_poly[1] = 0;
            settings.height_poly[2] = 0.001f;
            settings.height_poly[3] = 0.01f;
            settings.height_com_scale = 2;
            settings.height_com_offset_m = 0.003f;
            assert(fabsf(leg::get().height_m - 0.103f) < 1e-6f);
            settings.height_feedback = false;
            settings.balance.model_height_m = 0.06f;
            assert(leg::get().height_m == 0.06f);
            leg::reset();
            leg::update(0, control::buttons::SELECT | control::buttons::DOWN, false, 10, 50);
            leg::set_torque(0, 0);
            break;
        case 70:
            assert(left_target.position == 2182 && right_target.position == 1914 && torque_mode == 0);
            throw finished{};
    }
}

/**
 * @brief 验证腿部 PID、几何、目标限幅和串口任务的序列去重
 *
 * @return 0 全部断言通过
 */
int32_t main()
{
    assert(leg::init());
    leg::reset();
    leg::update(0, 0, false, 10);
    try
    {
        leg_task(nullptr);
    }
    catch(const finished &)
    {
    }
    puts("leg tests passed: PID, geometry, pose limits, command deduplication, calibration");
}
