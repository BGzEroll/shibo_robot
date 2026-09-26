#include "leg.h"

#include "hw/gamepad.h"
#include "freertos/FreeRTOS.h"

#include <algorithm>
#include <math.h>

namespace leg
{
    namespace
    {
        constexpr float RAD_TO_COUNT = 4096.0f / 6.2831853f;
        constexpr float RAD_TO_DEG = 180.0f / 3.1415927f;
        constexpr uint64_t FEEDBACK_TIMEOUT_US = 200000;
        constexpr int16_t LEFT_MIN = 2088;
        constexpr int16_t LEFT_MAX = 2398;
        constexpr int16_t RIGHT_MIN = 2008;
        constexpr int16_t RIGHT_MAX = 1698;

        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        leg_servo::state left_state;
        leg_servo::state right_state;
        float height_base = 20.0f;
        float roll_offset = 0.0f;
        float roll_filtered = 0.0f;
        float roll_integral = 0.0f;
        float gravity_ref = 0.0f;
        float duty_ref = 0.0f;
        contact ground_state = contact::unknown;
        uint64_t state_since_us = 0;
        uint64_t candidate_since_us = 0;
        bool candidate_airborne = false;

        /**
         * @brief 按旧项目三次多项式将舵机位置换算为腿高
         *
         * @param[in] position_rad 舵机位置，单位 rad
         *
         * @return 估计腿高，单位 m
         */
        float count_height(float position_rad)
        {
            const float distance = fabsf(position_rad * RAD_TO_COUNT - 2048.0f);
            return ((4.6289047954e-12f * distance - 9.3936274976e-08f) *
                distance + 1.5357902969e-04f) * distance + 4.2041568108e-02f;
        }
    }

    /**
     * @brief 复位腿高、横滚目标和控制积分
     */
    void reset()
    {
        height_base = 20.0f;
        roll_offset = 0.0f;
        roll_filtered = 0.0f;
        roll_integral = 0.0f;
    }

    /**
     * @brief 主动读取并缓存两只舵机的反馈
     */
    void refresh()
    {
        leg_servo::state left;
        leg_servo::state right;
        if(!leg_servo::read_feedback(left, right)){return;}
        portENTER_CRITICAL(&lock);
        left_state = left;
        right_state = right;
        portEXIT_CRITICAL(&lock);
    }

    /**
     * @brief 向两只舵机发送目标位置
     *
     * @param[in] left 左舵机位置计数
     * @param[in] right 右舵机位置计数
     * @param[in] speed 舵机速度
     * @param[in] acceleration 舵机加速度
     */
    void pose(int16_t left, int16_t right, uint16_t speed, uint8_t acceleration)
    {
        leg_servo::set_target({left, speed, acceleration},
            {right, speed, acceleration});
    }

    /**
     * @brief 控制正常平衡时的腿高和横滚补偿，每 20 ms 调用一次
     *
     * @param[in] roll_rad 机身横滚角，单位 rad
     * @param[in] buttons 当前手柄按键
     */
    void control(float roll_rad, uint16_t buttons)
    {
        if(buttons & gamepad::button::UP){height_base -= 0.5f;}
        if(buttons & gamepad::button::DOWN){height_base += 0.5f;}
        if(buttons & gamepad::button::RIGHT){roll_offset += 0.5f;}
        if(buttons & gamepad::button::LEFT){roll_offset -= 0.5f;}
        height_base = std::clamp(height_base, -10.0f, 52.0f);
        roll_filtered += (roll_rad * RAD_TO_DEG - roll_filtered) * 0.0645f;
        const float error = roll_filtered - roll_offset;
        roll_integral = std::clamp(roll_integral + error * 0.02f,
            -15.0f, 15.0f);
        const float adjust = std::clamp(8.0f * error + 30.0f * roll_integral,
            -450.0f, 450.0f);
        const float base = 8.4f * (30.0f - height_base);
        const int16_t left = static_cast<int16_t>(std::clamp(
            2048.0f + base - adjust,
            static_cast<float>(LEFT_MIN), static_cast<float>(LEFT_MAX)));
        const int16_t right = static_cast<int16_t>(std::clamp(
            2048.0f - base - adjust,
            static_cast<float>(RIGHT_MAX), static_cast<float>(RIGHT_MIN)));
        pose(left, right, 1000, 0);
    }

    /**
     * @brief 设置腿舵机扭矩使能
     *
     * @param[in] enabled 是否使能
     */
    void torque(bool enabled)
    {
        leg_servo::set_torque(enabled, enabled);
    }

    /**
     * @brief 判断两只舵机的反馈是否新鲜
     *
     * @param[in] now_us 当前时间，单位 us
     *
     * @return true 两侧反馈新鲜
     */
    bool ready(uint64_t now_us)
    {
        leg_servo::state left;
        leg_servo::state right;
        get_feedback(left, right);
        return left.timestamp_us && right.timestamp_us &&
            now_us >= left.timestamp_us && now_us >= right.timestamp_us &&
            now_us - left.timestamp_us <= FEEDBACK_TIMEOUT_US &&
            now_us - right.timestamp_us <= FEEDBACK_TIMEOUT_US;
    }

    /**
     * @brief 获取左右舵机位置估计的平均腿高
     *
     * @return 平均腿高，单位 m
     */
    float height_m()
    {
        leg_servo::state left;
        leg_servo::state right;
        get_feedback(left, right);
        return (count_height(left.position_rad) +
            count_height(right.position_rad)) * 0.5f;
    }

    /**
     * @brief 复制最近一次有效舵机反馈
     *
     * @param[out] left 左舵机反馈
     * @param[out] right 右舵机反馈
     */
    void get_feedback(leg_servo::state &left, leg_servo::state &right)
    {
        portENTER_CRITICAL(&lock);
        left = left_state;
        right = right_state;
        portEXIT_CRITICAL(&lock);
    }

    /**
     * @brief 用加速度模长和舵机负载更新接地状态
     *
     * @param[in] acceleration 当前加速度模长
     * @param[in] balanced 当前是否处于正常平衡
     * @param[in] now_us 当前时间，单位 us
     *
     * @return 当前接地判断
     */
    contact update_contact(float acceleration, bool balanced, uint64_t now_us)
    {
        if(!ready(now_us) || acceleration <= 0.0f)
        {
            ground_state = contact::unknown;
            candidate_since_us = 0;
            return ground_state;
        }
        leg_servo::state left;
        leg_servo::state right;
        get_feedback(left, right);
        const float duty = (fabsf(left.drive_duty) +
            fabsf(right.drive_duty)) * 0.5f;
        if(balanced)
        {
            gravity_ref = gravity_ref == 0.0f ? acceleration :
                gravity_ref + 0.001f * (acceleration - gravity_ref);
            duty_ref += 0.001f * (duty - duty_ref);
            ground_state = contact::ground;
            state_since_us = now_us;
            candidate_since_us = 0;
            return ground_state;
        }
        if(gravity_ref == 0.0f){return contact::unknown;}

        const bool airborne = acceleration < gravity_ref * 0.55f &&
            (duty_ref < 0.10f || duty < duty_ref * 0.6f);
        const bool landed = acceleration > gravity_ref * 0.85f ||
            (acceleration > gravity_ref * 0.70f && duty > duty_ref * 0.8f);
        const bool next_airborne = ground_state != contact::airborne;
        const bool candidate = next_airborne ? airborne : landed;
        if(!candidate)
        {
            candidate_since_us = 0;
        }
        else if(!candidate_since_us || candidate_airborne != next_airborne)
        {
            candidate_since_us = now_us;
            candidate_airborne = next_airborne;
        }
        else if(now_us - candidate_since_us >=
            (next_airborne ? 25000 : 20000) &&
            now_us - state_since_us >= 50000)
        {
            ground_state = next_airborne ? contact::airborne : contact::ground;
            state_since_us = now_us;
            candidate_since_us = 0;
        }
        if(ground_state == contact::airborne &&
            now_us - state_since_us > 1000000)
        {
            ground_state = contact::unknown;
        }
        return ground_state;
    }
}
