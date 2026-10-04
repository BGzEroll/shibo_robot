#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>

namespace control
{
    enum class balance_mode : uint8_t
    {
        OFF,
        BALANCE,
        DIRECT,
        RECOVER
    };

    struct balance_command
    {
        balance_mode mode = balance_mode::OFF;

        float linear_vel = 0.0f;
        float yaw_rate = 0.0f;

        float direct_left = 0.0f;
        float direct_right = 0.0f;

        float recover_blend = 1.0f;

        bool yaw_feedback = false;
        bool linear_feedback = true;

        bool reset_reference = false;
    };
}

namespace balance
{
    struct output
    {
        float left_Nm = 0.0f;
        float right_Nm = 0.0f;
    };

    struct config
    {
        // 左右轮各六项：俯仰角、角速度、线速度、偏航角速度、线速度误差积分、偏航角速度误差积分。
        // 每项四个系数按三次、二次、一次、常数排列。
        // wip_gain_export.m 的 1 ms 离散 LQR 候选值，实际电机效果仍需标定。
        float gain_poly[2][6][4] =
        {
            {
                {-24.68244362f, 6.430840969f, -1.335108042f, -0.1830263436f},
                {7.616821766f, -1.993669748f, 0.01620335877f, -0.01299167611f},
                {111.8181152f, -25.30869484f, 1.995317101f, -0.1718264818f},
                {0.0f, 0.0f, 0.0f, -0.008051658049f},
                {-7.148844242f, 1.419345975f, -0.06916835904f, 0.1706775278f},
                {0.0f, 0.0f, 0.0f, 0.04610298946f}
            },
            {
                {-24.68244362f, 6.430840969f, -1.335108042f, -0.1830263436f},
                {7.616821766f, -1.993669748f, 0.01620335877f, -0.01299167611f},
                {111.8181152f, -25.30869484f, 1.995317101f, -0.1718264818f},
                {0.0f, 0.0f, 0.0f, 0.008051658049f},
                {-7.148844242f, 1.419345975f, -0.06916835904f, 0.1706775278f},
                {0.0f, 0.0f, 0.0f, -0.04610298946f}
            }
        };

        float wheel_radius_m = 0.0263f;
        float model_height_m = 0.048f;
        float height_min_m = 0.031f;
        float height_max_m = 0.082f;

        float pitch_offset_rad = 0.0f;

        // 物理力矩增益的调试比例；不用于补偿错误的电机参数。
        float torque_scale = 1.0f;

        float reference_filter_s = 0.08f;

        float linear_integral_limit_m = 0.08f;
        float yaw_integral_limit_rad = 0.30f;
    };

    void init(const config &settings);
    void reset();
    output step(float height_m, float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float yaw_rate_rad_s, float dt_s,
        const control::balance_command &command = {}, float torque_limit_Nm = 0.025f);
}

#endif
