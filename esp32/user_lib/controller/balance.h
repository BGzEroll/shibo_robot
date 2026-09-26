#ifndef BALANCE_H
#define BALANCE_H

namespace balance
{
    struct output
    {
        float left_nm = 0.0f;
        float right_nm = 0.0f;
    };

    struct config
    {
        // 左右轮各六项：俯仰角、俯仰角速度、线速度、偏航角速度、线速度误差积分、偏航角速度误差积分。
        float gain_poly[2][6][4] =
        {
            {
                {-19.78318794f, 2.96741131f, -3.67412914f, -3.30769108f},
                {45.57101193f, -13.86222792f, -0.81410746f, -0.10765136f},
                {848.88274746f, -221.91446497f, 20.87630740f, -1.71800905f},
                {0.0f, 0.0f, 0.0f, -0.05583265f},
                {0.0f, 0.0f, 0.0f, 0.84459977f},
                {0.0f, 0.0f, 0.0f, 0.33502155f}
            },
            {
                {-19.78318794f, 2.96741131f, -3.67412914f, -3.30769108f},
                {45.57101193f, -13.86222792f, -0.81410746f, -0.10765136f},
                {848.88274746f, -221.91446497f, 20.87630740f, -1.71800905f},
                {0.0f, 0.0f, 0.0f, 0.05583265f},
                {0.0f, 0.0f, 0.0f, 0.84459977f},
                {0.0f, 0.0f, 0.0f, -0.33502155f}
            }
        };
        float wheel_radius_m = 0.0263f;
        float model_height_m = 0.048f;
        float linear_integral_limit_m = 2.28f;
        float yaw_integral_limit_rad = 0.55f;
        float torque_scale = 0.1f;
        float max_torque_mNm = 30.0f;
    };

    void init(const config &settings);
    void reset();
    output step(float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float yaw_rate_rad_s, float dt_s);
}

#endif
