#ifndef BALANCE_H
#define BALANCE_H

namespace balance
{
    struct config
    {
        // 行顺序：俯仰角、俯仰角速度、线速度、线速度误差积分。
        float gain_poly[4][4] =
        {
            {-19.78318794f, 2.96741131f, -3.67412914f, -3.30769108f},
            {45.57101193f, -13.86222792f, -0.81410746f, -0.10765136f},
            {848.88274746f, -221.91446497f, 20.87630740f, -1.71800905f},
            {0.0f, 0.0f, 0.0f, 0.84459977f}
        };
        float wheel_radius_m = 0.0263f;
        float model_height_m = 0.048f;
        float linear_integral_limit_m = 2.28f;
        float torque_scale = 0.1f;
        float max_torque_mNm = 30.0f;
    };

    void init(const config &settings);
    void reset();
    float step(float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float dt_s);
}

#endif
