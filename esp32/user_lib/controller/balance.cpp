#include "balance.h"

#include <math.h>
#include <stdint.h>

namespace balance
{
    namespace
    {
        config settings;
        float gain[4] = {};
        float filtered_speed_m_s = 0.0f;
        float linear_integral_m = 0.0f;
        bool first_sample = true;
    }

    /**
     * @brief 按固定腿长初始化原地平衡增益
     *
     * @param[in] next_settings 平衡配置
     * @param[in] leg_height_m 当前平均腿长，单位 m
     */
    void init(const config &next_settings, float leg_height_m)
    {
        settings = next_settings;
        const float height = fmaxf(0.02f, fminf(0.06f, leg_height_m));
        for(uint32_t i = 0; i < 4; i++)
        {
            const float *poly = settings.gain_poly[i];
            gain[i] = ((poly[0] * height + poly[1]) * height + poly[2]) *
                height + poly[3];
        }
        reset();
    }

    /**
     * @brief 清空平衡滤波与积分状态
     */
    void reset()
    {
        filtered_speed_m_s = 0.0f;
        linear_integral_m = 0.0f;
        first_sample = true;
    }

    /**
     * @brief 计算双轮相同的原地平衡力矩
     *
     * @param[in] pitch_rad 俯仰角，单位 rad
     * @param[in] pitch_rate_rad_s 俯仰角速度，单位 rad/s
     * @param[in] linear_speed_m_s 平均轮线速度，单位 m/s
     * @param[in] dt_s 控制周期，单位 s
     *
     * @return 单轮目标力矩，单位 N·m
     */
    float step(float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float dt_s)
    {
        if(first_sample)
        {
            filtered_speed_m_s = linear_speed_m_s;
            first_sample = false;
        }
        else
        {
            filtered_speed_m_s += (linear_speed_m_s - filtered_speed_m_s) *
                dt_s / (settings.speed_filter_tau_s + dt_s);
        }

        linear_integral_m -= filtered_speed_m_s * dt_s;
        linear_integral_m = fmaxf(-settings.linear_integral_limit_m,
            fminf(settings.linear_integral_limit_m, linear_integral_m));

        return (gain[0] * pitch_rad +
                gain[1] * pitch_rate_rad_s +
                gain[2] * filtered_speed_m_s +
                gain[3] * linear_integral_m) * settings.torque_scale;
    }
}
