#include "balance.h"

#include <math.h>
#include <stdint.h>

namespace balance
{
    namespace
    {
        config settings;
        float gain[4] = {};
        float linear_integral_m = 0.0f;
    }

    /**
     * @brief 按固定模型高度初始化原地平衡增益
     *
     * @param[in] next_settings 平衡配置
     */
    void init(const config &next_settings)
    {
        settings = next_settings;
        const float height = fmaxf(0.02f,
            fminf(0.06f, settings.model_height_m));
        for(uint32_t i = 0; i < 4; i++)
        {
            const float *poly = settings.gain_poly[i];
            gain[i] = ((poly[0] * height + poly[1]) * height + poly[2]) *
                height + poly[3];
        }
        reset();
    }

    /**
     * @brief 清空线速度误差积分
     */
    void reset()
    {
        linear_integral_m = 0.0f;
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
        linear_integral_m -= linear_speed_m_s * dt_s;
        linear_integral_m = fmaxf(-settings.linear_integral_limit_m,
            fminf(settings.linear_integral_limit_m, linear_integral_m));

        return (gain[0] * pitch_rad +
                gain[1] * pitch_rate_rad_s +
                gain[2] * linear_speed_m_s +
                gain[3] * linear_integral_m) * settings.torque_scale;
    }
}
