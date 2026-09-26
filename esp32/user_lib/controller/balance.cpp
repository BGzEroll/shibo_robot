#include "balance.h"

#include <math.h>
#include <stdint.h>

namespace balance
{
    namespace
    {
        config settings;
        float gain[2][6] = {};
        float linear_integral_m = 0.0f;
        float yaw_integral_rad = 0.0f;
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
        for(uint32_t side = 0; side < 2; side++)
        {
            for(uint32_t i = 0; i < 6; i++)
            {
                const float *poly = settings.gain_poly[side][i];
                gain[side][i] =
                    ((poly[0] * height + poly[1]) * height + poly[2]) *
                    height + poly[3];
            }
        }
        reset();
    }

    /**
     * @brief 清空线速度与偏航角速度误差积分
     */
    void reset()
    {
        linear_integral_m = 0.0f;
        yaw_integral_rad = 0.0f;
    }

    /**
     * @brief 计算原地平衡与偏航差动力矩
     *
     * @param[in] pitch_rad 俯仰角，单位 rad
     * @param[in] pitch_rate_rad_s 俯仰角速度，单位 rad/s
     * @param[in] linear_speed_m_s 平均轮线速度，单位 m/s
     * @param[in] yaw_rate_rad_s 偏航角速度，单位 rad/s
     * @param[in] dt_s 控制周期，单位 s
     *
     * @return 左右轮目标力矩，单位 N·m
     */
    output step(float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float yaw_rate_rad_s, float dt_s)
    {
        linear_integral_m -= linear_speed_m_s * dt_s;
        linear_integral_m = fmaxf(-settings.linear_integral_limit_m,
            fminf(settings.linear_integral_limit_m, linear_integral_m));

        yaw_integral_rad -= yaw_rate_rad_s * dt_s;
        yaw_integral_rad = fmaxf(-settings.yaw_integral_limit_rad,
            fminf(settings.yaw_integral_limit_rad, yaw_integral_rad));

        const float feedback[6] =
        {
            pitch_rad, pitch_rate_rad_s, linear_speed_m_s,
            yaw_rate_rad_s, linear_integral_m, yaw_integral_rad
        };
        float torque[2] = {};
        for(uint32_t side = 0; side < 2; side++)
        {
            for(uint32_t i = 0; i < 6; i++)
            {
                torque[side] += gain[side][i] * feedback[i];
            }
            torque[side] *= settings.torque_scale;
        }
        return {torque[0], torque[1]};
    }
}
