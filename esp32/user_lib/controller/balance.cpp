#include "balance.h"

#include <algorithm>
#include <stdint.h>

namespace balance
{
    namespace
    {
        config settings;
        float gain[2][6] = {};
        float linear_integral_m = 0.0f;
        float yaw_integral_rad = 0.0f;

        /**
         * @brief 根据当前模型高度计算左右轮反馈增益
         *
         * @param[in] height_m 模型高度，单位 m
         */
        void update_gain(float height_m)
        {
            const float height = std::clamp(height_m, 0.02f, 0.06f);
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
        }
    }

    /**
     * @brief 初始化原地平衡配置和积分状态
     *
     * @param[in] next_settings 平衡配置
     */
    void init(const config &next_settings)
    {
        settings = next_settings;
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
     * @param[in] height_m 本周期模型高度，单位 m
     * @param[in] pitch_rad 俯仰角，单位 rad
     * @param[in] pitch_rate_rad_s 俯仰角速度，单位 rad/s
     * @param[in] linear_speed_m_s 平均轮线速度，单位 m/s
     * @param[in] yaw_rate_rad_s 偏航角速度，单位 rad/s
     * @param[in] dt_s 控制周期，单位 s
     *
     * @return 左右轮原始目标力矩，单位 N·m
     */
    output step(float height_m, float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float yaw_rate_rad_s, float dt_s,
        const reference &target)
    {
        update_gain(height_m);

        linear_integral_m += (target.linear_m_s - linear_speed_m_s) * dt_s;
        linear_integral_m = std::clamp(linear_integral_m,
            -settings.linear_integral_limit_m, settings.linear_integral_limit_m);

        yaw_integral_rad += (target.yaw_rad_s - yaw_rate_rad_s) * dt_s;
        yaw_integral_rad = std::clamp(yaw_integral_rad,
            -settings.yaw_integral_limit_rad, settings.yaw_integral_limit_rad);

        const float feedback[6] =
        {
            pitch_rad, pitch_rate_rad_s,
            target.linear_feedback ? linear_speed_m_s - target.linear_m_s : 0.0f,
            target.yaw_feedback ? yaw_rate_rad_s - target.yaw_rad_s : 0.0f,
            target.linear_feedback ? linear_integral_m : 0.0f,
            target.yaw_feedback ? yaw_integral_rad : 0.0f
        };

        float torque[2] = {};

        for(uint32_t side = 0; side < 2; side++)
        {
            for(uint32_t i = 0; i < 6; i++)
            {
                torque[side] += gain[side][i] * feedback[i];
            }
        }
        return {torque[0], torque[1]};
    }
}
