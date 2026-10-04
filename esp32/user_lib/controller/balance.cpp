#include "balance.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

namespace balance
{
    namespace
    {
        config settings;
        float gain[2][6] = {};
        float linear_integral_m = 0.0f;
        float yaw_integral_rad = 0.0f;
        float linear_reference = 0.0f;
        float yaw_reference = 0.0f;

        /**
         * @brief 在模型的有效质心高度范围内计算反馈增益
         *
         * @param[in] height_m 质心到轮轴的距离，单位 m
         */
        void update_gain(float height_m)
        {
            const float height = std::clamp(height_m, settings.height_min_m, settings.height_max_m);
            for(uint32_t side = 0; side < 2; side++)
            {
                for(uint32_t i = 0; i < 6; i++)
                {
                    const float *poly = settings.gain_poly[side][i];
                    gain[side][i] = ((poly[0] * height + poly[1]) * height + poly[2]) *
                        height + poly[3];
                }
            }
        }
    }

    /**
     * @brief 初始化平衡配置、参考值和积分状态
     *
     * @param[in] next_settings 平衡配置
     */
    void init(const config &next_settings)
    {
        settings = next_settings;
        reset();
    }

    /**
     * @brief 清空线速度与偏航角速度参考值及误差积分
     */
    void reset()
    {
        linear_integral_m = 0.0f;
        yaw_integral_rad = 0.0f;
        linear_reference = 0.0f;
        yaw_reference = 0.0f;
    }

    /**
     * @brief 根据速度参考、六状态反馈和输出饱和约束计算物理轮力矩
     *
     * @param[in] height_m 质心到轮轴的距离，单位 m
     * @param[in] pitch_rad 俯仰角，单位 rad
     * @param[in] pitch_rate_rad_s 俯仰角速度，单位 rad/s
     * @param[in] linear_speed_m_s 平均轮线速度，单位 m/s
     * @param[in] yaw_rate_rad_s 偏航角速度，单位 rad/s
     * @param[in] dt_s 控制周期，单位 s
     * @param[in] command 本周期平衡指令
     * @param[in] torque_limit_Nm 单轮力矩上限，单位 N·m
     *
     * @return 限幅后的左右轮目标力矩，单位 N·m
     */
    output step(float height_m, float pitch_rad, float pitch_rate_rad_s,
        float linear_speed_m_s, float yaw_rate_rad_s, float dt_s,
        const control::balance_command &command, float torque_limit_Nm)
    {
        if(command.mode == control::balance_mode::OFF)
        {
            reset();
            return {};
        }

        if(command.reset_reference){reset();}

        if(command.mode == control::balance_mode::DIRECT)
        {
            reset();
            return {std::clamp(command.direct_left, -torque_limit_Nm, torque_limit_Nm),
                std::clamp(command.direct_right, -torque_limit_Nm, torque_limit_Nm)};
        }

        if(!std::isfinite(height_m) || !std::isfinite(pitch_rad) ||
           !std::isfinite(pitch_rate_rad_s) || !std::isfinite(linear_speed_m_s) ||
           !std::isfinite(yaw_rate_rad_s) || !std::isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.02f)
        {
            reset();
            return {};
        }

        update_gain(height_m);
        const float alpha = dt_s / (settings.reference_filter_s + dt_s);
        linear_reference += alpha * (command.linear_vel - linear_reference);
        yaw_reference += alpha * (command.yaw_rate - yaw_reference);

        const bool recovering = command.mode == control::balance_mode::RECOVER;
        const float linear_error = command.linear_feedback && !recovering ?
            linear_speed_m_s - linear_reference : 0.0f;
        const float yaw_error = command.yaw_feedback && !recovering ?
            yaw_rate_rad_s - yaw_reference : 0.0f;

        const float old_linear = linear_integral_m;
        const float old_yaw = yaw_integral_rad;
        linear_integral_m = std::clamp(linear_integral_m - linear_error * dt_s,
            -settings.linear_integral_limit_m, settings.linear_integral_limit_m);
        yaw_integral_rad = command.yaw_feedback ? std::clamp(yaw_integral_rad - yaw_error * dt_s,
            -settings.yaw_integral_limit_rad, settings.yaw_integral_limit_rad) : 0.0f;
        if(recovering)
        {
            linear_integral_m = 0.0f;
            yaw_integral_rad = 0.0f;
        }

        const float feedback[6] =
        {
            pitch_rad - settings.pitch_offset_rad, pitch_rate_rad_s,
            linear_error, yaw_error,
            command.linear_feedback ? linear_integral_m : 0.0f,
            command.yaw_feedback ? yaw_integral_rad : 0.0f
        };

        float torque[2] = {};
        const float scale = settings.torque_scale *
            (recovering ? std::clamp(command.recover_blend, 0.0f, 1.0f) : 1.0f);
        bool saturated = false;
        for(uint32_t side = 0; side < 2; side++)
        {
            for(uint32_t i = 0; i < 6; i++){torque[side] += gain[side][i] * feedback[i];}
            torque[side] *= scale;
            saturated = saturated || fabsf(torque[side]) > torque_limit_Nm;
            torque[side] = std::clamp(torque[side], -torque_limit_Nm, torque_limit_Nm);
        }

        // 饱和时只接受减少饱和程度的积分，避免长期积累恢复冲击。
        if(saturated)
        {
            if(fabsf(linear_integral_m) > fabsf(old_linear)){linear_integral_m = old_linear;}
            if(fabsf(yaw_integral_rad) > fabsf(old_yaw)){yaw_integral_rad = old_yaw;}
        }

        return {torque[0], torque[1]};
    }
}
