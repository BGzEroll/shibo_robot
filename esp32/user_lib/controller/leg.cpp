#include "leg.h"

#include "config.h"
#include "input.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <cmath>

namespace leg
{
    namespace
    {
        struct command
        {
            leg_servo::command left{LEG_LEFT_MIN, 450, 250};
            leg_servo::command right{LEG_RIGHT_MIN, 450, 250};
            uint8_t left_mode = 0;
            uint8_t right_mode = 0;
            uint32_t pose_sequence = 0;
            uint32_t torque_sequence = 0;
        };

        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        command target;
        package latest;

        float height_base = 20.0f;
        float roll_target_deg = 0.0f;
        float roll_filtered_deg = 0.0f;
        float roll_integral = 0.0f;
        float roll_error = 0.0f;

        /**
         * @brief 作为舵机串口唯一所有者，每 10 ms 写目标、每 20 ms 读取反馈
         */
        void task(void *)
        {
            uint32_t pose_sequence = 0;
            uint32_t torque_sequence = UINT32_MAX;
            uint32_t ticks = 0;
            package state;
            TickType_t wake = xTaskGetTickCount();

            while(true)
            {
                portENTER_CRITICAL(&lock);
                const command current = target;
                portEXIT_CRITICAL(&lock);

                bool written = true;
                if(current.torque_sequence != torque_sequence)
                {
                    if(current.left_mode == 128 || current.right_mode == 128)
                    {
                        written = leg_servo::calibrate_middle(leg_servo::side::left) &&
                            leg_servo::calibrate_middle(leg_servo::side::right);
                        state.calibrated = written;
                    }
                    else
                    {
                        written = leg_servo::set_torque_mode(current.left_mode, current.right_mode);
                    }
                    torque_sequence = current.torque_sequence;
                }

                if(current.pose_sequence != pose_sequence)
                {
                    written = leg_servo::set_target(current.left, current.right) && written;
                    pose_sequence = current.pose_sequence;
                }

                if((ticks++ % 2) == 0)
                {
                    leg_servo::read_feedback(state.left, state.right);
                }

                state.io_failed = !written;

                portENTER_CRITICAL(&lock);
                latest = state;
                portEXIT_CRITICAL(&lock);

                vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
            }
        }
    }

    /**
     * @brief 初始化腿部舵机并启动串口任务
     *
     * @return true 腿部任务已启动
     * @return false 串口初始化或任务创建失败
     */
    bool init()
    {
        return leg_servo::init() && xTaskCreatePinnedToCore(
            task, "leg", 4096, nullptr, 2, nullptr, 0) == pdPASS;
    }

    /**
     * @brief 获取缓存的左右腿反馈并计算模型高度
     *
     * @return 最新腿部反馈、模型高度及通信、校准状态
     */
    package get()
    {
        portENTER_CRITICAL(&lock);
        package snapshot = latest;
        portEXIT_CRITICAL(&lock);

        const config::settings &settings = config::get();
        snapshot.height_m = settings.balance.model_height_m;
        if(settings.height_feedback)
        {
            const float angles[2] = {snapshot.left.position_rad, snapshot.right.position_rad};
            const float *poly = settings.height_poly;
            snapshot.height_m = 0.0f;
            for(float angle : angles)
            {
                const float count = fabsf(angle * (4096.0f / 6.28318530718f) - 2048.0f);
                const float height = ((poly[0] * count + poly[1]) * count + poly[2]) * count + poly[3];
                snapshot.height_m += height * settings.height_com_scale + settings.height_com_offset_m;
            }
            snapshot.height_m *= 0.5f;
        }
        return snapshot;
    }

    /**
     * @brief 清空控制任务持有的腿高和横滚 PID 状态
     */
    void reset()
    {
        height_base = 20.0f;
        roll_target_deg = 0.0f;
        roll_filtered_deg = 0.0f;
        roll_integral = 0.0f;
        roll_error = 0.0f;
    }

    /**
     * @brief 在控制任务中更新腿高、横滚 PID 并提交舵机目标
     *
     * @param[in] roll_rad 横滚角，单位 rad
     * @param[in] held 当前按住的按键
     * @param[in] reset_pose 是否恢复默认腿高和横滚目标
     * @param[in] tick_ms 更新周期，单位 ms
     * @param[in] offset 腿部弯曲位置偏移，单位编码器计数
     */
    void update(float roll_rad, uint16_t held, bool reset_pose, uint32_t tick_ms, float offset)
    {
        const config::settings &settings = config::get();
        const float dt = tick_ms * 0.001f;
        if(reset_pose){reset();}

        const bool modifier = held & control::buttons::SELECT;
        const int8_t height_direction = modifier ? 0 :
            (held & control::buttons::DOWN ? 1 : 0) - (held & control::buttons::UP ? 1 : 0);
        const int8_t roll_direction = modifier ? 0 :
            (held & control::buttons::RIGHT ? 1 : 0) - (held & control::buttons::LEFT ? 1 : 0);
        height_base = std::clamp(height_base + height_direction * 25.0f * dt, -10.0f, 52.0f);
        roll_target_deg = std::clamp(roll_target_deg + roll_direction * 25.0f * dt, -15.0f, 15.0f);
        roll_filtered_deg += dt / (0.3f + dt) * (roll_rad * 180.0f / 3.14159265359f - roll_filtered_deg);

        const float error = roll_filtered_deg - roll_target_deg;
        roll_integral = std::clamp(roll_integral + settings.roll_i * error * dt,
            -settings.roll_limit_count, settings.roll_limit_count);
        const float correction = std::clamp(settings.roll_p * error + roll_integral +
            settings.roll_d * (error - roll_error) / dt, -settings.roll_limit_count, settings.roll_limit_count);
        roll_error = error;

        const float bend = 8.4f * (30.0f - height_base) + offset;
        set_pose(static_cast<int16_t>(std::clamp(2048.0f + bend - correction, 2088.0f, 2398.0f)),
            static_cast<int16_t>(std::clamp(2048.0f - bend - correction, 1698.0f, 2008.0f)), 1000, 0);
    }

    /**
     * @brief 在固定机构范围内提交左右腿目标位置
     *
     * @param[in] left 左腿位置，单位编码器计数
     * @param[in] right 右腿位置，单位编码器计数
     * @param[in] speed STS 舵机速度值
     * @param[in] acceleration STS 舵机加速度值
     */
    void set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t acceleration)
    {
        // 允许坐下中位到跳跃伸展位置，禁止越过固定机构边界。
        left = std::clamp<int16_t>(left, 2048, LEG_LEFT_MAX + 20);
        right = std::clamp<int16_t>(right, LEG_RIGHT_MAX - 20, 2048);

        portENTER_CRITICAL(&lock);
        if(target.left.position != left || target.right.position != right ||
           target.left.speed != speed || target.right.speed != speed ||
           target.left.acceleration != acceleration || target.right.acceleration != acceleration)
        {
            target.left = {left, speed, acceleration};
            target.right = {right, speed, acceleration};
            target.pose_sequence++;
        }
        portEXIT_CRITICAL(&lock);
    }

    /**
     * @brief 提交左右腿力矩模式或中位校准请求
     *
     * @param[in] left 左腿模式：0 关闭、1 位置、2 阻尼、128 校准
     * @param[in] right 右腿模式：0 关闭、1 位置、2 阻尼、128 校准
     */
    void set_torque(uint8_t left, uint8_t right)
    {
        portENTER_CRITICAL(&lock);
        if(target.left_mode != left || target.right_mode != right)
        {
            target.left_mode = left;
            target.right_mode = right;
            target.torque_sequence++;
        }
        portEXIT_CRITICAL(&lock);
    }
}
