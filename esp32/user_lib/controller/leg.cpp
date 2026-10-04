#include "leg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

namespace leg
{
    namespace
    {
        struct command
        {
            leg_servo::command left{2088, 450, 250};
            leg_servo::command right{2008, 450, 250};
            uint8_t left_mode = 0;
            uint8_t right_mode = 0;
            uint32_t pose_sequence = 0;
            uint32_t torque_sequence = 0;
        };

        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        command target;
        package latest;

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
     * @brief 获取缓存的左右腿反馈
     *
     * @return 最新腿部反馈及通信、校准状态
     */
    package get()
    {
        portENTER_CRITICAL(&lock);
        const package snapshot = latest;
        portEXIT_CRITICAL(&lock);
        return snapshot;
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
