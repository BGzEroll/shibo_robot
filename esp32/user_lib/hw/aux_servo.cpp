#include "aux_servo.h"

#include "driver/ledc.h"
#include <algorithm>

namespace aux_servo
{
    namespace
    {
        bool ready = false;

        /**
         * @brief 以 50 Hz、540 至 2600 us 脉宽驱动固定 PWM 舵机
         *
         * @param[in] channel LEDC 通道
         * @param[in] angle_deg 目标角度，单位 °
         */
        void set(ledc_channel_t channel, uint16_t angle_deg)
        {
            if(!ready){return;}

            angle_deg = std::min<uint16_t>(angle_deg, 180);
            const uint32_t pulse_us = 540 + angle_deg * 2060 / 180;
            const uint32_t duty = pulse_us * 65536 / 20000;

            ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
        }
    }

    /**
     * @brief 初始化摄像头和前挡板的 PWM 舵机
     *
     * @return true 两个舵机通道初始化成功
     * @return false LEDC 定时器或通道配置失败
     */
    bool init()
    {
        ledc_timer_config_t timer{};
        timer.speed_mode = LEDC_LOW_SPEED_MODE;
        timer.duty_resolution = LEDC_TIMER_16_BIT;
        timer.timer_num = LEDC_TIMER_0;
        timer.freq_hz = 50;
        timer.clk_cfg = LEDC_AUTO_CLK;
        if(ledc_timer_config(&timer) != ESP_OK){return false;}

        const int32_t pins[] = {4, 15};
        for(uint32_t i = 0; i < 2; i++)
        {
            ledc_channel_config_t channel{};
            channel.gpio_num = pins[i];
            channel.speed_mode = LEDC_LOW_SPEED_MODE;
            channel.channel = static_cast<ledc_channel_t>(i);
            channel.timer_sel = LEDC_TIMER_0;
            channel.duty = 5145;
            if(ledc_channel_config(&channel) != ESP_OK){return false;}
        }

        ready = true;
        set_camera(90);
        set_frontier(180);
        return true;
    }

    /**
     * @brief 设置摄像头舵机角度
     *
     * @param[in] angle_deg 目标角度，单位 °
     */
    void set_camera(uint16_t angle_deg)
    {
        set(LEDC_CHANNEL_0, angle_deg);
    }

    /**
     * @brief 设置前挡板舵机角度
     *
     * @param[in] angle_deg 目标角度，单位 °
     */
    void set_frontier(uint16_t angle_deg)
    {
        set(LEDC_CHANNEL_1, angle_deg);
    }
}
