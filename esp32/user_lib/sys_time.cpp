#include "sys_time.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace sys_time
{
    /**
     * @brief 延迟指定毫秒数
     *
     * @param duration_ms 延迟时长，单位毫秒
     */
    void delay_ms(uint32_t duration_ms)
    {
        if(duration_ms == 0){return;}
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }

    /**
     * @brief 延迟指定微秒数
     *
     * @param duration_us 延迟时长，单位微秒
     */
    void delay_us(uint32_t duration_us)
    {
        if(duration_us == 0){return;}
        esp_rom_delay_us(duration_us);
    }

    /**
     * @brief 获取系统毫秒时基计数
     *
     * @return 系统启动后的毫秒计数
     */
    uint64_t get_ms_tick()
    {
        return static_cast<uint64_t>(esp_timer_get_time()) / 1000;
    }

    /**
     * @brief 获取定时器微秒时基计数
     * 
     * @return 定时器当前微秒计数值
     */
    uint64_t get_us_tick()
    {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

}
