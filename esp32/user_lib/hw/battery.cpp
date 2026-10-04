#include "battery.h"

#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

namespace battery
{
    namespace
    {
        adc_oneshot_unit_handle_t adc = nullptr;
        adc_cali_handle_t calibration = nullptr;
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        state latest;

        /**
         * @brief 周期采样母线电压并发布电池状态
         */
        void task(void *)
        {
            state current;
            uint32_t low_count = 0;
            uint32_t normal_count = 0;
            TickType_t wake = xTaskGetTickCount();

            while(true)
            {
                int32_t sum = 0;
                bool valid = true;
                for(uint32_t i = 0; i < 16; i++)
                {
                    int raw = 0;
                    if(adc_oneshot_read(adc, ADC_CHANNEL_7, &raw) != ESP_OK)
                    {
                        valid = false;
                        break;
                    }
                    sum += raw;
                }

                int millivolts = 0;
                valid = valid && adc_cali_raw_to_voltage(calibration, sum / 16, &millivolts) == ESP_OK;
                if(valid)
                {
                    current.voltage_V = millivolts * 0.00397f;
                    if(current.voltage_V < config::get().battery_low_V)
                    {
                        normal_count = 0;
                        if(++low_count >= 5)
                        {
                            current.valid = true;
                            current.low = true;
                            low_count = 5;
                        }
                    }
                    else if(!current.valid || current.voltage_V >= config::get().battery_recover_V)
                    {
                        low_count = 0;
                        if(++normal_count >= (current.valid ? 10U : 5U))
                        {
                            current.valid = true;
                            current.low = false;
                            normal_count = 10;
                        }
                    }
                    else
                    {
                        low_count = 0;
                        normal_count = 0;
                    }
                }
                else
                {
                    current.valid = false;
                    current.low = true;
                    low_count = 0;
                    normal_count = 0;
                }

                portENTER_CRITICAL(&lock);
                latest = current;
                portEXIT_CRITICAL(&lock);

                vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
            }
        }
    }

    /**
     * @brief 初始化电池 ADC、电压校准和采样任务
     *
     * @return true 电池采样任务已启动
     * @return false ADC、校准或任务创建失败
     */
    bool init()
    {
        adc_oneshot_unit_init_cfg_t unit{};
        unit.unit_id = ADC_UNIT_1;
        if(adc_oneshot_new_unit(&unit, &adc) != ESP_OK){return false;}

        adc_oneshot_chan_cfg_t channel{};
        channel.atten = ADC_ATTEN_DB_12;
        channel.bitwidth = ADC_BITWIDTH_12;
        if(adc_oneshot_config_channel(adc, ADC_CHANNEL_7, &channel) != ESP_OK){return false;}

        adc_cali_line_fitting_config_t cal{};
        cal.unit_id = ADC_UNIT_1;
        cal.atten = ADC_ATTEN_DB_12;
        cal.bitwidth = ADC_BITWIDTH_12;
        cal.default_vref = 1100;
        if(adc_cali_create_scheme_line_fitting(&cal, &calibration) != ESP_OK){return false;}

        return xTaskCreatePinnedToCore(task, "battery", 3072, nullptr, 1, nullptr, 0) == pdPASS;
    }

    /**
     * @brief 获取最新电池状态
     *
     * @return 电压、测量有效性和低电状态
     */
    state get()
    {
        portENTER_CRITICAL(&lock);
        const state snapshot = latest;
        portEXIT_CRITICAL(&lock);
        return snapshot;
    }
}
