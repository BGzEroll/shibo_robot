#include "battery.h"

#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace battery
{
    namespace
    {
        adc_oneshot_unit_handle_t adc = nullptr;
        adc_cali_handle_t calibration = nullptr;
        rmt_channel_handle_t led_channel = nullptr;
        rmt_encoder_handle_t led_encoder = nullptr;
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        state latest;

        void show_led(bool low, uint32_t tick)
        {
            gpio_set_level(GPIO_NUM_13, low ? (tick % 4 < 2) : (tick % 20 == 0));
            if(led_channel == nullptr){return;}
            const bool red = low && ((tick % 20 < 2) || (tick % 20 >= 4 && tick % 20 < 6));
            rmt_symbol_word_t symbols[49] = {};
            // 两颗 WS2812，GRB，亮度 50。
            for(int i = 0; i < 48; i++)
            {
                const bool bit = red && i % 24 >= 8 && i % 24 < 16 &&
                    (50 & (1 << (15 - i % 24)));
                symbols[i].level0 = 1;
                symbols[i].duration0 = bit ? 8 : 4;
                symbols[i].level1 = 0;
                symbols[i].duration1 = bit ? 4 : 8;
            }
            symbols[48].duration0 = 500;
            symbols[48].duration1 = 500;
            rmt_transmit_config_t tx{};
            if(rmt_transmit(led_channel, led_encoder, symbols, sizeof(symbols), &tx) == ESP_OK)
            {
                rmt_tx_wait_all_done(led_channel, pdMS_TO_TICKS(5));
            }
        }

        void task(void *)
        {
            state current;
            uint32_t low_count = 0;
            uint32_t normal_count = 0;
            uint32_t tick = 0;
            TickType_t wake = xTaskGetTickCount();
            while(true)
            {
                int sum = 0;
                bool valid = true;
                for(int i = 0; i < 16; i++)
                {
                    int raw = 0;
                    if(adc_oneshot_read(adc, ADC_CHANNEL_7, &raw) != ESP_OK){valid = false; break;}
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
                        if(++low_count >= 5){current.valid = true; current.low = true; low_count = 5;}
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
                    else{low_count = 0; normal_count = 0;}
                }
                else{current.valid = false; current.low = true; low_count = 0; normal_count = 0;}
                portENTER_CRITICAL(&lock);
                latest = current;
                portEXIT_CRITICAL(&lock);
                show_led(current.low || !current.valid, tick++);
                vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
            }
        }
    }

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
        gpio_config_t pins{};
        pins.pin_bit_mask = 1ULL << GPIO_NUM_13;
        pins.mode = GPIO_MODE_OUTPUT;
        gpio_config(&pins);
        rmt_tx_channel_config_t led{};
        led.gpio_num = GPIO_NUM_21;
        led.clk_src = RMT_CLK_SRC_DEFAULT;
        led.resolution_hz = 10000000;
        led.mem_block_symbols = 64;
        led.trans_queue_depth = 1;
        rmt_copy_encoder_config_t encoder{};
        if(rmt_new_tx_channel(&led, &led_channel) == ESP_OK &&
           rmt_new_copy_encoder(&encoder, &led_encoder) == ESP_OK)
        {
            rmt_enable(led_channel);
        }
        else{led_channel = nullptr;}
        return xTaskCreatePinnedToCore(task, "battery", 3072, nullptr, 1, nullptr, 0) == pdPASS;
    }

    state get()
    {
        portENTER_CRITICAL(&lock);
        const state snapshot = latest;
        portEXIT_CRITICAL(&lock);
        return snapshot;
    }
}
