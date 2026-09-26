#include "test.h"

#include "hw/motor.h"
#include "hw/sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>

namespace test
{
    namespace
    {
        constexpr const char *TAG = "test";
        constexpr uint32_t COMMAND_PERIOD_MS = 10;
        constexpr uint32_t PRINT_PERIOD_MS = 100;
        constexpr int32_t LEFT_TORQUE_MNM = static_cast<int32_t>(0.025f * 1000.0f);

        void task(void *)
        {
            sensor::package snapshot{};
            TickType_t last_wake = xTaskGetTickCount();
            uint32_t elapsed_ms = 0;

            while(true)
            {
                // motor::set_target(LEFT_TORQUE_MNM, 0, true);
                // motor::set_target(0, LEFT_TORQUE_MNM, true);
                motor::set_target(0, 0, false);
                elapsed_ms += COMMAND_PERIOD_MS;

                if(elapsed_ms >= PRINT_PERIOD_MS)
                {
                    elapsed_ms = 0;
                    sensor::get_package(snapshot);
                    ESP_LOGI(
                        TAG,
                        "\033[H"
                        "REQ L=%+4" PRId32 " mNm R=OFF\n"
                        "L t=%12" PRIu64 " c=%+10" PRId32 " w=%+8" PRId32 " mrad/s\n"
                        "R t=%12" PRIu64 " c=%+10" PRId32 " w=%+8" PRId32 " mrad/s\033[J",
                        LEFT_TORQUE_MNM,
                        snapshot.left_encoder.timestamp_us,
                        snapshot.left_encoder.full_count,
                        snapshot.left_encoder.speed_mrad_s,
                        snapshot.right_encoder.timestamp_us,
                        snapshot.right_encoder.full_count,
                        snapshot.right_encoder.speed_mrad_s);
                }

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(COMMAND_PERIOD_MS));
            }
        }
    }

    bool init()
    {
        static bool started = false;

        if(started){return true;}

        if(xTaskCreatePinnedToCore(
            task,
            "motor_test",
            4096,
            nullptr,
            4,
            nullptr,
            0) != pdPASS)
        {
            return false;
        }

        started = true;
        return true;
    }
}
