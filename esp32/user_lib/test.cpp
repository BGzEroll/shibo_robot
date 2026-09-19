#include "test.h"

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
        constexpr uint32_t PRINT_PERIOD_MS = 100;

        void task(void *)
        {
            sensor::package snapshot{};

            while(true)
            {
                if(sensor::get_package(snapshot))
                {
                    ESP_LOGI(
                        TAG,
                        "\033[H"
                        "==================== SENSOR ====================\n"
                        "LEFT  ts=%12" PRIu64 " us  count=%+10" PRId32 "  speed=%+8" PRId32 " mrad/s\n"
                        "RIGHT ts=%12" PRIu64 " us  count=%+10" PRId32 "  speed=%+8" PRId32 " mrad/s\n"
                        "IMU   ts=%12" PRIu64 " us  temp=%+7.2f C\n"
                        "ACC   x=%+8.3f  y=%+8.3f  z=%+8.3f\n"
                        "GYRO  x=%+8.3f  y=%+8.3f  z=%+8.3f\n"
                        "ANGLE x=%+8.3f  y=%+8.3f  z=%+8.3f\n"
                        "=================================================\033[J",
                        snapshot.left_encoder.timestamp_us,
                        snapshot.left_encoder.full_count,
                        snapshot.left_encoder.speed_mrad_s,
                        snapshot.right_encoder.timestamp_us,
                        snapshot.right_encoder.full_count,
                        snapshot.right_encoder.speed_mrad_s,
                        snapshot.imu.timestamp_us,
                        snapshot.imu.temperature,
                        snapshot.imu.acc[0],
                        snapshot.imu.acc[1],
                        snapshot.imu.acc[2],
                        snapshot.imu.gyro[0],
                        snapshot.imu.gyro[1],
                        snapshot.imu.gyro[2],
                        snapshot.imu.angle[0],
                        snapshot.imu.angle[1],
                        snapshot.imu.angle[2]);
                }

                vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
            }
        }
    }

    bool init()
    {
        static bool started = false;

        if(started){return true;}

        if(xTaskCreatePinnedToCore(
            task,
            "sensor_print",
            8192,
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
