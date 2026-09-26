#include "test.h"

#include "controller/control.h"
#include "controller/leg.h"
#include "hw/gamepad.h"
#include "hw/sensor.h"
#include "hw/leg_servo.h"
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
        constexpr const char *ARM_STATES[] =
        {
            "PREPARING", "INIT_FAILED", "WAIT_SENSOR", "WAIT_GAMEPAD",
            "WAIT_BUTTON", "ACTIVE", "TRIP_SENSOR",
            "TRIP_GAMEPAD", "TRIP_PITCH"
        };

        void task(void *)
        {
            sensor::package snapshot{};
            gamepad::state pad{};
            leg_servo::state left_servo{};
            leg_servo::state right_servo{};
            TickType_t last_wake = xTaskGetTickCount();

            while(true)
            {
                sensor::get_package(snapshot);
                const bool pad_ready = gamepad::get_state(pad);
                leg::get_feedback(left_servo, right_servo);
                const control::status control_status = control::get_status();
                ESP_LOGI(
                        TAG,
                        "\033[2J\033[H"
                        "BALANCE MONITOR\n"
                        "CTRL %-12s pitch=%+6.3f rad v=%+6.3f m/s\n"
                        "PAD link=%" PRIu32 " ready=%" PRIu32 " btn=%04" PRIX32 " t=%12" PRIu64 "\n"
                        "IMU t=%12" PRIu64 " gyroY=%+7.3f gyroZ=%+7.3f rad/s\n"
                        "L t=%12" PRIu64 " c=%+10" PRId32 " w=%+8" PRId32 " mrad/s\n"
                        "R t=%12" PRIu64 " c=%+10" PRId32 " w=%+8" PRId32 " mrad/s\n"
                        "LEG ok  rad  rad/s duty     V   C     A mov err\n"
                        "  L %2" PRIu32 " %5.2f %+6.2f %+5.2f %5.2f %3" PRIu32
                        " %+5.2f  %" PRIu32 "  %02" PRIX32 "\n"
                        "  R %2" PRIu32 " %5.2f %+6.2f %+5.2f %5.2f %3" PRIu32
                        " %+5.2f  %" PRIu32 "  %02" PRIX32 "\n"
                        "last_us L=%12" PRIu64 " R=%12" PRIu64 "\033[J",
                        ARM_STATES[static_cast<uint8_t>(control_status.state)],
                        control_status.pitch_rad,
                        control_status.speed_m_s,
                        static_cast<uint32_t>(pad.connected),
                        static_cast<uint32_t>(pad_ready),
                        static_cast<uint32_t>(pad.buttons),
                        pad.timestamp_us,
                        snapshot.imu.timestamp_us,
                        snapshot.imu.gyro[1],
                        snapshot.imu.gyro[2],
                        snapshot.left_encoder.timestamp_us,
                        snapshot.left_encoder.full_count,
                        snapshot.left_encoder.speed_mrad_s,
                        snapshot.right_encoder.timestamp_us,
                        snapshot.right_encoder.full_count,
                        snapshot.right_encoder.speed_mrad_s,
                        static_cast<uint32_t>(left_servo.valid),
                        left_servo.position_rad,
                        left_servo.speed_rad_s,
                        left_servo.drive_duty,
                        left_servo.voltage_v,
                        static_cast<uint32_t>(left_servo.temperature_c),
                        left_servo.current_a,
                        static_cast<uint32_t>(left_servo.moving),
                        static_cast<uint32_t>(left_servo.status_bits),
                        static_cast<uint32_t>(right_servo.valid),
                        right_servo.position_rad,
                        right_servo.speed_rad_s,
                        right_servo.drive_duty,
                        right_servo.voltage_v,
                        static_cast<uint32_t>(right_servo.temperature_c),
                        right_servo.current_a,
                        static_cast<uint32_t>(right_servo.moving),
                        static_cast<uint32_t>(right_servo.status_bits),
                        left_servo.timestamp_us,
                        right_servo.timestamp_us);

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PRINT_PERIOD_MS));
            }
        }
    }

    bool init()
    {
        static bool started = false;

        if(started){return true;}

        if(xTaskCreatePinnedToCore(
            task,
            "test_monitor",
            4096,
            nullptr,
            3,
            nullptr,
            0) != pdPASS)
        {
            return false;
        }

        started = true;
        return true;
    }
}
