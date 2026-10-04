#include "config.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "controller/leg.h"
#include "hw/aux_servo.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "controller/control.h"
#include "io/host.h"
#include "io/web.h"
#include "esp_log.h"
#include "test.h"

/**
 * @brief 初始化固定硬件和应用任务，硬件失败时保留参数网页
 */
extern "C" void app_init(void)
{
    if(!config::init())
    {
        ESP_LOGE("app", "NVS initialization failed");
        return;
    }

    const bool web_ready = web::init();
    const bool battery_ready = battery::init();
    const bool legs_ready = leg::init();
    const bool aux_ready = aux_servo::init();
    const bool sensors_ready = sensor::init();
    const bool motors_ready = sensors_ready && motor::init(config::get().motor);
    const bool host_ready = host::init();
    const bool gamepad_ready = gamepad::init();
    const bool control_ready = control::init(battery_ready && legs_ready && aux_ready && motors_ready);

    ESP_LOGI("app", "web=%d battery=%d leg=%d aux=%d sensor=%d motor=%d host=%d gamepad=%d control=%d",
        web_ready, battery_ready, legs_ready, aux_ready, sensors_ready, motors_ready,
        host_ready, gamepad_ready, control_ready);

    test::init();
}
