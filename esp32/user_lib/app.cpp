#include "hw/motor.h"
#include "hw/sensor.h"
#include "hw/leg_servo.h"
#include "hw/gamepad.h"
#include "hw/wifi.h"
#include "io/web.h"
#include "controller/control.h"

#include "test.h"
#include "nvs_flash.h"

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    if(nvs_flash_init() != ESP_OK ||
        !sensor::init() ||
        !motor::init())
    {
        return;
    }

    leg_servo::init();
    gamepad::init();
    wifi::init();
    web::init();
    control::init();

    // test::init();
}
