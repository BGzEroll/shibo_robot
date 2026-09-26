#include "hw/motor.h"
#include "hw/sensor.h"
#include "hw/leg_servo.h"
#include "controller/control.h"

#include "test.h"

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    if(!sensor::init() ||
        !motor::init() ||
        !leg_servo::init())
    {
        return;
    }

    control::init();

    test::init();
}
