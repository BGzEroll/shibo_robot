#include "hw/motor.h"
#include "hw/sensor.h"
#include "hw/servo.h"
#include "test.h"

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    if(sensor::init())
    {
        motor::init();
    }

    servo::init();

    test::init();
}
