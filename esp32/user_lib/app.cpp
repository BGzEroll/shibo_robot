#include "hw/sensor.h"
#include "test.h"

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    sensor::init();

    test::init();
}