#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

struct encoder_package
{
    uint16_t timestamp_us = 0;

    int32_t full_count = 0;
    int32_t speed_mrad_s = 0;
};

struct imu_package
{
    uint16_t timestamp_us = 0;
};

namespace encoder
{
    bool init();
    bool update();
    bool get_package(encoder_package &snapshot);
}

namespace imu
{
    bool init();
    bool update();
    bool get_package(imu_package &imu_package);
}

#endif
