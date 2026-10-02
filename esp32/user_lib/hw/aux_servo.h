#ifndef AUX_SERVO_H
#define AUX_SERVO_H

#include <stdint.h>

namespace aux_servo
{
    constexpr uint16_t CAMERA_MIN = 0;
    constexpr uint16_t CAMERA_MAX = 180;
    constexpr uint16_t FRONTIER_MIN = 0;
    constexpr uint16_t FRONTIER_MAX = 180;

    bool init();
    void set_camera(uint16_t angle_deg);
    void set_frontier(uint16_t angle_deg);
}

#endif
