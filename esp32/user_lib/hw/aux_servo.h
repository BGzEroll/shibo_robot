#ifndef AUX_SERVO_H
#define AUX_SERVO_H

#include <stdint.h>

namespace aux_servo
{
    bool init();
    void set_camera(uint16_t angle_deg);
    void set_frontier(uint16_t angle_deg);
}

#endif
