#ifndef LEG_H
#define LEG_H

#include "hw/leg_servo.h"

#include <stdint.h>

namespace leg
{
    enum class contact : uint8_t
    {
        unknown,
        ground,
        airborne
    };

    void reset();
    void refresh();
    void control(float roll_rad, uint16_t buttons);
    void pose(int16_t left, int16_t right, uint16_t speed, uint8_t acceleration);
    void torque(bool enabled);
    bool ready(uint64_t now_us);
    float height_m();
    void get_feedback(leg_servo::state &left, leg_servo::state &right);
    contact update_contact(float acceleration, bool balanced, uint64_t now_us);
}

#endif
