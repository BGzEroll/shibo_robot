#ifndef LEG_H
#define LEG_H

#include "hw/leg_servo.h"

namespace leg
{
    constexpr int16_t LEG_LEFT_MIN = 2088;
    constexpr int16_t LEG_RIGHT_MIN = 2008;
    constexpr int16_t LEG_LEFT_MAX = 2498;
    constexpr int16_t LEG_RIGHT_MAX = 1598;
    constexpr int16_t LEG_LEFT_CROUCH = 2148;
    constexpr int16_t LEG_RIGHT_CROUCH = 1948;

    struct package
    {
        leg_servo::state left;
        leg_servo::state right;

        float height_m = 0.048f;
        bool io_failed = false;
        bool calibrated = false;
    };

    bool init();
    package get();
    void reset();
    void update(float roll_rad, uint16_t held, bool reset_pose,
        uint32_t tick_ms, float offset = 0.0f);
    void set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t acceleration);
    void set_torque(uint8_t left, uint8_t right);
}

#endif
