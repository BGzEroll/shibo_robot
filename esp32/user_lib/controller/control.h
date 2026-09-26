#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

namespace control
{
    enum class arm_state : uint8_t
    {
        preparing,
        init_failed,
        wait_sensor,
        wait_gamepad,
        wait_pitch,
        wait_button,
        active,
        tripped_sensor,
        tripped_gamepad,
        tripped_pitch
    };

    struct status
    {
        arm_state state = arm_state::preparing;
        float pitch_rad = 0.0f;
        float speed_m_s = 0.0f;
    };

    bool init();
    status get_status();
    bool request_leg_calibration();
}

#endif
