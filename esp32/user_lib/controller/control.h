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
        wait_pitch,
        arming,
        active,
        tripped_sensor,
        tripped_pitch,
        tripped_output
    };

    struct status
    {
        arm_state state = arm_state::preparing;
        float pitch_rad = 0.0f;
        float speed_m_s = 0.0f;
        uint32_t upright_ms = 0;
    };

    bool init();
    status get_status();
}

#endif
