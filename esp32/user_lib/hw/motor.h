#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

namespace motor
{
    struct directions
    {
        int8_t left = 0;
        int8_t right = 0;
    };

    bool init();
    directions get_directions();
    void set_target(int32_t left_mNm, int32_t right_mNm, bool enabled);
}

#endif
