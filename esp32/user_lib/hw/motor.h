#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

namespace motor
{
    bool init();
    void set_target(int32_t left_mNm, int32_t right_mNm, bool enabled);
}

#endif
