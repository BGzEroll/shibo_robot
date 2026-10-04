#ifndef ACTION_H
#define ACTION_H

#include "balance.h"
#include "control.h"
#include "input.h"
#include "leg.h"

namespace control::action
{
    struct output
    {
        balance_command command;
        mode current_mode = mode::STOP;
        uint8_t phase = 0;
    };

    void init();
    output stop();
    output step(const control_input &input, const feedback &measured,
        const leg::package &legs, const host::vision_measurement &vision,
        bool battery_low, uint32_t tick_ms);
}

#endif
