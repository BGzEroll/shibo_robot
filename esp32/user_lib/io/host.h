#ifndef HOST_H
#define HOST_H

#include "controller/input.h"
#include <stddef.h>

namespace host
{
    struct vision_measurement
    {
        int16_t dx = 0;
        int16_t dy = 0;

        uint64_t timestamp_us = 0;
        uint32_t sequence = 0;

        bool valid = false;
    };

    bool init();
    bool get_input(control::remote_input &out);
    bool get_vision(vision_measurement &out);
}

#endif
