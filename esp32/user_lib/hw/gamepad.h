#ifndef GAMEPAD_H
#define GAMEPAD_H

#include "controller/input.h"
#include <stddef.h>

namespace gamepad
{
    struct device
    {
        char address[18] = {};
        char name[32] = {};

        int8_t rssi = 0;
    };

    bool init();
    bool get(control::remote_input &out);
    bool connected();
    size_t get_devices(device *out, size_t capacity);
    bool parse_report(const uint8_t *data, size_t length, control::remote_input &out);
}

#endif
