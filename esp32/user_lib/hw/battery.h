#ifndef BATTERY_H
#define BATTERY_H

namespace battery
{
    struct state
    {
        float voltage_V = 0.0f;
        bool valid = false;
        bool low = true;
    };

    bool init();
    state get();
}

#endif
