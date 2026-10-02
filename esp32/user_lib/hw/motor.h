#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

namespace motor
{
    struct config
    {
        float phase_resistance_ohm = 12.27166f;
        float kt_Nm_A = 0.0796f;
        float ke_V_s_rad = 0.0796f;

        float bus_voltage_V = 7.4f;
        float torque_limit_Nm = 0.025f;
    };

    struct directions
    {
        int8_t left = 0;
        int8_t right = 0;
    };

    bool init(const config &settings = {});
    directions get_directions();
    void set_target(int32_t left_uNm, int32_t right_uNm, bool enabled);
}

#endif
