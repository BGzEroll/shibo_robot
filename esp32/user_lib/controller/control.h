#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

namespace control
{
    enum class mode : uint8_t
    {
        BOOT,
        BALANCE,
        SIT,
        JUMP,
        STOP,
        KICK_PLACE,
        KICK_RUN,
        MIDDLE_CALIBRATION
    };

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
        stopped,
        wait_input,
        low_battery
    };

    struct feedback
    {
        float pitch_rad = 0.0f;
        float pitch_rate = 0.0f;

        float speed_m_s = 0.0f;

        float yaw_angle = 0.0f;
        float yaw_rate = 0.0f;

        float roll_angle = 0.0f;
        float avg_leg_height = 0.048f;
    };

    struct status
    {
        arm_state state = arm_state::preparing;
        control::mode mode = control::mode::STOP;
        uint8_t phase = 0;

        feedback measured;

        float left_torque_Nm = 0.0f;
        float right_torque_Nm = 0.0f;

        uint32_t upright_ms = 0;

        bool enabled = false;
        bool calibration_success = false;
    };

    bool init(bool hardware_ready = true);
    status get_status();
    bool begin_configuration();
    void end_configuration();
}

#endif
