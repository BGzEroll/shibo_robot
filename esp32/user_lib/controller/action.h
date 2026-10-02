#ifndef ACTION_H
#define ACTION_H

#include "balance.h"
#include "control.h"
#include "input.h"

namespace control::action
{
    enum class mode : uint8_t
    {
        BOOT, BALANCE, SIT, JUMP, STOP, KICK_PLACE, KICK_RUN, MIDDLE_CALIBRATION
    };

    struct leg_runtime
    {
        float height_base = 20.0f;
        float roll_target_deg = 0.0f;
        float roll_filtered_deg = 0.0f;
        float roll_integral = 0.0f;
        float roll_error = 0.0f;
    };

    struct state
    {
        mode current_mode = mode::STOP;
        uint8_t phase = 0;
        uint32_t timer_ms = 0;
        uint32_t stable_ms = 0;
        int8_t jump_linear = 0;
        int8_t jump_turn = 0;
        float target_yaw = 0.0f;
        float camera_deg = 45.0f;
        int16_t last_dy = 0;
        uint32_t vision_sequence = 0;
        uint32_t vision_time_ms = 0;
        uint32_t cooldown_ms = 0;
        uint32_t post_kick_ms = 0;
    };

    struct context
    {
        control_input &input;
        const control::status &status;
        leg_runtime &leg;
        float max_linear_vel;
        float max_steer_vel;
        bool battery_valid;
        bool battery_low;
        int16_t servo_left_position;
        int16_t servo_right_position;
        bool vision_valid;
        int16_t vision_dx;
        int16_t vision_dy;
        uint32_t vision_sequence;
    };

    void init(state &value);
    balance_command step(state &value, context &ctx, uint32_t tick_ms);
}

#endif
