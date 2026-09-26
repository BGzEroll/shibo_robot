#ifndef LEG_SERVO_H
#define LEG_SERVO_H

#include <stdint.h>

namespace leg_servo
{
    enum class side : uint8_t
    {
        left,
        right
    };

    struct command
    {
        int16_t position = 0;
        uint16_t speed = 0;
        uint8_t acceleration = 0;
    };

    struct state
    {
        uint64_t timestamp_us = 0;      // 上次有效反馈的时间
        bool valid = false;     // 本次读取是否有效
        float position_rad = 0.0f;      // 0 计数对应 0 rad，中位对应 pi rad
        float speed_rad_s = 0.0f;
        float drive_duty = 0.0f;        // 电机驱动占空比，范围 -1 到 1
        float voltage_v = 0.0f;
        uint8_t temperature_c = 0;
        bool moving = false;
        float current_a = 0.0f;     // STS3032 不提供电流反馈
        uint8_t status_bits = 0;        // 舵机错误位
    };

    bool init();
    bool set_target(const command &left, const command &right);
    bool read_feedback(state &left, state &right);
    bool set_torque(bool left_enabled, bool right_enabled);
    bool calibrate_middle(side target);
}

#endif
