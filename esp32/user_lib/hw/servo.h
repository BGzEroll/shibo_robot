#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

namespace servo
{
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
        int16_t position = 0;
        int16_t speed = 0;
        int16_t load = 0;       // 以下数值保留舵机协议原始单位
        uint8_t voltage = 0;
        uint8_t temperature = 0;
        uint8_t moving = 0;
        int16_t current = 0;
        uint8_t status = 0;
    };

    bool init();
    bool set_target(const command &left, const command &right);
    bool read_feedback(state &left, state &right);
    bool set_torque(bool left_enabled, bool right_enabled);
}

#endif
