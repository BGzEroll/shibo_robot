#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stdint.h>

namespace gamepad
{
    namespace button
    {
        constexpr uint16_t A = 0x0001;
        constexpr uint16_t B = 0x0002;
        constexpr uint16_t X = 0x0004;
        constexpr uint16_t Y = 0x0008;
        constexpr uint16_t SHARE = 0x0010;
        constexpr uint16_t START = 0x0020;
        constexpr uint16_t SELECT = 0x0040;
        constexpr uint16_t XBOX = 0x0080;
        constexpr uint16_t LB = 0x0100;
        constexpr uint16_t RB = 0x0200;
        constexpr uint16_t LS = 0x0400;
        constexpr uint16_t RS = 0x0800;
        constexpr uint16_t UP = 0x1000;
        constexpr uint16_t LEFT = 0x2000;
        constexpr uint16_t RIGHT = 0x4000;
        constexpr uint16_t DOWN = 0x8000;
    }

    struct state
    {
        uint64_t timestamp_us = 0;
        uint32_t session = 0;
        uint16_t buttons = 0;
        float axes[6] = {};
        bool connected = false;
    };

    struct device
    {
        char address[18] = {};
        char name[32] = {};
        int8_t rssi = 0;
        bool xbox = false;
    };

    struct discovery
    {
        char target[18] = {};
        bool scanning = false;
        int32_t scan_error = 0;
    };

    bool init();
    bool get_state(state &out);
    bool scan_devices();
    uint8_t get_devices(device *out, uint8_t capacity);
    discovery get_discovery();
    bool select_device(uint8_t index);
}

#endif
