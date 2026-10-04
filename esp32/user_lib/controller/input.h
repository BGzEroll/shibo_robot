#ifndef CONTROL_INPUT_H
#define CONTROL_INPUT_H

#include <stdint.h>

namespace control
{
    namespace buttons
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

    enum class input_source : uint8_t
    {
        NONE = 0,
        XBOX,
        HOST
    };

    /**
     * @brief 所有遥控来源共享的原始输入快照
     */
    struct remote_input
    {
        uint32_t stream_id = 0;
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        uint16_t press_count[16] = {};

        float axes[6] = {};

        bool valid = false;
    };

    /**
     * @brief 输入路由输出的速度和按键快照
     */
    struct control_input
    {
        input_source source = input_source::NONE;
        uint32_t timestamp_us = 0;

        float linear = 0.0f;
        float yaw = 0.0f;

        uint16_t held = 0;
        uint16_t pressed = 0;

        bool fresh = false;
    };

    namespace input_router
    {
        void init();
        void update(control_input &out);
    }
}

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
