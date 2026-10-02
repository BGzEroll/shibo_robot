#include "input.h"

#include "action.h"
#include "hw/gamepad.h"
#include "io/host.h"
#include "sys_time.h"

#include <cmath>
#include <cstring>

namespace control::input_router
{
    namespace
    {
        input_source previous_source = input_source::NONE;
        uint32_t previous_stream = 0;
        uint16_t press_count[16] = {};
        bool was_fresh = false;

        float axis(float value)
        {
            return fabsf(value) <= 0.05f ? 0.0f : copysignf((fabsf(value) - 0.05f) / 0.95f, value);
        }
    }

    void init()
    {
        previous_source = input_source::NONE;
        previous_stream = 0;
        memset(press_count, 0, sizeof(press_count));
        was_fresh = false;
    }

    /** @brief Xbox 优先，上位机次之；切换来源或恢复连接时丢弃历史按键 */
    void update(action::mode mode, float max_linear_vel,
        float max_steer_vel, control_input &out)
    {
        out = {};
        remote_input snapshot;
        if(gamepad::connected())
        {
            out.source = input_source::XBOX;
            gamepad::get(snapshot);
        }
        else
        {
            out.source = input_source::HOST;
            host::get_input(snapshot);
        }
        const uint32_t now = static_cast<uint32_t>(sys_time::get_us_tick());
        out.timestamp_us = snapshot.timestamp_us;
        out.fresh = snapshot.valid && static_cast<uint32_t>(now - snapshot.timestamp_us) <= 250000;
        uint16_t pressed = 0;
        const bool continuous = was_fresh && out.fresh && previous_source == out.source &&
            previous_stream == snapshot.stream_id;
        for(int i = 0; i < 16; i++)
        {
            if(continuous && press_count[i] != snapshot.press_count[i]){pressed |= 1 << i;}
            press_count[i] = snapshot.press_count[i];
        }
        previous_source = out.source;
        previous_stream = snapshot.stream_id;
        was_fresh = out.fresh;
        if(!out.fresh){return;}
        const uint16_t held = snapshot.buttons;
        const bool modifier = held & buttons::SELECT;
        out.linear = axis(snapshot.axes[3]) * max_linear_vel;
        if(out.linear < 0.0f){out.linear *= 0.8f;}
        out.yaw = -axis(snapshot.axes[0]) * max_steer_vel;
        if(modifier)
        {
            out.camera_direction = (held & buttons::UP ? 1 : 0) - (held & buttons::DOWN ? 1 : 0);
        }
        else
        {
            out.leg_height_direction = (held & buttons::DOWN ? 1 : 0) - (held & buttons::UP ? 1 : 0);
            out.roll_direction = (held & buttons::RIGHT ? 1 : 0) - (held & buttons::LEFT ? 1 : 0);
        }
        if(pressed & buttons::START){out.action = action_request::STOP; return;}
        if(mode == action::mode::STOP)
        {
            if(pressed & buttons::RB){out.action = action_request::BOOT;}
        }
        else if(mode == action::mode::BALANCE)
        {
            out.reset_leg = (pressed & buttons::LS) && fabsf(out.linear) < max_linear_vel * 0.05f;
            if(modifier)
            {
                if(pressed & buttons::X){out.action = action_request::KICK_PLACE;}
                else if(pressed & buttons::Y){out.action = action_request::KICK_RUN;}
                else if(pressed & buttons::B){out.action = action_request::RESET_BALANCE;}
            }
            else
            {
                if(pressed & buttons::B){out.action = action_request::JUMP_RIGHT;}
                else if(pressed & buttons::X){out.action = action_request::JUMP_LEFT;}
                else if(pressed & buttons::A){out.action = action_request::JUMP_BACKWARD;}
                else if(pressed & buttons::Y){out.action = action_request::JUMP_FORWARD;}
                else if(pressed & buttons::RS){out.action = action_request::JUMP_IN_PLACE;}
                else if(pressed & buttons::LB){out.action = action_request::SIT;}
            }
        }
        else if(mode == action::mode::SIT || mode == action::mode::MIDDLE_CALIBRATION)
        {
            out.disable_leg_torque = held & buttons::LS;
            if(modifier && (pressed & buttons::LB)){out.action = action_request::MIDDLE_CALIBRATION;}
            else if(pressed & buttons::RB){out.action = action_request::EXIT;}
        }
        else if(mode == action::mode::KICK_PLACE || mode == action::mode::KICK_RUN)
        {
            if(modifier && (pressed & buttons::B)){out.action = action_request::KICK_EXIT;}
            else if(modifier && (pressed & buttons::X)){out.action = action_request::KICK_PLACE;}
            else if(modifier && (pressed & buttons::Y)){out.action = action_request::KICK_RUN;}
        }
    }
}
