#include "gamepad.h"

namespace gamepad
{
    /**
     * @brief 解码 Xbox Series X 的 16 字节 HID 输入报告
     *
     * @param[in] data HID 报告缓冲区
     * @param[in] length 报告长度，单位字节
     * @param[in,out] out 输入快照，保留并更新按键边沿计数
     *
     * @return true 报告格式有效并已解码
     * @return false 报告长度或扳机数值无效
     */
    bool parse_report(const uint8_t *data, size_t length, control::remote_input &out)
    {
        if(data == nullptr || length != 16){return false;}

        uint16_t buttons = 0;
        const uint16_t main[] =
        {
            control::buttons::A, control::buttons::B, 0, control::buttons::X,
            control::buttons::Y, 0, control::buttons::LB, control::buttons::RB
        };
        const uint16_t center[] =
        {
            0, 0, control::buttons::SELECT, control::buttons::START,
            control::buttons::XBOX, control::buttons::LS, control::buttons::RS, 0
        };

        for(uint32_t i = 0; i < 8; i++)
        {
            if(data[13] & (1 << i)){buttons |= main[i];}
            if(data[14] & (1 << i)){buttons |= center[i];}
        }
        if(data[15] & 1){buttons |= control::buttons::SHARE;}

        const uint8_t direction = data[12];
        if(direction == 1 || direction == 2 || direction == 8){buttons |= control::buttons::UP;}
        if(direction >= 2 && direction <= 4){buttons |= control::buttons::RIGHT;}
        if(direction >= 4 && direction <= 6){buttons |= control::buttons::DOWN;}
        if(direction >= 6 && direction <= 8){buttons |= control::buttons::LEFT;}

        const uint16_t pressed = buttons & ~out.buttons;
        for(uint32_t i = 0; i < 16; i++)
        {
            if(pressed & (1 << i)){out.press_count[i]++;}
        }

        for(uint32_t i = 0; i < 6; i++)
        {
            const uint16_t raw = data[i * 2] | static_cast<uint16_t>(data[i * 2 + 1]) << 8;
            out.axes[i] = i < 4 ? (static_cast<int32_t>(raw) - 32768) / 32768.0f : raw / 1023.0f;
            if(i == 1 || i == 3){out.axes[i] = -out.axes[i];}
            if(i >= 4 && raw > 1023){return false;}
        }

        out.buttons = buttons;
        out.valid = true;
        return true;
    }
}
