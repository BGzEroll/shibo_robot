#ifndef CONFIG_H
#define CONFIG_H

#include "controller/balance.h"
#include "hw/motor.h"
#include "cJSON.h"

namespace config
{
    struct settings
    {
        balance::config balance;
        motor::config motor;

        // FOC 正方向到小车前进方向的转换符号，仅取 1 或 -1。
        int8_t left_wheel_direction = -1;
        int8_t right_wheel_direction = -1;

        float max_linear_m_s = 0.6f;
        float max_yaw_rad_s = 2.0f;

        float arm_pitch_rad = 0.15f;
        float trip_pitch_rad = 0.5f;

        float battery_low_V = 7.4f;
        float battery_recover_V = 7.5f;

        float roll_p = 8.0f;
        float roll_i = 30.0f;
        float roll_d = 0.0f;
        float roll_limit_count = 450.0f;

        float height_poly[4] =
        {
            4.6289047954e-12f, -9.3936274976e-08f, 1.5357902969e-04f, 0.042041568108f
        };

        // 舵机几何拟合表示腿长；质心到轮轴距离必须通过实测修正。
        float height_com_scale = 1.0f;
        float height_com_offset_m = 0.0f;
        bool height_feedback = false;

        char wifi_ssid[33] = {};
        char wifi_password[65] = {};

        char gamepad_address[18] = {};
    };

    bool init();
    const settings &get();
    cJSON *to_json(const settings &value);
    bool from_json(const cJSON *json, settings &value);
    bool save(const settings &value);
}

#endif
