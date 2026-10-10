#include "motor_voltage.cpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{
    battery::state voltage;
    uint32_t reads = 0;
}

namespace battery
{
    /**
     * @brief 提供测试电压并记录读取次数
     *
     * @return 测试中的电池状态
     */
    state get()
    {
        reads++;
        return voltage;
    }
}

/**
 * @brief 验证实测电压更新周期、无效测量和定点电压换算
 *
 * @return 0 全部断言通过
 */
int32_t main()
{
    uint64_t now_us = 1000000;
    assert(!motor::update_bus_voltage(now_us));
    voltage = {8.4f, true, false};
    assert(!motor::update_bus_voltage(now_us + 49999));
    assert(reads == 1);

    const float samples[] = {8.4f, 7.4f, 6.0f};
    const int32_t torques[] = {-25000, 0, 25000};
    const int32_t speeds[] = {-12000, 0, 12000};
    for(float sample : samples)
    {
        now_us += 50000;
        voltage = {sample, true, sample < 7.4f};
        assert(motor::update_bus_voltage(now_us));
        const uint32_t count = reads;
        assert(motor::update_bus_voltage(now_us + 49999));
        assert(reads == count);
        assert(fabsf(motor::alignment_uq * sample / 32768.0f - 3.0f) < 0.001f);

        for(int32_t torque : torques)
        {
            for(int32_t speed : speeds)
            {
                const int64_t uq = (static_cast<int64_t>(torque) * motor::torque_gain_q16 >> 16) +
                    (static_cast<int64_t>(speed) * motor::bemf_gain_q14 >> 14);
                const double expected = motor::settings.phase_resistance_ohm *
                    (torque * 1.0e-6) / motor::settings.kt_Nm_A +
                    motor::settings.ke_V_s_rad * (speed * 0.001);
                assert(fabs(uq * sample / 32768.0 - expected) < 0.002);
            }
        }
    }

    voltage.valid = false;
    now_us += 50000;
    assert(!motor::update_bus_voltage(now_us));
    voltage = {0.0f, true, true};
    now_us += 50000;
    assert(!motor::update_bus_voltage(now_us));
    voltage = {7.8f, true, false};
    now_us += 50000;
    assert(motor::update_bus_voltage(now_us));
    puts("motor tests passed: measured bus voltage, 50 ms updates, invalid measurement, alignment, torque, BEMF");
}
