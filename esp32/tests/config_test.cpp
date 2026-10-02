#include "config.h"
#include "nvs.h"
#include <cassert>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <cstdio>

namespace
{
    std::string stored;
}

/**
 * @brief 模拟 NVS 初始化成功
 *
 * @return ESP_OK
 */
esp_err_t nvs_flash_init()
{
    return ESP_OK;
}

/**
 * @brief 为测试提供固定 NVS 句柄
 *
 * @param[out] out 输出句柄
 *
 * @return ESP_OK
 */
esp_err_t nvs_open(const char *, int32_t, nvs_handle_t *out)
{
    *out = 1;
    return ESP_OK;
}

/**
 * @brief 读取内存中保存的配置字符串
 *
 * @param[out] out 接收缓冲区，nullptr 时仅查询长度
 * @param[in,out] length 输入缓冲区容量，输出含结束符的字符串长度
 *
 * @return ESP_OK 读取成功；ESP_ERR_NVS_NOT_FOUND 尚未保存配置
 */
esp_err_t nvs_get_str(nvs_handle_t, const char *, char *out, size_t *length)
{
    if(stored.empty()){return ESP_ERR_NVS_NOT_FOUND;}
    if(out != nullptr)
    {
        assert(*length >= stored.size() + 1);
        memcpy(out, stored.c_str(), stored.size() + 1);
    }
    *length = stored.size() + 1;
    return ESP_OK;
}

/**
 * @brief 将配置字符串保存到测试内存
 *
 * @param[in] value 配置字符串
 *
 * @return ESP_OK
 */
esp_err_t nvs_set_str(nvs_handle_t, const char *, const char *value)
{
    stored = value;
    return ESP_OK;
}

/**
 * @brief 模拟 NVS 提交成功
 *
 * @return ESP_OK
 */
esp_err_t nvs_commit(nvs_handle_t)
{
    return ESP_OK;
}

/**
 * @brief 模拟关闭 NVS 句柄
 */
void nvs_close(nvs_handle_t)
{
}

/**
 * @brief 验证配置导入、原子校验和持久化边界
 *
 * @return 0 全部断言通过
 */
int32_t main()
{
    assert(config::init());
    config::settings original = config::get();
    cJSON *json = config::to_json(original);
    config::settings copy;
    assert(config::from_json(json, copy));
    cJSON_Delete(json);
    assert(copy.balance.gain_poly[0][0][3] == original.balance.gain_poly[0][0][3]);

    std::ifstream input("docs/gain_poly_candidate.json");
    const std::string candidate{
        std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    json = cJSON_Parse(candidate.c_str());
    assert(config::from_json(json, copy));
    assert(copy.balance.torque_scale == 1 && copy.balance.height_min_m == 0.031f);
    for(uint32_t side = 0; side < 2; side++)
    {
        for(uint32_t row = 0; row < 6; row++)
        {
            for(uint32_t col = 0; col < 4; col++)
            {
                assert(copy.balance.gain_poly[side][row][col] ==
                    original.balance.gain_poly[side][row][col]);
            }
        }
    }
    cJSON_Delete(json);

    // 局部更新、错误矩阵和错误标量不会部分写入配置。
    json = cJSON_Parse(
        "{\"network\":{\"wifi_ssid\":\"robot\",\"wifi_password\":\"secret\","
        "\"gamepad_address\":\"aa:bb:cc:dd:ee:ff\"},"
        "\"balance\":{\"pitch_offset_rad\":0.01}}");
    assert(config::from_json(json, copy));
    cJSON_Delete(json);
    assert(strcmp(copy.wifi_ssid, "robot") == 0 && copy.balance.pitch_offset_rad == 0.01f);

    const float previous = copy.balance.pitch_offset_rad;
    const char *invalid_configs[] =
    {
        "{\"balance\":{\"pitch_offset_rad\":0.1,\"gain_poly\":[1,2]}}",
        "{\"motor\":{\"kt_Nm_A\":0}}",
        "{\"leg\":{\"height_feedback\":1}}",
        "{\"network\":{\"gamepad_address\":\"no-address\"}}",
        "{\"balance\":{\"height_min_m\":0.09,\"height_max_m\":0.04}}"
    };
    for(const char *bad : invalid_configs)
    {
        json = cJSON_Parse(bad);
        assert(!config::from_json(json, copy));
        assert(copy.balance.pitch_offset_rad == previous);
        cJSON_Delete(json);
    }

    assert(config::save(copy));
    assert(config::get().wifi_ssid[0] == '\0'); // 写入不会修改运行中的参数。
    assert(config::init());
    assert(strcmp(config::get().wifi_ssid, "robot") == 0);
    assert(config::get().balance.pitch_offset_rad == 0.01f);
    puts("config tests passed: candidate import, atomic validation, persistence boundary");
}
