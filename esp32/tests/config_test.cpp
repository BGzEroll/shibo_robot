#include "config.h"
#include "nvs.h"

#include <cassert>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <cstdio>

namespace {std::string stored;}
esp_err_t nvs_flash_init(){return ESP_OK;}
esp_err_t nvs_open(const char *, int, nvs_handle_t *out){*out = 1; return ESP_OK;}
esp_err_t nvs_get_str(nvs_handle_t, const char *, char *out, size_t *length)
{
    if(stored.empty()){return ESP_ERR_NVS_NOT_FOUND;}
    if(out != nullptr){assert(*length >= stored.size()+1); memcpy(out,stored.c_str(),stored.size()+1);}
    *length = stored.size()+1;
    return ESP_OK;
}
esp_err_t nvs_set_str(nvs_handle_t, const char *, const char *value){stored = value; return ESP_OK;}
esp_err_t nvs_commit(nvs_handle_t){return ESP_OK;}
void nvs_close(nvs_handle_t){}

int main()
{
    assert(config::init());
    config::settings original = config::get();
    cJSON *json = config::to_json(original);
    config::settings copy;
    assert(config::from_json(json,copy));
    cJSON_Delete(json);
    assert(copy.balance.gain_poly[0][0][3] == original.balance.gain_poly[0][0][3]);
    std::ifstream input("docs/gain_poly_candidate.json");
    const std::string candidate{std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
    json = cJSON_Parse(candidate.c_str());
    assert(config::from_json(json,copy));
    assert(copy.balance.torque_scale == 1 && copy.balance.height_min_m == .031f);
    for(int side = 0; side < 2; side++)
    {
        for(int row = 0; row < 6; row++)
        {
            for(int col = 0; col < 4; col++)
            {assert(copy.balance.gain_poly[side][row][col] == original.balance.gain_poly[side][row][col]);}
        }
    }
    cJSON_Delete(json);
    // 局部更新、错误矩阵和错误标量不会部分写入配置。
    json = cJSON_Parse("{\"network\":{\"wifi_ssid\":\"robot\",\"wifi_password\":\"secret\",\"gamepad_address\":\"aa:bb:cc:dd:ee:ff\"},\"balance\":{\"pitch_offset_rad\":0.01}}");
    assert(config::from_json(json,copy));
    cJSON_Delete(json);
    assert(strcmp(copy.wifi_ssid,"robot") == 0 && copy.balance.pitch_offset_rad == .01f);
    const float previous = copy.balance.pitch_offset_rad;
    for(const char *bad : {
        "{\"balance\":{\"pitch_offset_rad\":0.1,\"gain_poly\":[1,2]}}",
        "{\"motor\":{\"kt_Nm_A\":0}}",
        "{\"leg\":{\"height_feedback\":1}}",
        "{\"network\":{\"gamepad_address\":\"no-address\"}}",
        "{\"balance\":{\"height_min_m\":0.09,\"height_max_m\":0.04}}"})
    {
        json = cJSON_Parse(bad);
        assert(!config::from_json(json,copy));
        assert(copy.balance.pitch_offset_rad == previous);
        cJSON_Delete(json);
    }
    assert(config::save(copy));
    assert(config::get().wifi_ssid[0] == '\0'); // 写入不会修改运行中的参数。
    assert(config::init());
    assert(strcmp(config::get().wifi_ssid,"robot") == 0);
    assert(config::get().balance.pitch_offset_rad == .01f);
    puts("config tests passed: candidate import, atomic validation, persistence boundary");
}
