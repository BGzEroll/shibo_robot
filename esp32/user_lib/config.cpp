#include "config.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

namespace config
{
    namespace
    {
        settings current;
        constexpr const char *TAG = "config";

        /** @brief 遍历可配置浮点参数并统一执行读写与范围校验 */
        template<typename Visitor>
        bool visit(settings &value, Visitor visitor)
        {
#define FIELD(group, member, low, high) \
            if(!visitor(group, #member, value.member, low, high)){return false;}
            FIELD("balance", balance.wheel_radius_m, 0.005f, 0.1f)
            FIELD("balance", balance.model_height_m, 0.01f, 0.2f)
            FIELD("balance", balance.height_min_m, 0.01f, 0.2f)
            FIELD("balance", balance.height_max_m, 0.01f, 0.2f)
            FIELD("balance", balance.pitch_offset_rad, -0.2f, 0.2f)
            FIELD("balance", balance.torque_scale, 0.001f, 2.0f)
            FIELD("balance", balance.reference_filter_s, 0.001f, 1.0f)
            FIELD("balance", balance.linear_integral_limit_m, 0.001f, 3.0f)
            FIELD("balance", balance.yaw_integral_limit_rad, 0.001f, 3.0f)
            FIELD("motor", motor.phase_resistance_ohm, 0.1f, 50.0f)
            FIELD("motor", motor.kt_Nm_A, 0.001f, 1.0f)
            FIELD("motor", motor.ke_V_s_rad, 0.001f, 1.0f)
            FIELD("motor", motor.bus_voltage_V, 4.0f, 12.6f)
            FIELD("motor", motor.torque_limit_Nm, 0.001f, 0.2f)
            FIELD("motion", max_linear_m_s, 0.01f, 1.0f)
            FIELD("motion", max_yaw_rad_s, 0.01f, 6.5f)
            FIELD("motion", arm_pitch_rad, 0.02f, 0.3f)
            FIELD("motion", trip_pitch_rad, 0.15f, 0.8f)
            FIELD("motion", battery_low_V, 6.0f, 8.4f)
            FIELD("motion", battery_recover_V, 6.0f, 8.4f)
            FIELD("leg", roll_p, 0.0f, 100.0f)
            FIELD("leg", roll_i, 0.0f, 100.0f)
            FIELD("leg", roll_d, 0.0f, 10.0f)
            FIELD("leg", roll_limit_count, 1.0f, 450.0f)
            FIELD("leg", height_com_scale, 0.1f, 2.0f)
            FIELD("leg", height_com_offset_m, -0.1f, 0.1f)
#undef FIELD
            return true;
        }

        const char *field_name(const char *name)
        {
            const char *separator = strchr(name, '.');
            return separator == nullptr ? name : separator + 1;
        }

        bool read_string(const cJSON *json, const char *name, char *out, size_t capacity)
        {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(json, name);
            if(item == nullptr){return true;}
            if(!cJSON_IsString(item) || strlen(item->valuestring) >= capacity){return false;}
            strcpy(out, item->valuestring);
            return true;
        }

        bool read_array(const cJSON *json, float *out, int size, float bound)
        {
            if(!cJSON_IsArray(json) || cJSON_GetArraySize(json) != size){return false;}
            for(int i = 0; i < size; i++)
            {
                const cJSON *item = cJSON_GetArrayItem(json, i);
                if(!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) ||
                   fabs(item->valuedouble) > bound){return false;}
                out[i] = static_cast<float>(item->valuedouble);
            }
            return true;
        }
    }

    /** @brief 输出完整参数对象，省略字段的写入保留原值 */
    cJSON *to_json(const settings &value)
    {
        cJSON *json = cJSON_CreateObject();
        if(json == nullptr){return nullptr;}
        for(const char *name : {"balance", "motor", "motion", "leg", "network"})
        {
            cJSON_AddObjectToObject(json, name);
        }
        settings copy = value;
        visit(copy, [json](const char *group, const char *name, float &number, float, float)
        {
            return cJSON_AddNumberToObject(cJSON_GetObjectItemCaseSensitive(json, group),
                field_name(name), number) != nullptr;
        });
        cJSON *balance = cJSON_GetObjectItemCaseSensitive(json, "balance");
        cJSON *poly = cJSON_AddArrayToObject(balance, "gain_poly");
        for(const auto &side : value.balance.gain_poly)
        {
            cJSON *rows = cJSON_CreateArray();
            cJSON_AddItemToArray(poly, rows);
            for(const auto &row : side)
            {
                cJSON_AddItemToArray(rows, cJSON_CreateFloatArray(row, 4));
            }
        }
        cJSON *leg = cJSON_GetObjectItemCaseSensitive(json, "leg");
        cJSON_AddBoolToObject(leg, "height_feedback", value.height_feedback);
        cJSON_AddItemToObject(leg, "height_poly", cJSON_CreateFloatArray(value.height_poly, 4));
        cJSON *network = cJSON_GetObjectItemCaseSensitive(json, "network");
        cJSON_AddStringToObject(network, "wifi_ssid", value.wifi_ssid);
        cJSON_AddStringToObject(network, "wifi_password", value.wifi_password);
        cJSON_AddStringToObject(network, "gamepad_address", value.gamepad_address);
        cJSON_AddNumberToObject(json, "version", 1);
        return json;
    }

    /** @brief 解析并校验参数；仅成功时替换调用方配置 */
    bool from_json(const cJSON *json, settings &value)
    {
        if(!cJSON_IsObject(json)){return false;}
        settings next = value;
        const cJSON *version = cJSON_GetObjectItemCaseSensitive(json, "version");
        if(version != nullptr && (!cJSON_IsNumber(version) || version->valuedouble != 1))
        {
            return false;
        }
        for(const char *name : {"balance", "motor", "motion", "leg", "network"})
        {
            const cJSON *group = cJSON_GetObjectItemCaseSensitive(json, name);
            if(group != nullptr && !cJSON_IsObject(group)){return false;}
        }
        if(!visit(next, [json](const char *group, const char *name,
            float &number, float low, float high)
        {
            const cJSON *item = cJSON_GetObjectItemCaseSensitive(
                cJSON_GetObjectItemCaseSensitive(json, group), field_name(name));
            if(item == nullptr){return true;}
            if(!cJSON_IsNumber(item) || !std::isfinite(item->valuedouble) ||
               item->valuedouble < low || item->valuedouble > high){return false;}
            number = static_cast<float>(item->valuedouble);
            return true;
        })){return false;}

        const cJSON *poly = cJSON_GetObjectItemCaseSensitive(
            cJSON_GetObjectItemCaseSensitive(json, "balance"), "gain_poly");
        if(poly != nullptr)
        {
            if(!cJSON_IsArray(poly) || cJSON_GetArraySize(poly) != 2){return false;}
            for(int side = 0; side < 2; side++)
            {
                const cJSON *rows = cJSON_GetArrayItem(poly, side);
                if(!cJSON_IsArray(rows) || cJSON_GetArraySize(rows) != 6){return false;}
                for(int row = 0; row < 6; row++)
                {
                    if(!read_array(cJSON_GetArrayItem(rows, row),
                        next.balance.gain_poly[side][row], 4, 1.0e6f)){return false;}
                }
            }
        }
        const cJSON *leg = cJSON_GetObjectItemCaseSensitive(json, "leg");
        const cJSON *height = cJSON_GetObjectItemCaseSensitive(leg, "height_feedback");
        if(height != nullptr)
        {
            if(!cJSON_IsBool(height)){return false;}
            next.height_feedback = cJSON_IsTrue(height);
        }
        height = cJSON_GetObjectItemCaseSensitive(leg, "height_poly");
        if(height != nullptr && !read_array(height, next.height_poly, 4, 1.0f)){return false;}
        const cJSON *network = cJSON_GetObjectItemCaseSensitive(json, "network");
        if(!read_string(network, "wifi_ssid", next.wifi_ssid, sizeof(next.wifi_ssid)) ||
           !read_string(network, "wifi_password", next.wifi_password, sizeof(next.wifi_password)) ||
           !read_string(network, "gamepad_address", next.gamepad_address, sizeof(next.gamepad_address)))
        {
            return false;
        }
        if(next.gamepad_address[0] != '\0')
        {
            unsigned bytes[6];
            int length = 0;
            if(sscanf(next.gamepad_address, "%2x:%2x:%2x:%2x:%2x:%2x%n",
                &bytes[0], &bytes[1], &bytes[2], &bytes[3], &bytes[4], &bytes[5], &length) != 6 ||
                length != 17){return false;}
        }
        if(next.balance.height_min_m >= next.balance.height_max_m ||
           next.balance.model_height_m < next.balance.height_min_m ||
           next.balance.model_height_m > next.balance.height_max_m ||
           next.arm_pitch_rad >= next.trip_pitch_rad ||
           next.battery_low_V >= next.battery_recover_V){return false;}
        value = next;
        return true;
    }

    /** @brief 初始化 NVS 并读取已保存参数，不主动擦除已有存储 */
    bool init()
    {
        if(nvs_flash_init() != ESP_OK){return false;}
        nvs_handle_t handle;
        const esp_err_t opened = nvs_open("robot", NVS_READONLY, &handle);
        if(opened == ESP_ERR_NVS_NOT_FOUND){return true;}
        if(opened != ESP_OK){return false;}
        size_t size = 0;
        const esp_err_t found = nvs_get_str(handle, "config", nullptr, &size);
        if(found == ESP_ERR_NVS_NOT_FOUND){nvs_close(handle); return true;}
        if(found != ESP_OK || size > 8192){nvs_close(handle); return false;}
        char *buffer = static_cast<char *>(malloc(size));
        if(buffer == nullptr){nvs_close(handle); return false;}
        const esp_err_t read = nvs_get_str(handle, "config", buffer, &size);
        nvs_close(handle);
        cJSON *json = read == ESP_OK ? cJSON_Parse(buffer) : nullptr;
        const bool valid = from_json(json, current);
        cJSON_Delete(json);
        free(buffer);
        if(!valid){ESP_LOGE(TAG, "Invalid stored parameters");}
        return valid;
    }

    const settings &get(){return current;}

    /** @brief 保存参数，运行中的配置只在下一次启动时加载 */
    bool save(const settings &value)
    {
        cJSON *json = to_json(value);
        char *buffer = cJSON_PrintUnformatted(json);
        cJSON_Delete(json);
        if(buffer == nullptr){return false;}
        nvs_handle_t handle;
        esp_err_t result = nvs_open("robot", NVS_READWRITE, &handle);
        if(result == ESP_OK)
        {
            result = nvs_set_str(handle, "config", buffer);
            if(result == ESP_OK){result = nvs_commit(handle);}
            nvs_close(handle);
        }
        cJSON_free(buffer);
        return result == ESP_OK;
    }
}
