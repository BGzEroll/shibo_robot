#include "web.h"

#include "config.h"
#include "controller/control.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "io/host.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include <cstdlib>
#include <cstring>

extern const char page_start[] asm("_binary_index_html_start");
extern const char page_end[] asm("_binary_index_html_end");

namespace web
{
    namespace
    {
        httpd_handle_t server = nullptr;
        esp_timer_handle_t restart_timer = nullptr;

        /**
         * @brief 发送 JSON 响应并释放传入的 JSON 对象
         *
         * @param[in] request HTTP 请求
         * @param[in] json 待发送对象，所有权交给本函数
         *
         * @return ESP_OK 响应成功；其他值表示序列化或发送失败
         */
        esp_err_t send_json(httpd_req_t *request, cJSON *json)
        {
            char *buffer = cJSON_PrintUnformatted(json);
            cJSON_Delete(json);
            if(buffer == nullptr)
            {
                return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
            }

            httpd_resp_set_type(request, "application/json");
            httpd_resp_set_hdr(request, "Cache-Control", "no-store");
            const esp_err_t result = httpd_resp_sendstr(request, buffer);
            cJSON_free(buffer);
            return result;
        }

        /**
         * @brief 返回嵌入的参数配置网页
         *
         * @param[in] request HTTP 请求
         *
         * @return ESP_OK 响应成功；其他值表示请求处理或响应发送失败
         */
        esp_err_t page(httpd_req_t *request)
        {
            httpd_resp_set_type(request, "text/html; charset=utf-8");
            return httpd_resp_send(request, page_start, page_end - page_start);
        }

        /**
         * @brief 返回本次启动加载的完整配置
         *
         * @param[in] request HTTP 请求
         *
         * @return ESP_OK 响应成功；其他值表示请求处理或响应发送失败
         */
        esp_err_t get_config(httpd_req_t *request)
        {
            return send_json(request, config::to_json(config::get()));
        }

        /**
         * @brief 返回默认参数
         *
         * @param[in] request HTTP 请求
         *
         * @return ESP_OK 响应成功；其他值表示请求处理或响应发送失败
         */
        esp_err_t defaults(httpd_req_t *request)
        {
            return send_json(request, config::to_json(config::settings{}));
        }

        /**
         * @brief 在小车停止时保存参数，响应后重启加载
         *
         * @param[in] request HTTP 请求
         *
         * @return ESP_OK 响应成功；其他值表示请求处理或响应发送失败
         */
        esp_err_t save_config(httpd_req_t *request)
        {
            if(request->content_len == 0 || request->content_len > 8191)
            {
                return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid configuration size");
            }

            char *buffer = static_cast<char *>(malloc(request->content_len + 1));
            if(buffer == nullptr)
            {
                return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
            }
            size_t used = 0;
            while(used < request->content_len)
            {
                const int32_t received = httpd_req_recv(request, buffer + used, request->content_len - used);
                if(received <= 0)
                {
                    free(buffer);
                    return ESP_FAIL;
                }
                used += received;
            }

            buffer[used] = '\0';
            cJSON *json = cJSON_Parse(buffer);
            free(buffer);

            config::settings next = config::get();
            const bool valid = config::from_json(json, next);
            cJSON_Delete(json);
            if(!valid)
            {
                return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST,
                    "Invalid parameter type, shape or range");
            }

            if(!control::begin_configuration())
            {
                httpd_resp_set_status(request, "409 Conflict");
                return httpd_resp_sendstr(request, "Use Start to stop the robot before saving");
            }

            if(!config::save(next))
            {
                control::end_configuration();
                return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS save failed");
            }

            httpd_resp_set_type(request, "application/json");
            const esp_err_t result = httpd_resp_sendstr(request, "{\"saved\":true,\"restarting\":true}");
            esp_timer_start_once(restart_timer, 750000);
            return result;
        }

        /**
         * @brief 返回控制、电池、视觉及手柄状态
         *
         * @param[in] request HTTP 请求
         *
         * @return ESP_OK 响应成功；其他值表示请求处理或响应发送失败
         */
        esp_err_t status(httpd_req_t *request)
        {
            const control::status state = control::get_status();
            const battery::state battery_state = battery::get();
            host::vision_measurement vision;

            cJSON *json = cJSON_CreateObject();
            cJSON_AddNumberToObject(json, "state", static_cast<uint8_t>(state.state));
            cJSON_AddNumberToObject(json, "mode", state.mode);
            cJSON_AddNumberToObject(json, "phase", state.phase);
            cJSON_AddBoolToObject(json, "enabled", state.enabled);
            cJSON_AddBoolToObject(json, "calibrated", state.calibration_success);
            cJSON_AddNumberToObject(json, "pitch_rad", state.pitch_rad);
            cJSON_AddNumberToObject(json, "speed_m_s", state.speed_m_s);
            cJSON_AddNumberToObject(json, "height_m", state.avg_leg_height);
            cJSON_AddNumberToObject(json, "left_torque_Nm", state.left_torque_Nm);
            cJSON_AddNumberToObject(json, "right_torque_Nm", state.right_torque_Nm);

            cJSON_AddNumberToObject(json, "battery_V", battery_state.voltage_V);
            cJSON_AddBoolToObject(json, "battery_low", battery_state.low);

            cJSON_AddBoolToObject(json, "gamepad", gamepad::connected());
            cJSON_AddBoolToObject(json, "vision", host::get_vision(vision));

            cJSON *list = cJSON_AddArrayToObject(json, "devices");
            gamepad::device devices[12];
            const size_t count = gamepad::get_devices(devices, 12);
            for(size_t i = 0; i < count; i++)
            {
                cJSON *item = cJSON_CreateObject();
                cJSON_AddStringToObject(item, "name", devices[i].name);
                cJSON_AddStringToObject(item, "address", devices[i].address);
                cJSON_AddNumberToObject(item, "rssi", devices[i].rssi);
                cJSON_AddItemToArray(list, item);
            }

            return send_json(request, json);
        }

        /**
         * @brief 处理 STA 重连并打印取得的 IP 地址
         *
         * @param[in] base 事件来源
         * @param[in] event 事件编号
         * @param[in] data 事件数据
         */
        void wifi_event(void *, esp_event_base_t base, int32_t event, void *data)
        {
            if(base == WIFI_EVENT && (event == WIFI_EVENT_STA_START || event == WIFI_EVENT_STA_DISCONNECTED))
            {
                esp_wifi_connect();
            }

            if(base == IP_EVENT && event == IP_EVENT_STA_GOT_IP)
            {
                const ip_event_got_ip_t *ip = static_cast<ip_event_got_ip_t *>(data);
                ESP_LOGI("web", "STA http://" IPSTR, IP2STR(&ip->ip_info.ip));
            }
        }
    }

    /**
     * @brief 初始化固定配置热点、可选 STA 和参数网页
     *
     * @return true Wi-Fi 和 HTTP 服务已启动
     * @return false 网络、定时器或 HTTP 服务初始化失败
     */
    bool init()
    {
        if(esp_netif_init() != ESP_OK || esp_event_loop_create_default() != ESP_OK){return false;}

        esp_netif_create_default_wifi_ap();
        const bool station = config::get().wifi_ssid[0] != '\0';

        if(station){esp_netif_create_default_wifi_sta();}

        wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
        if(esp_wifi_init(&wifi) != ESP_OK){return false;}

        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, nullptr);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, nullptr);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);

        wifi_config_t ap{};
        strcpy(reinterpret_cast<char *>(ap.ap.ssid), "SHIBO_ROBOT");
        strcpy(reinterpret_cast<char *>(ap.ap.password), "12345678");
        ap.ap.ssid_len = 11;
        ap.ap.channel = 1;
        ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
        ap.ap.max_connection = 2;
        if(esp_wifi_set_mode(station ? WIFI_MODE_APSTA : WIFI_MODE_AP) != ESP_OK ||
           esp_wifi_set_config(WIFI_IF_AP, &ap) != ESP_OK){return false;}

        if(station)
        {
            wifi_config_t sta{};
            memcpy(sta.sta.ssid, config::get().wifi_ssid, strlen(config::get().wifi_ssid));
            memcpy(sta.sta.password, config::get().wifi_password, strlen(config::get().wifi_password));
            if(esp_wifi_set_config(WIFI_IF_STA, &sta) != ESP_OK){return false;}
        }
        if(esp_wifi_start() != ESP_OK){return false;}

        esp_timer_create_args_t restart{};
        restart.callback = [](void *){esp_restart();};
        restart.name = "config_restart";
        if(esp_timer_create(&restart, &restart_timer) != ESP_OK){return false;}

        httpd_config_t http = HTTPD_DEFAULT_CONFIG();
        http.stack_size = 6144;
        if(httpd_start(&server, &http) != ESP_OK){return false;}

        const httpd_uri_t routes[] =
        {
            {"/", HTTP_GET, page, nullptr},
            {"/api/config", HTTP_GET, get_config, nullptr},
            {"/api/defaults", HTTP_GET, defaults, nullptr},
            {"/api/config", HTTP_POST, save_config, nullptr},
            {"/api/status", HTTP_GET, status, nullptr}
        };
        for(const auto &route : routes)
        {
            if(httpd_register_uri_handler(server, &route) != ESP_OK){return false;}
        }

        ESP_LOGI("web", "AP SHIBO_ROBOT, http://192.168.4.1");
        return true;
    }
}
