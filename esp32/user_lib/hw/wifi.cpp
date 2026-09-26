#include "wifi.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include <string.h>
#include <stdio.h>

namespace wifi
{
    namespace
    {
        constexpr char AP_SSID[] = "Shibo-Robot";
        constexpr char AP_PASSWORD[] = "shiborobot";
        constexpr uint8_t MAX_NETWORKS = 16;
        constexpr uint32_t CONNECT_TIMEOUT_MS = 12000;
        constexpr uint32_t RETRY_MS = 5000;

        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        state latest;
        network networks[MAX_NETWORKS];
        uint8_t network_count = 0;
        uint32_t connect_started_ms = 0;
        uint32_t last_attempt_ms = 0;
        bool started = false;

        /**
         * @brief 启动可配网热点，同时保留 STA 扫描能力
         */
        bool start_ap()
        {
            if(esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK)
            {
                return false;
            }
            wifi_config_t config = {};
            memcpy(config.ap.ssid, AP_SSID, sizeof(AP_SSID) - 1);
            memcpy(config.ap.password, AP_PASSWORD, sizeof(AP_PASSWORD) - 1);
            config.ap.ssid_len = sizeof(AP_SSID) - 1;
            config.ap.max_connection = 2;
            config.ap.authmode = WIFI_AUTH_WPA2_PSK;
            if(esp_wifi_set_config(WIFI_IF_AP, &config) != ESP_OK)
            {
                return false;
            }
            portENTER_CRITICAL(&lock);
            latest.ap_active = true;
            portEXIT_CRITICAL(&lock);
            return true;
        }

        /**
         * @brief 处理 Wi-Fi 扫描和连接结果
         */
        void event(void *, esp_event_base_t base, int32_t id, void *data)
        {
            if(base == WIFI_EVENT && id == WIFI_EVENT_SCAN_DONE)
            {
                uint16_t count = MAX_NETWORKS;
                static wifi_ap_record_t records[MAX_NETWORKS];
                esp_wifi_scan_get_ap_records(&count, records);
                portENTER_CRITICAL(&lock);
                network_count = static_cast<uint8_t>(count);
                for(uint8_t i = 0; i < network_count; i++)
                {
                    networks[i] = {};
                    memcpy(networks[i].ssid, records[i].ssid, 32);
                    networks[i].rssi = records[i].rssi;
                    networks[i].secured = records[i].authmode != WIFI_AUTH_OPEN;
                }
                latest.scanning = false;
                portEXIT_CRITICAL(&lock);
            }
            else if(base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
            {
                portENTER_CRITICAL(&lock);
                const bool was_connected = latest.connected;
                latest.connected = false;
                latest.ip[0] = 0;
                portEXIT_CRITICAL(&lock);
                if(was_connected)
                {
                    portENTER_CRITICAL(&lock);
                    connect_started_ms = pdTICKS_TO_MS(xTaskGetTickCount());
                    portEXIT_CRITICAL(&lock);
                }
            }
            else if(base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
            {
                const auto *ip = static_cast<ip_event_got_ip_t *>(data);
                const bool close_ap = get_state().ap_active;
                portENTER_CRITICAL(&lock);
                latest.connected = true;
                snprintf(latest.ip, sizeof(latest.ip), IPSTR,
                    IP2STR(&ip->ip_info.ip));
                portEXIT_CRITICAL(&lock);
                if(close_ap && esp_wifi_set_mode(WIFI_MODE_STA) == ESP_OK)
                {
                    portENTER_CRITICAL(&lock);
                    latest.ap_active = false;
                    portEXIT_CRITICAL(&lock);
                }
            }
        }

        /**
         * @brief 在连接超时后开放配置热点
         */
        void task(void *)
        {
            while(true)
            {
                portENTER_CRITICAL(&lock);
                const state snapshot = latest;
                const uint32_t since = connect_started_ms;
                const uint32_t last_attempt = last_attempt_ms;
                portEXIT_CRITICAL(&lock);
                const uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
                if(snapshot.ssid[0] && !snapshot.connected &&
                    !snapshot.ap_active &&
                    now - since >= CONNECT_TIMEOUT_MS)
                {
                    start_ap();
                }
                if(snapshot.ssid[0] && !snapshot.connected &&
                    !snapshot.scanning && now - last_attempt >= RETRY_MS)
                {
                    portENTER_CRITICAL(&lock);
                    last_attempt_ms = now;
                    portEXIT_CRITICAL(&lock);
                    esp_wifi_connect();
                }
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }

        /**
         * @brief 将目标网络应用到 STA 并开始连接
         */
        bool start_station(const char *ssid, const char *password)
        {
            if(get_state().connected){esp_wifi_disconnect();}
            wifi_config_t config = {};
            memcpy(config.sta.ssid, ssid, strlen(ssid));
            memcpy(config.sta.password, password, strlen(password));
            if(esp_wifi_set_config(WIFI_IF_STA, &config) != ESP_OK)
            {
                return false;
            }
            portENTER_CRITICAL(&lock);
            strncpy(latest.ssid, ssid, sizeof(latest.ssid) - 1);
            latest.connected = false;
            latest.ip[0] = 0;
            connect_started_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            last_attempt_ms = connect_started_ms;
            portEXIT_CRITICAL(&lock);
            esp_wifi_connect();
            return true;
        }
    }

    /**
     * @brief 初始化 STA、配网热点和扫描事件
     *
     * @return true Wi-Fi 已启动
     */
    bool init()
    {
        if(started){return true;}
        if(esp_netif_init() != ESP_OK ||
            esp_event_loop_create_default() != ESP_OK)
        {
            return false;
        }
        esp_netif_create_default_wifi_sta();
        esp_netif_create_default_wifi_ap();
        wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
        if(esp_wifi_init(&config) != ESP_OK ||
            esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
            esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                event, nullptr) != ESP_OK ||
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                event, nullptr) != ESP_OK)
        {
            return false;
        }

        char ssid[33] = {};
        char password[65] = {};
        nvs_handle_t storage;
        if(nvs_open("shibo_wifi", NVS_READONLY, &storage) == ESP_OK)
        {
            size_t size = sizeof(ssid);
            nvs_get_str(storage, "ssid", ssid, &size);
            size = sizeof(password);
            nvs_get_str(storage, "password", password, &size);
            nvs_close(storage);
        }

        if(!start_ap()){return false;}
        if(esp_wifi_start() != ESP_OK){return false;}
        if(ssid[0]){start_station(ssid, password);}
        if(xTaskCreatePinnedToCore(task, "wifi", 3072, nullptr, 2,
            nullptr, 0) != pdPASS)
        {
            return false;
        }
        started = true;
        return true;
    }

    /**
     * @brief 获取当前连接状态
     */
    state get_state()
    {
        portENTER_CRITICAL(&lock);
        const state snapshot = latest;
        portEXIT_CRITICAL(&lock);
        return snapshot;
    }

    /**
     * @brief 异步扫描附近网络
     */
    bool scan()
    {
        portENTER_CRITICAL(&lock);
        const bool busy = latest.scanning;
        if(!busy){latest.scanning = true;}
        portEXIT_CRITICAL(&lock);
        if(busy){return false;}
        if(esp_wifi_scan_start(nullptr, false) == ESP_OK){return true;}
        portENTER_CRITICAL(&lock);
        latest.scanning = false;
        portEXIT_CRITICAL(&lock);
        return false;
    }

    /**
     * @brief 复制最近一次扫描结果
     */
    uint8_t get_networks(network *out, uint8_t capacity)
    {
        portENTER_CRITICAL(&lock);
        const uint8_t count = network_count < capacity ? network_count : capacity;
        memcpy(out, networks, count * sizeof(network));
        portEXIT_CRITICAL(&lock);
        return count;
    }

    /**
     * @brief 保存并连接目标网络
     */
    bool connect(const char *ssid, const char *password)
    {
        const size_t ssid_size = strlen(ssid);
        const size_t password_size = strlen(password);
        if(ssid_size == 0 || ssid_size > 32 || password_size > 64)
        {
            return false;
        }
        nvs_handle_t storage;
        if(nvs_open("shibo_wifi", NVS_READWRITE, &storage) != ESP_OK)
        {
            return false;
        }
        const bool saved = nvs_set_str(storage, "ssid", ssid) == ESP_OK &&
            nvs_set_str(storage, "password", password) == ESP_OK &&
            nvs_commit(storage) == ESP_OK;
        nvs_close(storage);
        return saved && start_station(ssid, password);
    }
}
