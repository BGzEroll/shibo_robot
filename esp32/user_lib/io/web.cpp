#include "web.h"

#include "hw/gamepad.h"
#include "hw/wifi.h"
#include "cJSON.h"
#include "esp_http_server.h"

#include <string.h>

namespace web
{
    namespace
    {
        constexpr char PAGE_ORIGIN[] = "https://bgzeroll.github.io";
        constexpr char PORTAL[] = R"html(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Shibo 配网</title><style>body{font:16px system-ui;max-width:480px;margin:40px auto;padding:0 16px}input,button{font:inherit;padding:10px;margin:6px 0;width:100%;box-sizing:border-box}li{padding:8px;cursor:pointer}</style><h1>Shibo 配网</h1><p id="status">正在读取状态…</p><button onclick="scan()">扫描 Wi-Fi</button><ul id="networks"></ul><form id="form"><input name="ssid" placeholder="Wi-Fi 名称" required maxlength="32"><input name="password" type="password" placeholder="密码" maxlength="64"><button>保存并连接</button></form><script>const $=s=>document.querySelector(s);async function status(){let j=await(await fetch('/api/status')).json();$('#status').textContent=j.wifi.connected?'已连接 '+j.wifi.ssid+' · '+j.wifi.ip:j.wifi.ap?'配置热点已开启':'正在连接 '+j.wifi.ssid}async function scan(){await fetch('/api/wifi/scan',{method:'POST'});let t=setInterval(async()=>{let j=await(await fetch('/api/wifi/networks')).json();if(j.scanning)return;clearInterval(t);$('#networks').replaceChildren(...j.networks.map(n=>{let li=document.createElement('li');li.textContent=n.ssid+' ('+n.rssi+' dBm)';li.onclick=()=>$('#form').elements.ssid.value=n.ssid;return li}))},500)}$('#form').onsubmit=async e=>{e.preventDefault();let f=e.target;let r=await fetch('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:f.elements.ssid.value,password:f.elements.password.value})});$('#status').textContent=r.ok?'已提交，正在连接':'连接请求失败';setTimeout(status,3000)};status()</script></html>)html";

        /**
         * @brief 只允许本项目 GitHub Pages 跨源访问
         */
        void cors(httpd_req_t *req)
        {
            char origin[80] = {};
            if(httpd_req_get_hdr_value_str(req, "Origin", origin,
                sizeof(origin)) == ESP_OK && strcmp(origin, PAGE_ORIGIN) == 0)
            {
                httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",
                    PAGE_ORIGIN);
                httpd_resp_set_hdr(req, "Vary", "Origin");
            }
        }

        /**
         * @brief 发送 JSON 并释放临时对象
         */
        esp_err_t send_json(httpd_req_t *req, cJSON *json)
        {
            cors(req);
            httpd_resp_set_type(req, "application/json");
            char *body = cJSON_PrintUnformatted(json);
            const esp_err_t result = body ? httpd_resp_sendstr(req, body) :
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                    "JSON failed");
            cJSON_free(body);
            cJSON_Delete(json);
            return result;
        }

        /**
         * @brief 接收长度有限的 JSON 请求
         */
        cJSON *read_json(httpd_req_t *req)
        {
            if(req->content_len <= 0 || req->content_len > 256)
            {
                return nullptr;
            }
            char body[257] = {};
            size_t received = 0;
            while(received < static_cast<size_t>(req->content_len))
            {
                const int count = httpd_req_recv(req, body + received,
                    req->content_len - received);
                if(count <= 0){return nullptr;}
                received += count;
            }
            return cJSON_Parse(body);
        }

        /**
         * @brief 获取当前 Wi-Fi 和手柄状态
         */
        esp_err_t status(httpd_req_t *req)
        {
            const wifi::state link = wifi::get_state();
            gamepad::state pad;
            gamepad::get_state(pad);
            const gamepad::discovery search = gamepad::get_discovery();
            cJSON *root = cJSON_CreateObject();
            cJSON *wifi_json = cJSON_AddObjectToObject(root, "wifi");
            cJSON_AddStringToObject(wifi_json, "ssid", link.ssid);
            cJSON_AddStringToObject(wifi_json, "ip", link.ip);
            cJSON_AddBoolToObject(wifi_json, "connected", link.connected);
            cJSON_AddBoolToObject(wifi_json, "ap", link.ap_active);
            cJSON *pad_json = cJSON_AddObjectToObject(root, "gamepad");
            cJSON_AddBoolToObject(pad_json, "connected", pad.connected);
            cJSON_AddStringToObject(pad_json, "target", search.target);
            return send_json(req, root);
        }

        /**
         * @brief 返回最近一次 Wi-Fi 扫描结果
         */
        esp_err_t wifi_networks(httpd_req_t *req)
        {
            wifi::network found[16];
            const uint8_t count = wifi::get_networks(found, 16);
            cJSON *root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "scanning",
                wifi::get_state().scanning);
            cJSON *list = cJSON_AddArrayToObject(root, "networks");
            for(uint8_t i = 0; i < count; i++)
            {
                cJSON *item = cJSON_CreateObject();
                cJSON_AddStringToObject(item, "ssid", found[i].ssid);
                cJSON_AddNumberToObject(item, "rssi", found[i].rssi);
                cJSON_AddBoolToObject(item, "secured", found[i].secured);
                cJSON_AddItemToArray(list, item);
            }
            return send_json(req, root);
        }

        /**
         * @brief 开始异步 Wi-Fi 扫描
         */
        esp_err_t wifi_scan(httpd_req_t *req)
        {
            if(!wifi::scan())
            {
                cors(req);
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "Wi-Fi busy");
            }
            return send_json(req, cJSON_CreateObject());
        }

        /**
         * @brief 保存并切换 Wi-Fi 目标
         */
        esp_err_t wifi_connect(httpd_req_t *req)
        {
            cJSON *input = read_json(req);
            const cJSON *ssid = input ? cJSON_GetObjectItemCaseSensitive(input,
                "ssid") : nullptr;
            const cJSON *password = input ? cJSON_GetObjectItemCaseSensitive(input,
                "password") : nullptr;
            const bool valid = cJSON_IsString(ssid) &&
                cJSON_IsString(password) &&
                wifi::connect(ssid->valuestring, password->valuestring);
            cJSON_Delete(input);
            if(!valid)
            {
                cors(req);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                    "Invalid Wi-Fi target");
            }
            return send_json(req, cJSON_CreateObject());
        }

        /**
         * @brief 返回最近一次 Xbox 扫描结果
         */
        esp_err_t gamepad_devices(httpd_req_t *req)
        {
            gamepad::device found[8];
            const uint8_t count = gamepad::get_devices(found, 8);
            cJSON *root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "scanning",
                gamepad::get_discovery().scanning);
            cJSON *list = cJSON_AddArrayToObject(root, "devices");
            for(uint8_t i = 0; i < count; i++)
            {
                cJSON *item = cJSON_CreateObject();
                cJSON_AddNumberToObject(item, "index", i);
                cJSON_AddStringToObject(item, "address", found[i].address);
                cJSON_AddNumberToObject(item, "rssi", found[i].rssi);
                cJSON_AddItemToArray(list, item);
            }
            return send_json(req, root);
        }

        /**
         * @brief 开始 Xbox 扫描
         */
        esp_err_t gamepad_scan(httpd_req_t *req)
        {
            if(!gamepad::scan_devices())
            {
                cors(req);
                httpd_resp_set_status(req, "503 Service Unavailable");
                return httpd_resp_sendstr(req, "Gamepad unavailable");
            }
            return send_json(req, cJSON_CreateObject());
        }

        /**
         * @brief 切换到扫描结果中的手柄
         */
        esp_err_t gamepad_target(httpd_req_t *req)
        {
            cJSON *input = read_json(req);
            const cJSON *index = input ? cJSON_GetObjectItemCaseSensitive(input,
                "index") : nullptr;
            const bool valid = cJSON_IsNumber(index) &&
                index->valuedouble >= 0 && index->valuedouble <= 7 &&
                index->valuedouble == index->valueint &&
                gamepad::select_device(static_cast<uint8_t>(index->valueint));
            cJSON_Delete(input);
            if(!valid)
            {
                cors(req);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                    "Invalid gamepad");
            }
            return send_json(req, cJSON_CreateObject());
        }

        /**
         * @brief 处理浏览器 JSON 请求的跨源预检
         */
        esp_err_t options(httpd_req_t *req)
        {
            cors(req);
            char origin[80] = {};
            if(httpd_req_get_hdr_value_str(req, "Origin", origin,
                sizeof(origin)) == ESP_OK && strcmp(origin, PAGE_ORIGIN) == 0)
            {
                httpd_resp_set_hdr(req,
                    "Access-Control-Allow-Private-Network", "true");
            }
            httpd_resp_set_hdr(req, "Access-Control-Allow-Methods",
                "GET, POST, OPTIONS");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Headers",
                "Content-Type");
            return httpd_resp_send(req, nullptr, 0);
        }

        /**
         * @brief 提供没有互联网时使用的本地配网页面
         */
        esp_err_t portal(httpd_req_t *req)
        {
            httpd_resp_set_type(req, "text/html; charset=utf-8");
            return httpd_resp_sendstr(req, PORTAL);
        }
    }

    /**
     * @brief 启动配置页面和 JSON API
     *
     * @return true HTTP 服务已启动
     */
    bool init()
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.core_id = 0;
        config.task_priority = 2;
        config.uri_match_fn = httpd_uri_match_wildcard;
        config.max_uri_handlers = 9;
        httpd_handle_t server = nullptr;
        if(httpd_start(&server, &config) != ESP_OK){return false;}
        const httpd_uri_t routes[] =
        {
            {"/", HTTP_GET, portal, nullptr},
            {"/api/status", HTTP_GET, status, nullptr},
            {"/api/wifi/networks", HTTP_GET, wifi_networks, nullptr},
            {"/api/wifi/scan", HTTP_POST, wifi_scan, nullptr},
            {"/api/wifi/connect", HTTP_POST, wifi_connect, nullptr},
            {"/api/gamepad/devices", HTTP_GET, gamepad_devices, nullptr},
            {"/api/gamepad/scan", HTTP_POST, gamepad_scan, nullptr},
            {"/api/gamepad/target", HTTP_POST, gamepad_target, nullptr},
            {"/api/*", HTTP_OPTIONS, options, nullptr}
        };
        for(const httpd_uri_t &route : routes)
        {
            if(httpd_register_uri_handler(server, &route) != ESP_OK)
            {
                httpd_stop(server);
                return false;
            }
        }
        return true;
    }
}
