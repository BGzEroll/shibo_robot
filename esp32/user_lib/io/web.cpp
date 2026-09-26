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
        constexpr char HOME[] = R"html(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Shibo Console</title><style>body{font:16px system-ui;max-width:560px;margin:40px auto;padding:0 20px}a{display:block;padding:18px;margin:16px 0;border:1px solid #bbb;border-radius:10px;color:inherit;text-decoration:none}</style><h1>Shibo Console</h1><a href="/wifi">Wi-Fi 设置</a><a href="/bluetooth">蓝牙设置</a></html>)html";
        constexpr char WIFI_PAGE[] = R"html(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Wi-Fi 设置</title><style>body{font:16px system-ui;max-width:560px;margin:24px auto;padding:0 20px}button,input{font:inherit;padding:10px;margin:5px 0}input{width:95%}li{padding:8px 0}li button{margin-left:10px}</style><a href="/">← 首页</a><h1>Wi-Fi 设置</h1><p id="state">读取中…</p><button id="scan">扫描网络</button><ul id="list"></ul><form id="form"><input name="ssid" placeholder="SSID" required maxlength="32"><input name="password" type="password" placeholder="密码" maxlength="64"><button>保存并连接</button></form><p id="message"></p><script>
const $=id=>document.getElementById(id);
async function api(path,options){const r=await fetch(path,options);if(!r.ok)throw Error('HTTP '+r.status);return r.json()}
async function status(){const d=await api('/api/status');$('state').textContent=d.wifi.connected?'已连接 '+d.wifi.ssid+' · '+d.wifi.ip:d.wifi.ap?'热点已开启 · '+(d.wifi.ssid||'未设置目标网络'):'正在连接 '+d.wifi.ssid}
async function scan(){try{await api('/api/wifi/scan',{method:'POST'});for(let i=0;i<25;i++){await new Promise(r=>setTimeout(r,400));const d=await api('/api/wifi/networks');if(d.scanning)continue;$('list').replaceChildren(...d.networks.map(n=>{const li=document.createElement('li'),b=document.createElement('button');li.textContent=(n.ssid||'隐藏网络')+' · '+n.rssi+' dBm';b.textContent='选择';b.onclick=()=>$('form').elements.ssid.value=n.ssid;li.append(b);return li}));return}throw Error('扫描超时')}catch(e){$('message').textContent=e.message}}
$('scan').onclick=scan;$('form').onsubmit=async e=>{e.preventDefault();try{const f=e.target;await api('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:f.elements.ssid.value,password:f.elements.password.value})});$('message').textContent='连接请求已提交';setTimeout(()=>status().catch(e=>$('message').textContent=e.message),3000)}catch(e){$('message').textContent=e.message}};status().catch(e=>$('message').textContent=e.message);
</script></html>)html";
        constexpr char BLUETOOTH_PAGE[] = R"html(<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>蓝牙设置</title><style>body{font:16px system-ui;max-width:560px;margin:24px auto;padding:0 20px}button{font:inherit;padding:10px;margin:5px}li{padding:8px 0}</style><a href="/">← 首页</a><h1>蓝牙手柄</h1><p id="state">读取中…</p><p>让手柄进入蓝牙配对模式后扫描。</p><button id="scan">扫描 5 秒</button><ul id="list"></ul><p id="message"></p><script>
const $=id=>document.getElementById(id);
async function api(path,options){const r=await fetch(path,options);if(!r.ok)throw Error('HTTP '+r.status);return r.json()}
async function status(){const d=await api('/api/status');$('state').textContent=(d.gamepad.connected?'已连接':'未连接')+' · 目标 '+(d.gamepad.target||'自动搜索 Xbox')}
$('scan').onclick=async()=>{try{await api('/api/gamepad/scan',{method:'POST'});for(let i=0;i<25;i++){await new Promise(r=>setTimeout(r,400));const d=await api('/api/gamepad/devices');if(d.scanning)continue;if(d.error)throw Error('BLE 扫描失败：'+d.error);$('list').replaceChildren(...d.devices.map(n=>{const li=document.createElement('li'),b=document.createElement('button');li.textContent=(n.name||'未命名设备')+' · '+n.address+' · '+n.rssi+' dBm'+(n.xbox?' · Xbox':'');b.textContent='连接';b.onclick=async()=>{try{await api('/api/gamepad/target',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index:n.index})});$('message').textContent='已选择目标，等待连接';setTimeout(()=>status().catch(()=>{}),2000)}catch(e){$('message').textContent=e.message}};li.append(b);return li}));if(!d.devices.length)$('message').textContent='未发现 BLE 广播，请检查手柄配对模式';return}throw Error('扫描超时')}catch(e){$('message').textContent=e.message}};status().catch(e=>$('message').textContent=e.message);
</script></html>)html";

        /**
         * @brief 发送 JSON 并释放临时对象
         */
        esp_err_t send_json(httpd_req_t *req, cJSON *json)
        {
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
            gamepad::device found[24];
            const uint8_t count = gamepad::get_devices(found, 24);
            cJSON *root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "scanning",
                gamepad::get_discovery().scanning);
            cJSON_AddNumberToObject(root, "error",
                gamepad::get_discovery().scan_error);
            cJSON *list = cJSON_AddArrayToObject(root, "devices");
            for(uint8_t i = 0; i < count; i++)
            {
                cJSON *item = cJSON_CreateObject();
                cJSON_AddNumberToObject(item, "index", i);
                cJSON_AddStringToObject(item, "address", found[i].address);
                cJSON_AddStringToObject(item, "name", found[i].name);
                cJSON_AddNumberToObject(item, "rssi", found[i].rssi);
                cJSON_AddBoolToObject(item, "xbox", found[i].xbox);
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
                index->valuedouble >= 0 && index->valuedouble <= 23 &&
                index->valuedouble == index->valueint &&
                gamepad::select_device(static_cast<uint8_t>(index->valueint));
            cJSON_Delete(input);
            if(!valid)
            {
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                    "Invalid gamepad");
            }
            return send_json(req, cJSON_CreateObject());
        }

        /**
         * @brief 提供固件内置页面
         */
        esp_err_t page(httpd_req_t *req)
        {
            httpd_resp_set_type(req, "text/html; charset=utf-8");
            const char *html = strcmp(req->uri, "/wifi") == 0 ? WIFI_PAGE :
                strcmp(req->uri, "/bluetooth") == 0 ? BLUETOOTH_PAGE : HOME;
            return httpd_resp_sendstr(req, html);
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
        config.stack_size = 6144;
        config.uri_match_fn = httpd_uri_match_wildcard;
        config.max_uri_handlers = 10;
        httpd_handle_t server = nullptr;
        if(httpd_start(&server, &config) != ESP_OK){return false;}
        const httpd_uri_t routes[] =
        {
            {"/", HTTP_GET, page, nullptr},
            {"/wifi", HTTP_GET, page, nullptr},
            {"/bluetooth", HTTP_GET, page, nullptr},
            {"/api/status", HTTP_GET, status, nullptr},
            {"/api/wifi/networks", HTTP_GET, wifi_networks, nullptr},
            {"/api/wifi/scan", HTTP_POST, wifi_scan, nullptr},
            {"/api/wifi/connect", HTTP_POST, wifi_connect, nullptr},
            {"/api/gamepad/devices", HTTP_GET, gamepad_devices, nullptr},
            {"/api/gamepad/scan", HTTP_POST, gamepad_scan, nullptr},
            {"/api/gamepad/target", HTTP_POST, gamepad_target, nullptr}
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
