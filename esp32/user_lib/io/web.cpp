#include "web.h"

#include "hw/gamepad.h"
#include "hw/wifi.h"
#include "cJSON.h"
#include "esp_http_server.h"

namespace web
{
    namespace
    {
        constexpr char PAGE[] = R"html(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Shibo Console</title>
<style>
:root{font:16px/1.5 system-ui,-apple-system,"Segoe UI",sans-serif;color:#173047;background:#eef3f5}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:radial-gradient(circle at 85% 5%,#d4ece9,transparent 35%),#eef3f5}
a{color:inherit;text-decoration:none}button,input{font:inherit}button{cursor:pointer}
.shell{width:min(100% - 32px,800px);margin:auto}.top{background:#142f40;color:#fff;padding:28px 0 24px}
.brand{display:flex;align-items:center;gap:13px}.logo{width:38px;height:38px;border-radius:12px;background:linear-gradient(135deg,#42dfc7,#1b9ea3);box-shadow:0 0 0 5px #ffffff14}
.brand strong{display:block;font-size:20px;letter-spacing:.02em}.brand span{font-size:12px;color:#bed0d9}
.topline{display:flex;justify-content:space-between;align-items:center;gap:16px}.pill{font-size:12px;padding:6px 11px;border-radius:30px;background:#ffffff16;color:#dcebef;white-space:nowrap}
nav{display:flex;gap:8px;margin-top:26px}nav a{padding:9px 16px;border-radius:9px;color:#bfd0d8;font-size:14px}nav a.active{background:#ffffff1e;color:#fff}
main{padding:30px 0 50px}.eyebrow{color:#118c86;font-size:12px;font-weight:750;letter-spacing:.16em;text-transform:uppercase}
h1{font-size:clamp(26px,5vw,35px);line-height:1.18;margin:7px 0 10px;letter-spacing:-.03em}h2{font-size:18px;margin:0 0 5px}
.lead,.muted{color:#667a87}.lead{margin:0 0 24px}.card{background:#fff;border:1px solid #dce6e9;border-radius:16px;box-shadow:0 8px 25px #1730470a;padding:22px;margin-top:16px}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:16px}.tile{display:block;min-height:160px;transition:transform .15s,box-shadow .15s}.tile:hover{transform:translateY(-2px);box-shadow:0 12px 28px #17304718}
.tile .icon{width:42px;height:42px;display:grid;place-items:center;border-radius:11px;background:#def4ef;color:#087c73;font-weight:800;margin-bottom:18px}
.tile p{margin:0;color:#667a87;font-size:14px}.tile b{display:block;font-size:19px;margin-bottom:4px}
.status{display:flex;align-items:flex-start;gap:12px}.dot{flex:none;width:10px;height:10px;border-radius:50%;margin-top:7px;background:#a0adb4;box-shadow:0 0 0 4px #a0adb41e}.dot.ok{background:#14a98f;box-shadow:0 0 0 4px #14a98f24}.status p{margin:2px 0 0;color:#667a87;font-size:14px;overflow-wrap:anywhere}
.row{display:flex;align-items:center;justify-content:space-between;gap:16px}.stack{display:grid;gap:12px}label{display:block;font-size:13px;font-weight:650;margin-bottom:6px}
input{width:100%;border:1px solid #cbd9dd;background:#fbfdfd;border-radius:10px;padding:11px 12px;color:#173047;outline:none}input:focus{border-color:#0f9b91;box-shadow:0 0 0 3px #0f9b9120}
.button{border:0;border-radius:10px;padding:10px 15px;background:#097e78;color:#fff;font-weight:650;white-space:nowrap}.button:hover{background:#066e69}.button:disabled{opacity:.48;cursor:default}
.button.secondary{background:#e8f2f1;color:#0d6c69}.button.secondary:hover{background:#d7ebe8}.section-title{margin:30px 0 4px}.list{display:grid;gap:9px;margin-top:15px}.item{display:flex;align-items:center;justify-content:space-between;gap:16px;border:1px solid #dce6e9;border-radius:12px;padding:13px 15px;background:#fff}
.item strong{display:block;font-size:14px;overflow-wrap:anywhere}.item small{color:#6b7e89;overflow-wrap:anywhere}.badge{display:inline-block;padding:2px 7px;border-radius:6px;background:#def4ef;color:#087c73;font-size:11px;font-weight:750;margin-left:6px}
.note{font-size:13px;color:#667a87}.message{min-height:23px;font-size:13px;color:#0a746e}.message.error{color:#b4493b}footer{padding:24px 0 34px;color:#83939b;font-size:12px}
[hidden]{display:none!important}@media(max-width:600px){.grid{grid-template-columns:1fr}.tile{min-height:130px}.topline{align-items:flex-start}.pill{margin-top:4px}.card{padding:18px}.item{align-items:flex-start}.item .button{font-size:13px;padding:8px 10px}}
</style>
</head>
<body>
<header class="top"><div class="shell"><div class="topline"><div class="brand"><div class="logo"></div><div><strong>Shibo Console</strong><span>机器人设备设置</span></div></div><span id="header-status" class="pill">正在读取状态</span></div><nav><a href="/" data-route="/">概览</a><a href="/wifi" data-route="/wifi">Wi-Fi</a><a href="/bluetooth" data-route="/bluetooth">蓝牙手柄</a></nav></div></header>
<div class="shell">
<main id="home" hidden><span class="eyebrow">DEVICE SETTINGS</span><h1>连接与配置</h1><p class="lead">在这里管理机器人使用的网络和手柄。</p><div class="grid"><a class="card tile" href="/wifi"><div class="icon">Wi</div><b>Wi-Fi 设置</b><p>扫描网络、保存连接目标、查看当前地址。</p></a><a class="card tile" href="/bluetooth"><div class="icon">BT</div><b>蓝牙手柄</b><p>查找设备并选择要连接的 Xbox 手柄。</p></a></div></main>
<main id="wifi" hidden><span class="eyebrow">NETWORK</span><h1>Wi-Fi 设置</h1><p class="lead">扫描附近网络，然后选择 SSID 并填写密码。</p><div class="card status"><span id="wifi-dot" class="dot"></span><div><b id="wifi-state">读取中</b><p id="wifi-detail">正在获取网络状态…</p></div></div><div class="row section-title"><h2>附近网络</h2><button class="button secondary" id="wifi-scan">扫描网络</button></div><div id="wifi-list" class="list"></div><form id="wifi-form" class="card stack"><h2>连接目标</h2><div><label for="ssid">网络名称</label><input id="ssid" maxlength="32" required placeholder="SSID" autocomplete="off"></div><div><label for="password">密码</label><input id="password" maxlength="64" type="password" placeholder="输入 Wi-Fi 密码" autocomplete="new-password"></div><button class="button" type="submit">保存并连接</button><div id="wifi-message" class="message" role="status"></div></form></main>
<main id="bluetooth" hidden><span class="eyebrow">CONTROLLER</span><h1>蓝牙手柄</h1><p class="lead">先让手柄进入蓝牙配对模式，再扫描并选择设备。</p><div class="card status"><span id="pad-dot" class="dot"></span><div><b id="pad-state">读取中</b><p id="pad-detail">正在获取连接状态…</p></div></div><div class="row section-title"><h2>附近设备</h2><button class="button secondary" id="pad-scan">扫描 5 秒</button></div><p class="note">扫描结果会标明 Xbox 广播和可连接状态。选择后可在上方查看连接阶段与错误码。</p><div id="pad-list" class="list"></div><div id="pad-message" class="message" role="status"></div></main>
<footer>Shibo Robot · 本地设备页面</footer></div>
<script>
const $=id=>document.getElementById(id);
const route=location.pathname==='/wifi'?'wifi':location.pathname==='/bluetooth'?'bluetooth':'home';
$(route).hidden=false;document.querySelector(`nav a[data-route="${location.pathname}"]`)?.classList.add('active');
const phases=['搜索目标','正在连接','正在配对','查找 HID 服务','订阅输入报告','已收到手柄输入','连接失败'];
async function api(path,options){const r=await fetch(path,options);if(!r.ok)throw Error('HTTP '+r.status);return r.json()}
function message(id,text,error=false){$(id).textContent=text;$(id).classList.toggle('error',error)}
async function refresh(){
  const d=await api('/api/status');const w=d.wifi,g=d.gamepad;
  $('header-status').textContent=g.connected?'手柄已连接':w.connected?'网络已连接':w.ap?'配置热点已开启':'设备在线';
  $('wifi-dot').classList.toggle('ok',w.connected);$('wifi-state').textContent=w.connected?'已连接 '+w.ssid:w.ap?'配置热点已开启':'正在连接网络';
  $('wifi-detail').textContent=w.connected?'设备地址 '+w.ip:w.ssid?'目标网络 '+w.ssid:'尚未设置目标网络';
  $('pad-dot').classList.toggle('ok',g.connected);$('pad-state').textContent=g.connected?'手柄已连接':g.phase===6?`连接失败（${phases[g.failed_at]||'未知阶段'}）`:phases[g.phase]||'等待手柄';
  $('pad-detail').textContent='目标：'+(g.target||'自动搜索 Xbox')+(g.error?' · 错误码 '+g.error:'');
  return d;
}
function item(title,detail,action,disabled=false,badge=''){
  const row=document.createElement('div');row.className='item';const info=document.createElement('div');
  const name=document.createElement('strong');name.textContent=title;
  if(badge){const mark=document.createElement('span');mark.className='badge';mark.textContent=badge;name.append(mark)}
  const meta=document.createElement('small');meta.textContent=detail;info.append(name,meta);
  const button=document.createElement('button');button.className='button secondary';button.textContent=action.label;button.disabled=disabled;button.onclick=action.run;
  row.append(info,button);return row;
}
$('wifi-scan').onclick=async()=>{
  const button=$('wifi-scan');button.disabled=true;button.textContent='扫描中…';message('wifi-message','');
  try{await api('/api/wifi/scan',{method:'POST'});for(let i=0;i<25;i++){
    await new Promise(r=>setTimeout(r,400));const d=await api('/api/wifi/networks');if(d.scanning)continue;
    $('wifi-list').replaceChildren(...d.networks.map(n=>item(n.ssid||'隐藏网络',`${n.rssi} dBm · ${n.secured?'需要密码':'开放网络'}`,{label:'选择',run:()=>{$('ssid').value=n.ssid;$('ssid').focus()}})));
    if(!d.networks.length)message('wifi-message','未找到附近网络');return;
  }throw Error('扫描超时')}catch(e){message('wifi-message',e.message,true)}finally{button.disabled=false;button.textContent='扫描网络'}
};
$('wifi-form').onsubmit=async e=>{
  e.preventDefault();try{await api('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:$('ssid').value,password:$('password').value})});message('wifi-message','已保存连接目标。网络切换后请重新打开设备地址。');setTimeout(()=>refresh().catch(()=>{}),3000)}catch(err){message('wifi-message',err.message,true)}
};
$('pad-scan').onclick=async()=>{
  const button=$('pad-scan');button.disabled=true;button.textContent='扫描中…';message('pad-message','');
  try{await api('/api/gamepad/scan',{method:'POST'});for(let i=0;i<24;i++){
    await new Promise(r=>setTimeout(r,500));const d=await api('/api/gamepad/devices');if(d.scanning)continue;
    if(d.error)throw Error('扫描失败，错误码 '+d.error);
    $('pad-list').replaceChildren(...d.devices.map(n=>item(n.name||'未命名设备',`${n.address} · ${n.rssi} dBm`,{label:n.connectable?'选择设备':'不可连接',run:()=>selectPad(n.index)},!n.connectable,n.xbox?'Xbox':'')));
    if(!d.devices.length)message('pad-message','未发现 BLE 设备，请确认手柄已进入配对模式');return;
  }throw Error('扫描超时')}catch(e){message('pad-message',e.message,true)}finally{button.disabled=false;button.textContent='扫描 5 秒'}
};
async function selectPad(index){
  try{await api('/api/gamepad/target',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({index})});message('pad-message','目标已保存，正在等待手柄广播和连接…');
    for(let i=0;i<25;i++){await new Promise(r=>setTimeout(r,500));const d=await refresh();if(d.gamepad.connected){message('pad-message','手柄输入已连接');return}if(d.gamepad.phase===6){message('pad-message','连接失败于'+phases[d.gamepad.failed_at]+'，错误码 '+d.gamepad.error,true);return}}
    message('pad-message','仍在搜索或连接，请保持手柄配对模式并查看上方阶段');
  }catch(e){message('pad-message',e.message,true)}
}
refresh().catch(()=>{$('header-status').textContent='设备状态不可用'});setInterval(()=>refresh().catch(()=>{}),3000);
</script>
</body>
</html>)html";

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
            const bool pad_ready = gamepad::get_state(pad);
            const gamepad::discovery search = gamepad::get_discovery();
            cJSON *root = cJSON_CreateObject();
            cJSON *wifi_json = cJSON_AddObjectToObject(root, "wifi");
            cJSON_AddStringToObject(wifi_json, "ssid", link.ssid);
            cJSON_AddStringToObject(wifi_json, "ip", link.ip);
            cJSON_AddBoolToObject(wifi_json, "connected", link.connected);
            cJSON_AddBoolToObject(wifi_json, "ap", link.ap_active);
            cJSON *pad_json = cJSON_AddObjectToObject(root, "gamepad");
            cJSON_AddBoolToObject(pad_json, "connected", pad_ready);
            cJSON_AddStringToObject(pad_json, "target", search.target);
            cJSON_AddNumberToObject(pad_json, "phase",
                static_cast<uint8_t>(search.phase));
            cJSON_AddNumberToObject(pad_json, "failed_at",
                static_cast<uint8_t>(search.failed_at));
            cJSON_AddNumberToObject(pad_json, "error", search.link_error);
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
            const gamepad::discovery search = gamepad::get_discovery();
            cJSON *root = cJSON_CreateObject();
            cJSON_AddBoolToObject(root, "scanning", search.scanning);
            cJSON_AddNumberToObject(root, "error", search.scan_error);
            cJSON *list = cJSON_AddArrayToObject(root, "devices");
            for(uint8_t i = 0; i < count; i++)
            {
                cJSON *item = cJSON_CreateObject();
                cJSON_AddNumberToObject(item, "index", i);
                cJSON_AddStringToObject(item, "address", found[i].address);
                cJSON_AddStringToObject(item, "name", found[i].name);
                cJSON_AddNumberToObject(item, "rssi", found[i].rssi);
                cJSON_AddBoolToObject(item, "xbox", found[i].xbox);
                cJSON_AddBoolToObject(item, "connectable",
                    found[i].connectable);
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
            return httpd_resp_sendstr(req, PAGE);
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
