#include "gamepad.h"

#include "config.h"
#include "sys_time.h"
#include "esp_hidh.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" void ble_store_config_init(void);

namespace gamepad
{
    namespace
    {
        portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
        control::remote_input latest;
        bool linked = false;
        device devices[12];
        size_t device_count = 0;
        uint8_t own_address_type = 0;
        SemaphoreHandle_t scan_done = nullptr;
        SemaphoreHandle_t synced = nullptr;
        ble_addr_t candidate;
        bool candidate_found = false;

        /**
         * @brief NimBLE 同步完成后取得本机地址类型并通知连接任务
         */
        void on_sync()
        {
            if(ble_hs_id_infer_auto(0, &own_address_type) == 0){xSemaphoreGive(synced);}
        }

        /**
         * @brief 处理 Xbox 广播并将设备列表提供给配置网页
         *
         * @param[in] event BLE 扫描事件
         *
         * @return 0，不中断 GAP 事件处理
         */
        int scan_event(ble_gap_event *event, void *)
        {
            if(event->type == BLE_GAP_EVENT_DISC_COMPLETE)
            {
                xSemaphoreGive(scan_done);

                return 0;
            }
            if(event->type != BLE_GAP_EVENT_DISC){return 0;}

            ble_hs_adv_fields fields{};
            if(ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0)
            {
                return 0;
            }

            device found;
            const uint8_t *address = event->disc.addr.val;
            snprintf(found.address, sizeof(found.address), "%02x:%02x:%02x:%02x:%02x:%02x",
                address[5], address[4], address[3], address[2], address[1], address[0]);
            if(fields.name != nullptr)
            {
                const size_t length = std::min<size_t>(fields.name_len, sizeof(found.name) - 1);
                memcpy(found.name, fields.name, length);
            }

            found.rssi = event->disc.rssi;
            const bool xbox = strstr(found.name, "Xbox") != nullptr;
            const char *configured = config::get().gamepad_address;
            const bool selected = configured[0] != '\0' ? strcasecmp(configured, found.address) == 0 : xbox;
            if(xbox || selected)
            {
                portENTER_CRITICAL(&lock);
                size_t index = 0;
                while(index < device_count && strcmp(devices[index].address, found.address) != 0){index++;}
                if(index < 12)
                {
                    devices[index] = found;
                    if(index == device_count){device_count++;}
                }
                portEXIT_CRITICAL(&lock);
            }

            if(selected && event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND)
            {
                candidate = event->disc.addr;
                candidate_found = true;
            }

            return 0;
        }

        /**
         * @brief 处理 HID 连接和输入事件，仅在收到有效报告时更新时间戳
         *
         * @param[in] event HID 事件编号
         * @param[in] data HID 事件数据
         */
        void hid_event(void *, esp_event_base_t, int32_t event, void *data)
        {
            const esp_hidh_event_data_t *value = static_cast<esp_hidh_event_data_t *>(data);
            if(event == ESP_HIDH_OPEN_EVENT)
            {
                portENTER_CRITICAL(&lock);
                linked = value->open.status == ESP_OK;
                latest = {};
                latest.stream_id = static_cast<uint32_t>(sys_time::get_us_tick());
                portEXIT_CRITICAL(&lock);
            }
            else if(event == ESP_HIDH_INPUT_EVENT && value->input.report_id == 1)
            {
                portENTER_CRITICAL(&lock);
                control::remote_input next = latest;
                if(parse_report(value->input.data, value->input.length, next))
                {
                    next.timestamp_us = static_cast<uint32_t>(sys_time::get_us_tick());
                    latest = next;
                }
                portEXIT_CRITICAL(&lock);
            }
            else if(event == ESP_HIDH_CLOSE_EVENT)
            {
                portENTER_CRITICAL(&lock);
                linked = false;
                latest.valid = false;
                portEXIT_CRITICAL(&lock);
                // IDF 6 的 HID 事件后处理负责释放设备。
            }
        }

        /**
         * @brief 运行 NimBLE 协议栈任务并在退出时释放任务资源
         */
        void host_task(void *)
        {
            nimble_port_run();
            nimble_port_freertos_deinit();
        }

        /**
         * @brief 在低优先级任务中扫描、连接手柄并发送连接振动提示
         */
        void task(void *)
        {
            xSemaphoreTake(synced, portMAX_DELAY);
            while(true)
            {
                if(!connected())
                {
                    candidate_found = false;
                    ble_gap_disc_params params{};
                    params.itvl = 80;
                    params.window = 40;
                    params.filter_duplicates = 0;
                    if(ble_gap_disc(own_address_type, 3000, &params, scan_event, nullptr) == 0)
                    {
                        xSemaphoreTake(scan_done, portMAX_DELAY);
                        if(candidate_found)
                        {
                            esp_hidh_dev_t *device = esp_hidh_dev_open(
                                candidate.val, ESP_HID_TRANSPORT_BLE, candidate.type);
                            size_t count = 0;
                            esp_hid_report_item_t *reports = nullptr;
                            if(device != nullptr &&
                               esp_hidh_dev_reports_get(device, &count, &reports) == ESP_OK)
                            {
                                // Xbox 连接成功后振动 1 秒，由手柄自身计时停止。
                                uint8_t vibration[8] = {0x0F, 0, 0, 50, 50, 100, 0, 0};
                                for(size_t i = 0; i < count; i++)
                                {
                                    if(reports[i].report_type == ESP_HID_REPORT_TYPE_OUTPUT &&
                                       reports[i].value_len == sizeof(vibration))
                                    {
                                        esp_hidh_dev_output_set(device, reports[i].map_index,
                                            reports[i].report_id, vibration, sizeof(vibration));
                                        break;
                                    }
                                }
                                free(reports);
                            }
                        }
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }

    /**
     * @brief 初始化 NimBLE HID 主机并启动手柄连接任务
     *
     * @return true 手柄连接任务已启动
     * @return false 同步对象、协议栈或任务初始化失败
     */
    bool init()
    {
        scan_done = xSemaphoreCreateBinary();
        synced = xSemaphoreCreateBinary();
        if(scan_done == nullptr || synced == nullptr || nimble_port_init() != ESP_OK){return false;}

        ble_hs_cfg.sync_cb = on_sync;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
        ble_store_config_init();

        esp_hidh_config_t hid{};
        hid.callback = hid_event;
        hid.event_stack_size = 4096;
        if(esp_hidh_init(&hid) != ESP_OK){return false;}

        // Xbox 支持无输入输出配对，不要求键盘输入 PIN。
        ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm = 0;
        nimble_port_freertos_init(host_task);

        return xTaskCreatePinnedToCore(task, "gamepad", 6144, nullptr, 1, nullptr, 0) == pdPASS;
    }

    /**
     * @brief 获取最新手柄输入快照
     *
     * @param[out] out 手柄输入快照
     *
     * @return true 已收到有效输入报告
     * @return false 尚无有效报告或手柄已断开
     */
    bool get(control::remote_input &out)
    {
        portENTER_CRITICAL(&lock);
        out = latest;
        portEXIT_CRITICAL(&lock);
        return out.valid;
    }

    /**
     * @brief 查询手柄连接状态
     *
     * @return true HID 连接已建立
     * @return false HID 未连接
     */
    bool connected()
    {
        portENTER_CRITICAL(&lock);
        const bool result = linked;
        portEXIT_CRITICAL(&lock);
        return result;
    }

    /**
     * @brief 复制扫描到的手柄设备列表
     *
     * @param[out] out 设备数组
     * @param[in] capacity 数组可容纳的设备数量
     *
     * @return 实际复制的设备数量
     */
    size_t get_devices(device *out, size_t capacity)
    {
        portENTER_CRITICAL(&lock);
        const size_t count = std::min(device_count, capacity);
        memcpy(out, devices, count * sizeof(device));
        portEXIT_CRITICAL(&lock);
        return count;
    }
}
