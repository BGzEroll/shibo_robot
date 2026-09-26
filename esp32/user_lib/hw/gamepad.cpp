#include "gamepad.h"

#include "sys_time.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"

#include <string.h>
#include <stdio.h>
#include <atomic>

extern "C" void ble_store_config_init(void);

namespace gamepad
{
    namespace
    {
        constexpr uint8_t MAX_REPORTS = 8;
        constexpr uint8_t MAX_DEVICES = 8;
        constexpr ble_uuid16_t HID_SERVICE = BLE_UUID16_INIT(0x1812);

        struct report
        {
            uint16_t value = 0;
            uint16_t cccd = 0;
        };

        portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
        state latest_state;
        report reports[MAX_REPORTS];
        uint8_t report_count = 0;
        uint8_t subscribe_index = 0;
        uint16_t connection = UINT16_MAX;
        uint16_t service_start = 0;
        uint16_t service_end = 0;
        uint8_t own_address_type = 0;
        bool connecting = false;
        bool started = false;
        std::atomic<bool> host_ready{false};
        std::atomic<bool> manual_scan{false};
        std::atomic<bool> manual_scan_pending{false};
        bool target_set = false;
        ble_addr_t target = {};
        ble_addr_t discovered_addresses[MAX_DEVICES] = {};
        device discovered[MAX_DEVICES];
        uint8_t discovered_count = 0;
        ble_npl_event command_event;
        uint8_t command = 0;

        /**
         * @brief 将 BLE 地址格式化为显示字符串
         */
        void address_text(const ble_addr_t &address, char *out)
        {
            snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                address.val[5], address.val[4], address.val[3],
                address.val[2], address.val[1], address.val[0]);
        }

        int gap_event(ble_gap_event *event, void *);

        /**
         * @brief 搜索旧项目使用的 Xbox BLE 手柄
         */
        void scan()
        {
            if(connecting || connection != UINT16_MAX || manual_scan ||
                manual_scan_pending){return;}
            ble_gap_disc_params params = {};
            params.passive = 0;
            params.filter_duplicates = 0;
            ble_gap_disc(own_address_type, BLE_HS_FOREVER, &params,
                gap_event, nullptr);
        }

        /**
         * @brief 清空失联手柄的输入
         */
        void disconnect()
        {
            portENTER_CRITICAL(&state_lock);
            const uint32_t next_session = latest_state.session + 1;
            latest_state = state{};
            latest_state.session = next_session;
            portEXIT_CRITICAL(&state_lock);
        }

        /**
         * @brief 将 Xbox 16 字节 HID 报告转换为手柄快照
         *
         * @param[in] data HID 报告
         */
        void receive(const uint8_t *data)
        {
            state next;
            next.timestamp_us = sys_time::get_us_tick();
            next.connected = true;

            const uint8_t main = data[13];
            const uint8_t center = data[14];
            const uint8_t direction = data[12];
            const uint8_t share = data[15];

            if(main & 0x01){next.buttons |= button::A;}
            if(main & 0x02){next.buttons |= button::B;}
            if(main & 0x08){next.buttons |= button::X;}
            if(main & 0x10){next.buttons |= button::Y;}
            if(main & 0x40){next.buttons |= button::LB;}
            if(main & 0x80){next.buttons |= button::RB;}
            if(center & 0x04){next.buttons |= button::SELECT;}
            if(center & 0x08){next.buttons |= button::START;}
            if(center & 0x10){next.buttons |= button::XBOX;}
            if(center & 0x20){next.buttons |= button::LS;}
            if(center & 0x40){next.buttons |= button::RS;}
            if(share & 0x01){next.buttons |= button::SHARE;}
            if(direction == 1 || direction == 2 || direction == 8)
            {
                next.buttons |= button::UP;
            }
            if(direction >= 2 && direction <= 4){next.buttons |= button::RIGHT;}
            if(direction >= 4 && direction <= 6){next.buttons |= button::DOWN;}
            if(direction >= 6 && direction <= 8){next.buttons |= button::LEFT;}

            for(uint8_t i = 0; i < 4; i++)
            {
                const uint16_t raw = static_cast<uint16_t>(
                    data[i * 2] | (static_cast<uint16_t>(data[i * 2 + 1]) << 8));
                next.axes[i] = (static_cast<int32_t>(raw) - 32768) / 32768.0f;
            }
            next.axes[1] = -next.axes[1];
            next.axes[3] = -next.axes[3];
            next.axes[4] = static_cast<float>(
                data[8] | (static_cast<uint16_t>(data[9]) << 8)) / 1023.0f;
            next.axes[5] = static_cast<float>(
                data[10] | (static_cast<uint16_t>(data[11]) << 8)) / 1023.0f;

            portENTER_CRITICAL(&state_lock);
            next.session = latest_state.session;
            latest_state = next;
            portEXIT_CRITICAL(&state_lock);
        }

        /**
         * @brief 依次订阅 HID 输入报告
         */
        void subscribe();

        /**
         * @brief 处理报告通知订阅结果
         */
        int subscribed(uint16_t, const ble_gatt_error *error,
            ble_gatt_attr *, void *)
        {
            if(error->status != 0)
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                return 0;
            }
            subscribe_index++;
            subscribe();
            return 0;
        }

        /**
         * @brief 读取输入报告后开启通知
         */
        int report_read(uint16_t, const ble_gatt_error *,
            ble_gatt_attr *, void *)
        {
            const uint8_t enabled[] = {1, 0};
            if(ble_gattc_write_flat(connection,
                reports[subscribe_index].cccd, enabled, sizeof(enabled),
                subscribed, nullptr) != 0)
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }

        void subscribe()
        {
            while(subscribe_index < report_count)
            {
                if(reports[subscribe_index].cccd != 0){break;}
                subscribe_index++;
            }
            if(subscribe_index < report_count &&
                ble_gattc_read(connection, reports[subscribe_index].value,
                    report_read, nullptr) != 0)
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            }
        }

        /**
         * @brief 查找 HID 报告的通知描述符
         */
        int descriptor_found(uint16_t, const ble_gatt_error *error,
            uint16_t value, const ble_gatt_dsc *descriptor, void *)
        {
            if(error->status == BLE_HS_EDONE)
            {
                bool found = false;
                for(uint8_t i = 0; i < report_count; i++)
                {
                    if(reports[i].cccd != 0){found = true;}
                }
                if(!found)
                {
                    ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                    return 0;
                }
                subscribe_index = 0;
                subscribe();
            }
            else if(error->status == 0)
            {
                if(ble_uuid_u16(&descriptor->uuid.u) == 0x2902)
                {
                    for(uint8_t i = 0; i < report_count; i++)
                    {
                        if(reports[i].value == value)
                        {
                            reports[i].cccd = descriptor->handle;
                            break;
                        }
                    }
                }
            }
            else
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }

        /**
         * @brief 查找 HID 服务内可通知的报告
         */
        int characteristic_found(uint16_t, const ble_gatt_error *error,
            const ble_gatt_chr *characteristic, void *)
        {
            if(error->status == BLE_HS_EDONE)
            {
                if(report_count == 0 || ble_gattc_disc_all_dscs(connection,
                    service_start, service_end, descriptor_found, nullptr) != 0)
                {
                    ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
            else if(error->status == 0)
            {
                if(ble_uuid_u16(&characteristic->uuid.u) == 0x2a4d &&
                    (characteristic->properties & BLE_GATT_CHR_F_NOTIFY) &&
                    report_count < MAX_REPORTS)
                {
                    reports[report_count++].value = characteristic->val_handle;
                }
            }
            else
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }

        /**
         * @brief 查找 Xbox HID 服务
         */
        int service_found(uint16_t, const ble_gatt_error *error,
            const ble_gatt_svc *service, void *)
        {
            if(error->status == BLE_HS_EDONE)
            {
                if(service_start == 0 || ble_gattc_disc_all_chrs(connection,
                    service_start, service_end, characteristic_found, nullptr) != 0)
                {
                    ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
            else if(error->status == 0)
            {
                service_start = service->start_handle;
                service_end = service->end_handle;
            }
            else
            {
                ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
            }
            return 0;
        }

        /**
         * @brief 判断广播是否来自 Xbox 手柄
         */
        bool is_xbox(const ble_gap_disc_desc &device)
        {
            if(device.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
                device.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP)
            {
                return false;
            }

            ble_hs_adv_fields fields = {};
            if(ble_hs_adv_parse_fields(&fields, device.data,
                device.length_data) != 0 ||
                fields.mfg_data == nullptr)
            {
                return false;
            }
            const uint8_t normal[] = {0x06, 0x00, 0x00};
            const uint8_t searching[] = {0x06, 0x00, 0x03, 0x00, 0x80};
            return (fields.mfg_data_len == sizeof(normal) &&
                memcmp(fields.mfg_data, normal, sizeof(normal)) == 0) ||
                (fields.mfg_data_len == sizeof(searching) &&
                memcmp(fields.mfg_data, searching, sizeof(searching)) == 0);
        }

        /**
         * @brief 在 NimBLE 主机任务中执行网页发来的扫描或切换
         */
        void handle_command(ble_npl_event *)
        {
            uint8_t next;
            portENTER_CRITICAL(&state_lock);
            next = command;
            command = 0;
            portEXIT_CRITICAL(&state_lock);
            if(next == 1)
            {
                if(ble_gap_disc_active() && ble_gap_disc_cancel() != 0)
                {
                    manual_scan_pending = false;
                    return;
                }
                manual_scan = true;
                manual_scan_pending = false;
                ble_gap_disc_params params = {};
                params.passive = 0;
                params.filter_duplicates = 0;
                if(ble_gap_disc(own_address_type, 5000, &params,
                    gap_event, nullptr) != 0)
                {
                    manual_scan = false;
                    scan();
                }
            }
            else if(next == 2)
            {
                manual_scan = false;
                manual_scan_pending = false;
                if(ble_gap_disc_active()){ble_gap_disc_cancel();}
                if(connection != UINT16_MAX)
                {
                    ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                }
                else{scan();}
            }
        }

        /**
         * @brief 处理 BLE 连接、配对与输入通知
         */
        int gap_event(ble_gap_event *event, void *)
        {
            switch(event->type)
            {
                case BLE_GAP_EVENT_DISC:
                    if(is_xbox(event->disc))
                    {
                        const ble_addr_t address = event->disc.addr;
                        if(manual_scan)
                        {
                            portENTER_CRITICAL(&state_lock);
                            uint8_t index = 0;
                            while(index < discovered_count &&
                                memcmp(&discovered_addresses[index], &address,
                                    sizeof(address)) != 0)
                            {
                                index++;
                            }
                            if(index == discovered_count && index < MAX_DEVICES)
                            {
                                discovered_addresses[index] = address;
                                address_text(address, discovered[index].address);
                                discovered[index].rssi = event->disc.rssi;
                                discovered_count++;
                            }
                            portEXIT_CRITICAL(&state_lock);
                            break;
                        }
                        portENTER_CRITICAL(&state_lock);
                        const bool selected = target_set;
                        const ble_addr_t selected_address = target;
                        portEXIT_CRITICAL(&state_lock);
                        if(selected && memcmp(&selected_address, &address,
                            sizeof(address)) != 0)
                        {
                            break;
                        }
                        connecting = true;
                        ble_gap_disc_cancel();
                        if(ble_gap_connect(own_address_type, &address, 10000,
                            nullptr, gap_event, nullptr) != 0)
                        {
                            connecting = false;
                            scan();
                        }
                    }
                    break;

                case BLE_GAP_EVENT_CONNECT:
                    connecting = false;
                    if(event->connect.status != 0)
                    {
                        scan();
                        break;
                    }
                    connection = event->connect.conn_handle;
                    service_start = 0;
                    service_end = 0;
                    report_count = 0;
                    subscribe_index = 0;
                    memset(reports, 0, sizeof(reports));
                    disconnect();
                    portENTER_CRITICAL(&state_lock);
                    latest_state.connected = true;
                    portEXIT_CRITICAL(&state_lock);
                    if(ble_gap_security_initiate(connection) != 0)
                    {
                        ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                    }
                    break;

                case BLE_GAP_EVENT_ENC_CHANGE:
                    if(event->enc_change.status == 0)
                    {
                        if(ble_gattc_disc_svc_by_uuid(connection,
                            &HID_SERVICE.u, service_found, nullptr) == 0)
                        {
                            break;
                        }
                    }
                    ble_gap_terminate(connection, BLE_ERR_REM_USER_CONN_TERM);
                    break;

                case BLE_GAP_EVENT_PASSKEY_ACTION:
                {
                    ble_sm_io response = {};
                    if(event->passkey.params.action == BLE_SM_IOACT_INPUT)
                    {
                        response.action = BLE_SM_IOACT_INPUT;
                        response.passkey = 0;
                    }
                    else if(event->passkey.params.action == BLE_SM_IOACT_NUMCMP)
                    {
                        response.action = BLE_SM_IOACT_NUMCMP;
                        response.numcmp_accept = 1;
                    }
                    else
                    {
                        break;
                    }
                    ble_sm_inject_io(event->passkey.conn_handle, &response);
                    break;
                }

                case BLE_GAP_EVENT_NOTIFY_RX:
                    if(event->notify_rx.conn_handle == connection &&
                        OS_MBUF_PKTLEN(event->notify_rx.om) == 16)
                    {
                        bool known = false;
                        for(uint8_t i = 0; i < report_count; i++)
                        {
                            if(reports[i].value == event->notify_rx.attr_handle)
                            {
                                known = true;
                                break;
                            }
                        }
                        if(known)
                        {
                            uint8_t data[16];
                            if(os_mbuf_copydata(event->notify_rx.om, 0,
                                sizeof(data), data) == 0)
                            {
                                receive(data);
                            }
                        }
                    }
                    break;

                case BLE_GAP_EVENT_DISCONNECT:
                    connecting = false;
                    connection = UINT16_MAX;
                    disconnect();
                    scan();
                    break;

                case BLE_GAP_EVENT_DISC_COMPLETE:
                    if(manual_scan){manual_scan = false;}
                    if(!connecting){scan();}
                    break;
            }
            return 0;
        }

        /**
         * @brief 蓝牙主机就绪后开始扫描
         */
        void sync()
        {
            if(ble_hs_id_infer_auto(0, &own_address_type) == 0)
            {
                host_ready = true;
                scan();
            }
        }

        /**
         * @brief 运行 NimBLE 主机
         */
        void host_task(void *)
        {
            nimble_port_run();
            nimble_port_freertos_deinit();
        }
    }

    /**
     * @brief 初始化 Xbox BLE 手柄连接
     *
     * @return true 蓝牙主机已启动
     */
    bool init()
    {
        if(started){return true;}
        if(nimble_port_init() != ESP_OK)
        {
            return false;
        }

        ble_hs_cfg.sync_cb = sync;
        ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
        ble_hs_cfg.sm_bonding = 1;
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_sc = 0;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
        ble_store_config_init();
        nvs_handle_t storage;
        if(nvs_open("gamepad", NVS_READONLY, &storage) == ESP_OK)
        {
            size_t size = sizeof(target);
            target_set = nvs_get_blob(storage, "target", &target,
                &size) == ESP_OK && size == sizeof(target);
            nvs_close(storage);
        }
        ble_npl_event_init(&command_event, handle_command, nullptr);
        nimble_port_freertos_init(host_task);
        started = true;
        return true;
    }

    /**
     * @brief 读取最近一次 Xbox 输入快照
     *
     * @param[out] out 手柄输入快照
     *
     * @return true 手柄已连接且收到过输入报告
     */
    bool get_state(state &out)
    {
        portENTER_CRITICAL(&state_lock);
        out = latest_state;
        portEXIT_CRITICAL(&state_lock);
        return out.connected && out.timestamp_us != 0;
    }

    /**
     * @brief 开始五秒手柄扫描
     */
    bool scan_devices()
    {
        if(!started || !host_ready){return false;}
        portENTER_CRITICAL(&state_lock);
        discovered_count = 0;
        manual_scan_pending = true;
        command = 1;
        portEXIT_CRITICAL(&state_lock);
        ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &command_event);
        return true;
    }

    /**
     * @brief 复制手柄扫描结果
     */
    uint8_t get_devices(device *out, uint8_t capacity)
    {
        portENTER_CRITICAL(&state_lock);
        const uint8_t count = discovered_count < capacity ?
            discovered_count : capacity;
        memcpy(out, discovered, count * sizeof(device));
        portEXIT_CRITICAL(&state_lock);
        return count;
    }

    /**
     * @brief 获取目标手柄和扫描状态
     */
    discovery get_discovery()
    {
        discovery snapshot;
        portENTER_CRITICAL(&state_lock);
        if(target_set){address_text(target, snapshot.target);}
        snapshot.scanning = manual_scan || manual_scan_pending;
        portEXIT_CRITICAL(&state_lock);
        return snapshot;
    }

    /**
     * @brief 保存扫描列表中的手柄为连接目标
     */
    bool select_device(uint8_t index)
    {
        ble_addr_t selected;
        portENTER_CRITICAL(&state_lock);
        const bool found = index < discovered_count;
        if(found){selected = discovered_addresses[index];}
        portEXIT_CRITICAL(&state_lock);
        if(!found){return false;}
        nvs_handle_t storage;
        if(nvs_open("gamepad", NVS_READWRITE, &storage) != ESP_OK)
        {
            return false;
        }
        const bool saved = nvs_set_blob(storage, "target", &selected,
            sizeof(selected)) == ESP_OK && nvs_commit(storage) == ESP_OK;
        nvs_close(storage);
        if(!saved){return false;}
        portENTER_CRITICAL(&state_lock);
        target = selected;
        target_set = true;
        command = 2;
        portEXIT_CRITICAL(&state_lock);
        ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &command_event);
        return true;
    }
}
