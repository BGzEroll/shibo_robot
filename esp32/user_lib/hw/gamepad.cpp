#include "gamepad.h"

#include "sys_time.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"

#include <string.h>

extern "C" void ble_store_config_init(void);

namespace gamepad
{
    namespace
    {
        constexpr uint8_t MAX_REPORTS = 8;
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

        int gap_event(ble_gap_event *event, void *);

        /**
         * @brief 搜索旧项目使用的 Xbox BLE 手柄
         */
        void scan()
        {
            if(connecting || connection != UINT16_MAX){return;}
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
            if(ble_hs_id_infer_auto(0, &own_address_type) == 0){scan();}
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
        if(nvs_flash_init() != ESP_OK || nimble_port_init() != ESP_OK)
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
}
