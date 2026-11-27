/* Modified for Nexfoot; upstream notices are retained. */
/*********************************************************************************************
    *   Filename        : ble_trans_search.c

    *   Description     : 配置搜索指定的services

    *   Author          :JM

    *   Email           : zh-jieli.com

    *   Last modifiled  : 2022-9-17 11:14

    *   Copyright:(c)JIELI  2011-2016  @ , All Rights Reserved.
*********************************************************************************************/
/*
 * 修改日期: 2026-02-10
 * 修改说明: 按方案2，将GATT Client搜索表切换为BLE MIDI(UUID128)，并增加BLE MIDI收发句柄与发送接口
 */
#include "system/app_core.h"
#include "system/includes.h"

#include "app_config.h"
#include "app_action.h"

#include "btstack/btstack_task.h"
#include "btstack/bluetooth.h"
#include "user_cfg.h"
#include "vm.h"
#include "btcontroller_modules.h"
#include "bt_common.h"
#include "3th_profile_api.h"
#include "le_common.h"
#include "rcsp_bluetooth.h"
#include "custom_cfg.h"
#include "btstack/btstack_event.h"
#include "le_client_demo.h"
#include "gatt_common/le_gatt_common.h"
// 修改日期: 2026-02-10, 修改说明: 引入统一连接状态接口(连接后停止分时切换/禁止再开scan&adv)
#include "ble_trans.h"

#if CONFIG_APP_SPP_LE && CONFIG_BT_GATT_CLIENT_NUM

#if LE_DEBUG_PRINT_EN
#define log_info(x, ...)  printf("[TRANS-SEARCH]" x " ", ## __VA_ARGS__)
#define log_info_hexdump  put_buf

#else
#define log_info(...)
#define log_info_hexdump(...)
#endif


static int trans_client_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param);
static void trans_scan_conn_config_set(struct ctl_pair_info_t *pair_info);
int trans_client_search_remote_profile(u16 conn_handle);
int trans_client_search_remote_stop(u16 conn_handle);

const gatt_client_cfg_t trans_client_init_cfg = {
    .event_packet_handler = trans_client_event_packet_handler,
};
//---------------------------------------------------------------------------
#define UUID_HANDLE_MAX   16
typedef struct {
    u16 uuid16;
    /* u16 uuid128[16]; */
    u16 value_handle;
    u16 properties;
} match_uuid_t;
static match_uuid_t uuid_handle_table[UUID_HANDLE_MAX];
static u16 match_uuid_cnt;

static u16 trans_client_conn_handle;
static u16 trans_client_read_handle;
static u16 trans_client_write_handle;
static u16 trans_client_notify_handle;
static u16 trans_client_midi_io_handle;
static u16 trans_client_midi_properties;
static att_op_type_e trans_client_midi_write_op;

#define TRANS_TEST_READ_SEND            0 //测试发数
//---------------------------------------------------------------------------
static const u8 ble_midi_service_uuid128[16] = {0x03, 0xB8, 0x0E, 0x5A, 0xED, 0xE8, 0x4B, 0x33, 0xA7, 0x51, 0x6C, 0xE3, 0x4E, 0xC4, 0xC7, 0x00};
static const u8 ble_midi_io_char_uuid128[16] = {0x77, 0x72, 0xE5, 0xDB, 0x38, 0x68, 0x41, 0x12, 0xA1, 0xA9, 0xF2, 0x66, 0x9D, 0x10, 0x6B, 0xF3};
// 修改日期: 2026-02-10, 修改说明: 增加分时互斥角色标志位(主机扫描时才允许按BLE MIDI UUID尝试连接)
static u8 trans_midi_scan_connect_enable;

#ifndef CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
#define CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG 0
#endif

// 修改日期: 2026-02-10, 修改说明: 主机扫描时解析并打印广播设备名称(AD type: 0x09/0x08)
static u8 trans_adv_get_local_name(const u8 *adv_data, u8 adv_len, u8 *name_buf, u8 name_buf_len)
{
    u8 i = 0;
    u8 field_len;
    u8 ad_type;

    if (!adv_data || !adv_len || !name_buf || (name_buf_len < 2)) {
        return 0;
    }

    while ((i + 1) < adv_len) {
        field_len = adv_data[i];
        if (field_len == 0) {
            break;
        }
        if ((i + field_len) >= adv_len) {
            break;
        }
        ad_type = adv_data[i + 1];
        if ((ad_type == 0x09) || (ad_type == 0x08)) {
            u8 name_len = field_len - 1;
            u8 copy_len = (name_len < (name_buf_len - 1)) ? name_len : (name_buf_len - 1);
            memcpy(name_buf, &adv_data[i + 2], copy_len);
            name_buf[copy_len] = 0;
            return copy_len;
        }
        i += (field_len + 1);
    }
    return 0;
}

//指定搜索uuid
//指定搜索uuid
static const target_uuid_t  jl_trans_search_uuid_table[] = {

    {
        .services_uuid16 = 0,
        .services_uuid128 = {0x03, 0xB8, 0x0E, 0x5A, 0xED, 0xE8, 0x4B, 0x33, 0xA7, 0x51, 0x6C, 0xE3, 0x4E, 0xC4, 0xC7, 0x00},
        .characteristic_uuid16 = 0,
        .characteristic_uuid128 = {0x77, 0x72, 0xE5, 0xDB, 0x38, 0x68, 0x41, 0x12, 0xA1, 0xA9, 0xF2, 0x66, 0x9D, 0x10, 0x6B, 0xF3},
        .opt_type = ATT_PROPERTY_NOTIFY,
    },

};

//测试write数据操作
static void trans_client_test_read(void)
{
#if TRANS_TEST_READ_SEND
    int ret = 0;
    u16 tmp_handle = trans_client_conn_handle;

    if (tmp_handle && trans_client_read_handle) {
        ret = ble_comm_att_send_data(tmp_handle, trans_client_read_handle, 0, 0, ATT_OP_READ);
        log_info("test_read:%04x, %04x, %d ", tmp_handle, trans_client_read_handle, ret);
    }

    static int write_count;
    if (tmp_handle && trans_client_write_handle) {
        ret = ble_comm_att_send_data(tmp_handle, trans_client_write_handle, &write_count, 4, ATT_OP_WRITE_WITHOUT_RESPOND);
        log_info("test_write:%04x, %04x, %d ", tmp_handle, trans_client_write_handle, ret);
        if (!ret) {
            write_count++;
        }
    } else {
        write_count = 0;
    }

#endif
}

//配置扫描匹配连接的设备，已经连上后搜索匹配的profile uuid
static const gatt_search_cfg_t trans_client_search_config = {
    .match_devices = NULL,
    .match_devices_count = 0,
    .match_rssi_enable = 0,

    .search_uuid_group = jl_trans_search_uuid_table,
    .search_uuid_count = (sizeof(jl_trans_search_uuid_table) / sizeof(target_uuid_t)),
    .auto_enable_ccc = 1,
};

/*handle 转换 对应的uuid*/
static u16 __handle_to_uuid(u16 value_handle)
{
    u8 i;
    match_uuid_t *t;
    for (i = 0; i < match_uuid_cnt; i++) {
        t = &uuid_handle_table[i];
        if (t->value_handle == value_handle) {
            return t->uuid16;
        }
    }
    return 0xFFFF;
}

/*解析对应handle的数据内容*/
static void __read_info_handle(u16 value_handle, u8 *data, u16 len)
{
    u16 uuid16 = __handle_to_uuid(value_handle);
    switch (uuid16) {
    case 0x2a00:
        log_info("0x2a00: Device Name: %s", data);
        break;

    case 0x2a19:
        log_info("0x2a19: Battery Level: %d", data[0]);
        break;

    case 0x2a2b:
        log_info("0x2a2b: Current Time: %d-%d-%d %d:%d:%d week:%d",
                 (data[1] << 8) | data[0], data[2], data[3],
                 data[4], data[5], data[6], data[7]);
        break;

    case 0x2a0f:
        char utc = data[0];
        log_info("0x2a0f: Local Time Information: UTC:%d", utc / 4);
        break;

    case 0x2a29:
        log_info("0x2a29: Manufacturer Name String: %s", data);
        break;

    case 0x2a24:
        log_info("0x2a24: Model Number String: %s", data);
        break;

    default:
        break;
    }
}

//-------------------------------------------------------------------------------------
// 修改日期: 2026-02-10, 修改说明: BLE MIDI 设备广播解析(支持128bit Service UUID字段匹配)
static u8 trans_adv_has_uuid128(const u8 *adv_data, u8 adv_len, const u8 uuid128[16])
{
    u8 i = 0;
    const u8 *p = adv_data;

    while (i < adv_len) {
        u8 field_len;
        u8 field_type;
        u8 value_len;

        field_len = p[0];
        if (field_len == 0) {
            break;
        }
        if ((i + 1 + field_len) > adv_len) {
            break;
        }
        field_type = p[1];
        value_len = field_len - 1;

        if ((field_type == HCI_EIR_DATATYPE_MORE_128BIT_SERVICE_UUIDS) ||
            (field_type == HCI_EIR_DATATYPE_COMPLETE_128BIT_SERVICE_UUIDS)) {
            const u8 *uuid_list = &p[2];
            u8 uuid_list_len = value_len;
            u8 off = 0;
            while ((off + 16) <= uuid_list_len) {
                if (!memcmp(&uuid_list[off], uuid128, 16)) {
                    return 1;
                }
                {
                    u8 k;
                    u8 match = 1;
                    for (k = 0; k < 16; k++) {
                        if (uuid_list[off + k] != uuid128[15 - k]) {
                            match = 0;
                            break;
                        }
                    }
                    if (match) {
                        return 1;
                    }
                }
                off += 16;
            }
        }

        i += (field_len + 1);
        p += (field_len + 1);
    }

    return 0;
}

//-------------------------------------------------------------------------------------
//处理gatt 回调的事件，hci & gatt
static int trans_client_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param)
{
    /* log_info("event: %02x,size= %d\n",event,size); */
    switch (event) {
    case GATT_COMM_EVENT_SCAN_ADV_REPORT: {
        adv_report_t *report_pt = (adv_report_t *)packet;
        if (!trans_midi_scan_connect_enable) {
            break;
        }

        if ((report_pt->event_type != 0) && (report_pt->event_type != 1)) {
            break;
        }
        // 修改日期: 2026-02-10, 修改说明: 只打印符合BLE MIDI(包含MIDI Service UUID128)的设备名称
        if (trans_adv_has_uuid128(report_pt->data, report_pt->length, ble_midi_service_uuid128)) {
            u8 name_buf[32];
#if CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
            u8 has_name;
#endif
            if (trans_adv_get_local_name(report_pt->data, report_pt->length, name_buf, sizeof(name_buf))) {
                log_info("scan_midi:rssi:%d,name:%s", report_pt->rssi, name_buf);
            } else {
                log_info("scan_midi:rssi:%d,name:<unknown>", report_pt->rssi);
            }
#if CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
            has_name = trans_adv_get_local_name(report_pt->data, report_pt->length, name_buf, sizeof(name_buf));
            printf("[TRANS-SEARCH] scan midi addr=%02X:%02X:%02X:%02X:%02X:%02X raw=%02X %02X %02X %02X %02X %02X type=%u evt=%u rssi=%d name=%s\r\n",
                   report_pt->address[5], report_pt->address[4], report_pt->address[3],
                   report_pt->address[2], report_pt->address[1], report_pt->address[0],
                   report_pt->address[0], report_pt->address[1], report_pt->address[2],
                   report_pt->address[3], report_pt->address[4], report_pt->address[5],
                   report_pt->address_type, report_pt->event_type, report_pt->rssi,
                   has_name ? (char *)name_buf : "<unknown>");
#endif

            int ret;
            ble_gatt_client_scan_enable(0);
            ret = ble_gatt_client_create_connection_request(report_pt->address, report_pt->address_type, 0);
#if CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
            printf("[TRANS-SEARCH] connect request ret=%d\r\n", ret);
#endif
            if (ret) {
                ble_gatt_client_scan_enable(1);
            } else {
                trans_midi_scan_connect_enable = 0;
            }
        }
    }
    break;

    case GATT_COMM_EVENT_GATT_DATA_REPORT: {
        att_data_report_t *report_data = (void *)packet;
        switch (report_data->packet_type) {
        case GATT_EVENT_NOTIFICATION://notify
            if (trans_client_midi_io_handle && report_data->value_handle == trans_client_midi_io_handle) {
                // 修改日期: 2026-02-10, 修改说明: BLE MIDI client接收统一入口(落地rx_buf并触发上层回调)
                trans_ble_midi_on_rx(report_data->conn_handle, report_data->blob, report_data->blob_length);
            }
            break;

        case GATT_EVENT_INDICATION://indicate
            log_info("indicate_report:");
            break;

        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT://read
            log_info("read_report:");
            __read_info_handle(report_data->value_handle, report_data->blob, report_data->blob_length);
            break;

        case GATT_EVENT_LONG_CHARACTERISTIC_VALUE_QUERY_RESULT://read long
            log_info("read_long_report:");
            __read_info_handle(report_data->value_handle, report_data->blob, report_data->blob_length);
            break;

        case GATT_EVENT_QUERY_COMPLETE:
            /*ATT读写操作等结果返回,对应spec的att error*/
            log_info("QUERY_COMPLETE:att_error: %02x\n", report_data->blob[0]);
            break;

        default:
            log_info("unknow_report:");
            break;
        }
        // 修改日期: 2026-02-10, 修改说明: 关闭TRANS-SEARCH数据上报调试打印(避免notify数据刷屏)
    }
    break;

    case GATT_COMM_EVENT_GATT_SEARCH_MATCH_UUID: {
        opt_handle_t *opt_hdl = packet;
        log_info("match:server_uuid= %04x,charactc_uuid= %04x,value_handle= %04x\n", \
                 opt_hdl->search_uuid->services_uuid16, opt_hdl->search_uuid->characteristic_uuid16, opt_hdl->value_handle);
        if (opt_hdl->search_uuid->services_uuid16 == 0) {
            log_info("match:server_uuid128:");
            log_info_hexdump(opt_hdl->search_uuid->services_uuid128, 16);
            log_info("[DEBUG] Searching for BLE MIDI service: ");
            log_info_hexdump(ble_midi_service_uuid128, 16);
        }
        if (opt_hdl->search_uuid->characteristic_uuid16 == 0) {
            log_info("match:charactc_uuid128:");
            log_info_hexdump(opt_hdl->search_uuid->characteristic_uuid128, 16);
            log_info("[DEBUG] Searching for BLE MIDI char: ");
            log_info_hexdump(ble_midi_io_char_uuid128, 16);
        }

        if (match_uuid_cnt < UUID_HANDLE_MAX) {
            uuid_handle_table[match_uuid_cnt].uuid16 = opt_hdl->search_uuid->characteristic_uuid16;
            uuid_handle_table[match_uuid_cnt].properties = opt_hdl->properties;
            uuid_handle_table[match_uuid_cnt].value_handle = opt_hdl->value_handle;
            match_uuid_cnt++;
        }

#if TRANS_TEST_READ_SEND
        //for test
#if CONFIG_BT_SM_SUPPORT_ENABLE
        if (opt_hdl->search_uuid->characteristic_uuid16 == 0x2a2b) {
#else
        if (opt_hdl->search_uuid->characteristic_uuid16 == 0x2a00) {
#endif
            trans_client_read_handle = opt_hdl->value_handle;
            log_info("get read_handle= %04x\n", trans_client_read_handle);
        }

        if (opt_hdl->search_uuid->characteristic_uuid16 == 0xae01) {
            trans_client_write_handle = opt_hdl->value_handle;
            log_info("get write_handle= %04x\n", trans_client_write_handle);
        }

        if (opt_hdl->search_uuid->characteristic_uuid16 == 0xae02) {
            trans_client_notify_handle = opt_hdl->value_handle;
            log_info("get notify_handle= %04x\n", trans_client_notify_handle);
        }

#endif

        if ((0 == opt_hdl->search_uuid->services_uuid16) &&
            (0 == opt_hdl->search_uuid->characteristic_uuid16) &&
            (0 == memcmp(opt_hdl->search_uuid->services_uuid128, ble_midi_service_uuid128, 16)) &&
            (0 == memcmp(opt_hdl->search_uuid->characteristic_uuid128, ble_midi_io_char_uuid128, 16)) &&
            (opt_hdl->properties & ATT_PROPERTY_NOTIFY) &&
            (opt_hdl->properties & (ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_WRITE))) {
            trans_client_midi_io_handle = opt_hdl->value_handle;
            trans_client_write_handle = opt_hdl->value_handle;
            trans_client_notify_handle = opt_hdl->value_handle;
            trans_client_midi_properties = opt_hdl->properties;
            trans_client_midi_write_op =
                (opt_hdl->properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) ?
                ATT_OP_WRITE_WITHOUT_RESPOND : ATT_OP_WRITE;
            log_info("get ble_midi_io_handle= %04x props=%02x op=%u\n",
                     trans_client_midi_io_handle, opt_hdl->properties,
                     trans_client_midi_write_op);
#if CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
            printf("[TRANS-SEARCH] match midi io handle=%04x props=%02x write_op=%u\r\n",
                   trans_client_midi_io_handle, opt_hdl->properties,
                   trans_client_midi_write_op);
#endif
        } else if ((0 == opt_hdl->search_uuid->services_uuid16) &&
                   (0 == opt_hdl->search_uuid->characteristic_uuid16) &&
                   (0 == memcmp(opt_hdl->search_uuid->services_uuid128, ble_midi_service_uuid128, 16)) &&
                   (0 == memcmp(opt_hdl->search_uuid->characteristic_uuid128, ble_midi_io_char_uuid128, 16))) {
#if CONFIG_MD400_BLE_CENTRAL_SCAN_DIAG
            printf("[TRANS-SEARCH] reject midi io handle=%04x props=%02x need notify+write\r\n",
                   opt_hdl->value_handle, opt_hdl->properties);
#endif
        }
    }
    break;

    case GATT_COMM_EVENT_GATT_SEARCH_DESCRIPTOR_RESULT: {
        log_info("GATT_COMM_EVENT_GATT_SEARCH_DESCRIPTOR_RESULT");
        charact_descriptor_t *result_descriptor = ext_param;
    }
    break;

    case GATT_COMM_EVENT_GATT_SEARCH_PROFILE_COMPLETE:
        log_info("GATT_SEARCH_PROFILE_COMPLETE:%04x\n", little_endian_read_16(packet, 0));
        if (trans_client_conn_handle &&
            (trans_client_conn_handle == little_endian_read_16(packet, 0)) &&
            !trans_client_midi_io_handle) {
            printf("[TRANS-SEARCH] BLE MIDI IO not found, disconnect handle=%04x\r\n",
                   trans_client_conn_handle);
            ble_comm_disconnect(trans_client_conn_handle);
        }
        break;

    case GATT_COMM_EVENT_CLIENT_STATE:
        log_info("client_state: handle=%02x,%02x\n", little_endian_read_16(packet, 1), packet[0]);
        // 修改日期: 2026-02-10, 修改说明: client连接/断开时同步统一连接状态，避免连接后仍分时切换
        {
            u16 conn_handle = little_endian_read_16(packet, 1);
            u8 st = packet[0];
            if (conn_handle) {
                if (st == BLE_ST_CONNECT) {
                    trans_client_search_remote_profile(conn_handle);
                    trans_comm_connected_handle_on_connect(conn_handle);
                } else if ((st == BLE_ST_DISCONN) || (st == BLE_ST_CONNECT_FAIL)) {
                    trans_client_search_remote_stop(conn_handle);
                    trans_comm_connected_handle_on_disconnect(conn_handle);
                }
            } else {
                if ((st == BLE_ST_DISCONN) || (st == BLE_ST_CONNECT_FAIL)) {
                    u16 cur = trans_comm_connected_handle_get();
                    if (cur) {
                        trans_client_search_remote_stop(cur);
                        trans_comm_connected_handle_on_disconnect(cur);
                    }
                }
            }
        }
        break;

    default:
        break;
    }
    return 0;
}

/*启动搜索服务*/
int trans_client_search_remote_profile(u16 conn_handle)
{
#if TRANS_CLIENT_SEARCH_PROFILE_ENABLE
    log_info("%s, conn_handle=%04x", __FUNCTION__, conn_handle);
    ble_gatt_server_adv_enable(0);
    ble_gatt_just_search_profile_start(conn_handle);
    trans_client_conn_handle = conn_handle;
    match_uuid_cnt = 0;
    trans_client_read_handle = 0;
    trans_client_write_handle = 0;
    trans_client_notify_handle = 0;
    trans_client_midi_io_handle = 0;
    trans_client_midi_properties = 0;
    trans_client_midi_write_op = 0;
    log_info("[DEBUG] Search started, waiting for GATT_COMM_EVENT_GATT_SEARCH_MATCH_UUID...");
#endif
    return 0;
}

/*停止搜索服务*/
int trans_client_search_remote_stop(u16 conn_handle)
{
#if TRANS_CLIENT_SEARCH_PROFILE_ENABLE
    log_info("%s", __FUNCTION__);
    if (conn_handle == trans_client_conn_handle) {
        ble_gatt_just_search_profile_stop(conn_handle);
        trans_client_conn_handle = 0;
        trans_client_read_handle = 0;
        trans_client_write_handle = 0;
        trans_client_notify_handle = 0;
        trans_client_midi_io_handle = 0;
        trans_client_midi_properties = 0;
        trans_client_midi_write_op = 0;
        ble_gatt_server_adv_enable(1);
    }
#endif
    return 0;
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI client发送能力查询(用于统一发送泵判断当前是否处于client链路)
 */
u8 trans_ble_midi_client_is_ready(void)
{
    return (trans_client_conn_handle && trans_client_midi_io_handle && trans_client_midi_write_op) ? 1 : 0;
}

void trans_ble_midi_client_diag(u16 *conn_handle, u16 *io_handle,
                                u16 *write_handle, u16 *notify_handle)
{
    if (conn_handle) {
        *conn_handle = trans_client_conn_handle;
    }
    if (io_handle) {
        *io_handle = trans_client_midi_io_handle;
    }
    if (write_handle) {
        *write_handle = trans_client_write_handle;
    }
    if (notify_handle) {
        *notify_handle = trans_client_notify_handle;
    }
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI client侧立即发送(用于统一发送泵内部调用)
 */
int trans_ble_midi_client_send_now(const u8 *data, u16 len)
{
    if (!data || !len) {
        return GATT_CMD_PARAM_ERROR;
    }
    if (!trans_client_conn_handle || !trans_client_midi_io_handle) {
        return GATT_CMD_STACK_NOT_RUN;
    }
    if (!trans_client_midi_write_op) {
        return GATT_CMD_PARAM_ERROR;
    }
    if (!ble_comm_att_check_send(trans_client_conn_handle, len)) {
        return GATT_BUFFER_FULL;
    }
    return ble_comm_att_send_data(trans_client_conn_handle, trans_client_midi_io_handle,
                                  (void *)data, len, trans_client_midi_write_op);
}


/*固定默认配置,不做scan操作*/
static const scan_conn_cfg_t trans_client_scan_cfg = {
    .scan_auto_do = 0,
    .creat_auto_do = 0,
};

//trans client 初始化
void trans_client_init(void)
{
    log_info("%s", __FUNCTION__);
    ble_gatt_client_set_scan_config(&trans_client_scan_cfg);//默认要配置,但不开scan

#if TRANS_CLIENT_SEARCH_PROFILE_ENABLE
    ble_gatt_client_set_search_config(&trans_client_search_config);

#if TRANS_TEST_READ_SEND
    sys_timer_add(0, trans_client_test_read, 500);
#endif
#endif
}

//trans client exit
void trans_client_exit(void)
{
#if TRANS_CLIENT_SEARCH_PROFILE_ENABLE
    log_info("%s", __FUNCTION__);
#endif
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: 分时互斥角色控制入口(主机扫描阶段才允许按BLE MIDI UUID尝试连接)
 */
void trans_client_set_midi_scan_connect_enable(u8 en)
{
    trans_midi_scan_connect_enable = !!en;
}

#endif
