/* Modified for Nexfoot; upstream notices are retained. */
/*********************************************************************************************
    *   Filename        : .c

    *   Description     :

    *   Author          : JM

    *   Email           : zh-jieli.com

    *   Last modifiled  : 2017-01-17 11:14

    *   Copyright:(c)JIELI  2011-2016  @ , All Rights Reserved.
*********************************************************************************************/
#include "system/app_core.h"
#include "system/includes.h"

#include "app_config.h"
#include "md400_ble_codec.h"
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
#include "JL_rcsp_api.h"
#include "custom_cfg.h"
#include "btstack/btstack_event.h"
#include "gatt_common/le_gatt_common.h"
#include "ble_trans.h"
#include "ble_trans_profile.h"
#include "asm/gpio.h"
#include "asm/cpu.h"
#include "asm/wdt.h"
#include "asm/uart_dev.h"
#include "usb/device/usb_stack.h"
#include "usb/otg.h"
#include "os/os_error.h"
#include "system/fs/fs.h"
#include "md400_display.h"
#include "md400_ms32.h"
#include "md400_preset_storage.h"
#include "md400_flash_store.h"

#define syscfg_read md400_preset_storage_read
#define syscfg_write md400_preset_storage_write

#ifndef CONFIG_MD400_ENCODER_CC_ONLY_TEST
#define CONFIG_MD400_ENCODER_CC_ONLY_TEST 0
#endif
#ifndef CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
#define CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE 1
#endif
#ifndef CONFIG_MD400_MIDI_OUT_ENABLE
#define CONFIG_MD400_MIDI_OUT_ENABLE CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
#endif
#ifndef CONFIG_MD400_USB_MIDI_ONE_PORT_BASELINE
#define CONFIG_MD400_USB_MIDI_ONE_PORT_BASELINE 0
#endif
#ifndef CONFIG_MD400_ENCODER_LONG_PRESS_ENABLE
#define CONFIG_MD400_ENCODER_LONG_PRESS_ENABLE 0
#endif
#ifndef CONFIG_MD400_ENCODER_SHORT_PRESS_ENABLE
#define CONFIG_MD400_ENCODER_SHORT_PRESS_ENABLE 1
#endif
#ifndef CONFIG_MD400_ENCODER_DOUBLE_PRESS_ENABLE
#define CONFIG_MD400_ENCODER_DOUBLE_PRESS_ENABLE 0
#endif
#ifndef CONFIG_MD400_ENCODER_COMBO_PRESS_ENABLE
#define CONFIG_MD400_ENCODER_COMBO_PRESS_ENABLE 0
#endif
#ifndef CONFIG_MD400_ENCODER_PRESS_EDIT_ENABLE
#define CONFIG_MD400_ENCODER_PRESS_EDIT_ENABLE 0
#endif
#ifndef CONFIG_MD400_ENCODER_COMBO_NAV_ENABLE
#define CONFIG_MD400_ENCODER_COMBO_NAV_ENABLE 1
#endif

#if CONFIG_APP_SPP_LE

// 修改日期: 2026-02-17, 修改说明: USB MIDI发送由IN中断持续完成, 大循环负责触发kick检查
void usb_midi_ac632n_kick_tx(void);
static void ble_midi_tx_send_pump(void);
static int ble_midi_tx_write_drop_on_full(const u8 *data, u16 len);
static int usb_midi_tx_write_drop_on_full(const u8 *data, u16 len);
static int uart_midi_tx_write_drop_on_full(const u8 *data, u16 len);
extern int usb_midi_device_send(const usb_dev usb_id, const u8 *data, u32 len);
extern u16 usb_midi_device_tx_space(const usb_dev usb_id);
// 修改日期: 2026-02-17, 修改说明: 引用USB MIDI设备端发送接口
extern int usb_midi_device_send(const usb_dev usb_id, const u8 *data, u32 len);
void md400_midi_send_encoder_cc(u8 encoder, u8 value, u8 event);
void md400_midi_send_encoder_press(u8 encoder, u8 pressed, u8 value);

#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
static int usb1_midi_tx_write_drop_on_full(const u8 *data, u16 len);
extern int usb_midi_host_send(const usb_dev usb_id, const u8 *data, u32 len);
// 修改日期: 2026-02-18, 修改说明: 引用USB1 Host挂载状态
extern u8 usb1_host_mounted;
#endif

#if LE_DEBUG_PRINT_EN
#define log_info(x, ...)  printf("[BLE_TRANS]" x " ", ## __VA_ARGS__)
#define log_info_hexdump  put_buf

#else
#define log_info(...)
#define log_info_hexdump(...)
#endif

/* Production keeps CONFIG_DEBUG_ENABLE because the JL startup path depends on
 * its internal backend, but PA1/PA6 UART is disabled and application printf
 * traffic has no consumer.  Compile these strings/calls out locally to keep
 * the app below 0x40000 so firmware upgrades preserve the existing VM start. */
#ifndef CONFIG_MD400_APPLICATION_LOG_ENABLE
#if CONFIG_MD400_UART0_RX_TEST_ENABLE
#define CONFIG_MD400_APPLICATION_LOG_ENABLE 1
#else
#define CONFIG_MD400_APPLICATION_LOG_ENABLE 0
#endif
#endif
#if !CONFIG_MD400_APPLICATION_LOG_ENABLE
#define printf(...) do { } while (0)
#endif

/* All product VM mutations are serialized on app_core through Q_CALLBACK.
 * In particular, never run syscfg_write() from the 128-byte systimer task or
 * from the USB-MIDI request handler: an existing-item rewrite may erase and
 * compact VM before returning.  app_core is an SDK-owned task with a static
 * stack; a dynamically-created writer task watchdog-reset on real hardware. */
#define MD400_PERSIST_QUEUE_DEPTH     16

enum {
    MD400_PERSIST_MSG_NANOBRIDGE = 1,
    MD400_PERSIST_MSG_HEADER,
    MD400_PERSIST_MSG_STORE,
    MD400_PERSIST_MSG_PEDAL_SAVE,
    MD400_PERSIST_MSG_FOOT_MODE,
    MD400_PERSIST_MSG_BT_ENABLE,
    MD400_PERSIST_MSG_PEDAL_MODE,
    MD400_PERSIST_MSG_PEDAL_CAL,
    MD400_PERSIST_MSG_RUNTIME_SAVE,
    MD400_PERSIST_MSG_RUNTIME_LOAD,
    MD400_PERSIST_MSG_RUNTIME_FACTORY,
    MD400_PERSIST_MSG_WDT_GUARD_CLEAR,
    MD400_PERSIST_MSG_OTA_BLE_ARM,
    MD400_PERSIST_MSG_NB_FRAME,
    MD400_PERSIST_MSG_NB_NOTIFY,
    MD400_PERSIST_MSG_BLE_PAIR,
};

static volatile u8 md400_persist_in_task;
static int md400_persist_submit(u8 message, u8 arg0, u8 arg1);
static void md400_ota_ble_arm_persist(void);

#if defined(TCFG_BT_STATUS_LED_ENABLE) && (TCFG_BT_STATUS_LED_ENABLE)
// 修改日期: 2026-02-11, 修改说明: 蓝牙状态指示灯(未连接1Hz闪烁，连接常亮)，按工程隔离宏适配电平逻辑
static int bt_status_led_timer;
static u8 bt_status_led_level;

static void bt_status_led_write(u8 on)
{
    bt_status_led_level = !!on;
#if defined(AC6321A_BOARD_ONLY) && (AC6321A_BOARD_ONLY)
    gpio_direction_output(TCFG_BT_STATUS_LED_IO, bt_status_led_level ? 0 : 1);
#else
    gpio_direction_output(TCFG_BT_STATUS_LED_IO, bt_status_led_level);
#endif
}

static void bt_status_led_blink_tick(void)
{
    bt_status_led_write(!bt_status_led_level);
}

static void bt_status_led_set_connected(u8 connected)
{
    if (connected) {
        if (bt_status_led_timer) {
            sys_timer_del(bt_status_led_timer);
            bt_status_led_timer = 0;
        }
        bt_status_led_write(1);
    } else {
        bt_status_led_write(0);
        if (bt_status_led_timer == 0) {
            bt_status_led_timer = sys_timer_add(NULL, bt_status_led_blink_tick, 500);
        }
    }
}

static void bt_status_led_init(void)
{
    gpio_set_die(TCFG_BT_STATUS_LED_IO, 1);
    gpio_set_pull_down(TCFG_BT_STATUS_LED_IO, 0);
    gpio_set_pull_up(TCFG_BT_STATUS_LED_IO, 0);
    bt_status_led_timer = 0;
    bt_status_led_level = 0;
    bt_status_led_set_connected(0);
}

static void bt_status_led_deinit(void)
{
    if (bt_status_led_timer) {
        sys_timer_del(bt_status_led_timer);
        bt_status_led_timer = 0;
    }
    bt_status_led_write(0);
}
#endif

// 修改日期: 2026-02-10, 修改说明: 为分时互斥角色控制提前声明连接句柄
static u16 trans_con_handle;
// 修改日期: 2026-02-10, 修改说明: 统一client/server连接句柄(任一角色连接后停止分时切换)
static u16 trans_comm_connected_handle;

uint8_t ble_midi_rx_buf[BLE_MIDI_RX_BUFF_SIZE];
uint16_t ble_midi_rx_buf_rp = 0;
uint16_t ble_midi_rx_buf_wp = 0;

uint8_t ble_midi_tx_buf[BLE_MIDI_TX_BUFF_SIZE];
uint16_t ble_midi_tx_buf_rp = 0;
uint16_t ble_midi_tx_buf_wp = 0;

uint8_t usb_midi_rx_buf[USB_MIDI_RX_BUFF_SIZE];
uint16_t usb_midi_rx_buf_rp = 0;
uint16_t usb_midi_rx_buf_wp = 0;
uint8_t usb_midi_tx_buf[USB_MIDI_TX_BUFF_SIZE];
uint16_t usb_midi_tx_buf_rp = 0;
uint16_t usb_midi_tx_buf_wp = 0;

uint8_t uart_midi_rx_buf[UART_MIDI_RX_BUFF_SIZE];
uint16_t uart_midi_rx_buf_rp = 0;
uint16_t uart_midi_rx_buf_wp = 0;
uint8_t uart_midi_tx_buf[UART_MIDI_TX_BUFF_SIZE];
uint16_t uart_midi_tx_buf_rp = 0;
uint16_t uart_midi_tx_buf_wp = 0;

#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
uint8_t usb1_midi_rx_buf[USB1_MIDI_RX_BUFF_SIZE];
uint16_t usb1_midi_rx_buf_rp = 0;//接收数据读取指针
uint16_t usb1_midi_rx_buf_wp = 0;//接收数据写入指针

uint8_t usb1_midi_tx_buf[USB1_MIDI_TX_BUFF_SIZE];
uint16_t usb1_midi_tx_buf_rp = 0;//发送数据读取指针
uint16_t usb1_midi_tx_buf_wp = 0;//发送数据写入指针

// 修改日期: 2026-02-10, 修改说明: BLE MIDI统一接收回调(主机notify与从机write共用)
#endif
static void (*trans_ble_midi_rx_cb)(u16 conn_handle, const u8 *data, u16 len);

// 修改日期: 2026-02-10, 修改说明: BLE MIDI统一发送泵定时器(持续检测tx_buf并按角色发送)
static int ble_midi_tx_timer;

// 修改日期: 2026-02-16, 修改说明: 6个循环buffer满标志(不预留1字节，rp==wp时用该标志区分空/满)
static u8 ble_midi_rx_full;
static u8 ble_midi_tx_full;
static u8 usb_midi_rx_full;
static u8 usb_midi_tx_full;
static u8 uart_midi_rx_full;
static u8 uart_midi_tx_full;
static u32 ble_midi_rx_packet_bytes;
static u32 ble_midi_rx_queued_bytes;
static u32 ble_midi_rx_dropped_bytes;
static u32 ble_midi_tx_queued_bytes;
static u32 ble_midi_tx_written_bytes;
static u32 ble_midi_tx_packet_bytes;
static u32 ble_midi_tx_dropped_bytes;
static u32 ble_midi_tx_send_failures;
static u32 ble_midi_tx_ccc_disabled_drops;
static u16 ble_midi_connect_count;
static u16 ble_midi_disconnect_count;
// 修改日期: 2026-02-18, 修改说明: USB1 MIDI buffer满标志
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
static u8 usb1_midi_rx_full;
static u8 usb1_midi_tx_full;
static u32 usb1_midi_rx_packet_bytes;
static u32 usb1_midi_rx_queued_bytes;
static u32 usb1_midi_rx_dropped_bytes;
static u32 usb1_midi_tx_queued_bytes;
static u32 usb1_midi_tx_written_bytes;
static u32 usb1_midi_tx_packet_bytes;
static u32 usb1_midi_tx_dropped_bytes;
static u32 usb1_midi_tx_send_failures;
#endif

// 修改日期: 2026-02-11, 修改说明: UART<->BLE MIDI桥接使用的UART设备句柄(按工程配置引脚, 31.25K 8N1)
static const uart_bus_t *ble_midi_uart_bus;
// 修改日期: 2026-02-10, 修改说明: UART接收cbuf(必须为2的幂，开启中断DMA接收)
static u8 ble_midi_uart_rx_cbuf[256];
// 修改日期: 2026-02-10, 修改说明: UART<->BLE MIDI桥接轮询泵定时器
static int ble_midi_uart_timer;
#if CONFIG_MD400_MIDI_OUT_ENABLE
static const uart_bus_t *md400_midi_out_uart_bus;
static u32 md400_midi_out_queued_bytes;
static u32 md400_midi_out_written_bytes;
static u32 md400_midi_out_dropped_bytes;
static u32 md400_midi_out_open_failures;
#define MD400_MIDI_OUT_TRACE_SIZE 16
static u8 md400_midi_out_trace[MD400_MIDI_OUT_TRACE_SIZE];
static u8 md400_midi_out_trace_len;
#endif

// 修改日期: 2026-02-15, 修改说明: 配对信息定义（全局，不受条件编译限制）
// 参考 SDK 原版 fw-AC63_BT_SDK-master/apps/hid/modules/bt/ble_hogp.c
//------------------------------------------------------------------------------------------------------------
#define BLE_VM_HEAD_TAG           (0xB35C)
#define BLE_VM_TAIL_TAG           (0x5CB3)
#define REPEAT_DIRECT_ADV_COUNT   (2)      // 定向广播重试次数 * 1.28s

// 回连广播配置（参考SDK原版HID项目）
// 关键修改：使用普通广播（ADV_IND）
// 让iOS可以主动扫描并连接已配对的设备
// 定向广播(ADV_DIRECT_IND)不带广播数据，iOS无法通过扫描发现设备
#define PAIR_RECONNECT_ADV_EN         1    // 上电使用普通广播优先回连
// ADV_DIRECT_IND     : 高占空比，1.28s超时，interval固定2ms（不带广播数据，iOS无法识别）
// ADV_DIRECT_IND_LOW : 低占空比定向广播
// ADV_IND            : 普通广播（推荐，iOS兼容）
// iOS 16+ 会自动连接已配对的BLE MIDI设备
// 参考HID项目：用户主动断开后使用普通广播
static u8 trans_pair_reconnect_adv_type = ADV_IND;
#define PAIR_RECONNECT_ADV_INTERVAL   (32)    // 20ms，Apple规范要求的快速广播间隔
static u32 trans_pair_reconnect_adv_timeout = 5000;  // 回连广播持续5秒
static u16 trans_pair_adv_timeout_id = 0;

struct trans_pair_info_t {
    u16 head_tag;
    u8  pair_flag;
    u8  direct_adv_cnt;
    u8  peer_address_info[7];
    u16 tail_tag;
};

static u8 trans_pair_info_addr_is_valid(const u8 *addr_info)
{
    u8 i;

    if (!addr_info) {
        return 0;
    }

    for (i = 0; i < 7; i++) {
        if (addr_info[i] != 0x00 && addr_info[i] != 0xFF) {
            return 1;
        }
    }

    return 0;
}

static u8 trans_pair_info_is_valid(const struct trans_pair_info_t *info)
{
    if (!info) {
        return 0;
    }

    if (info->head_tag != BLE_VM_HEAD_TAG || info->tail_tag != BLE_VM_TAIL_TAG) {
        return 0;
    }

    if (info->pair_flag != 1) {
        return 0;
    }

    return trans_pair_info_addr_is_valid(info->peer_address_info);
}

static struct trans_pair_info_t trans_pair_info;
static void trans_conn_pair_vm_do(struct trans_pair_info_t *info, u8 rw_flag);
static u8 trans_cur_peer_addr_info[7];
static void trans_adv_config_set(void);  // 前向声明
//------------------------------------------------------------------------------------------------------------

#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
// 修改日期: 2026-04-02, 修改说明: 分时互斥角色控制优化
#define TRANS_ROLE_SCAN_TIME_MS        (3000)
#define TRANS_ROLE_ADV_TIME_MS         (5000)
#define TRANS_ROLE_DISCONN_DELAY_MS    (2000)
#define TRANS_ROLE_CONN_WAIT_MS        (500)

enum {
    TRANS_ROLE_MODE_CENTRAL = 0,
    TRANS_ROLE_MODE_PERIPHERAL,
};

static u8 trans_role_mode;
static u8 trans_role_enabled;
static u8 trans_role_inited;
static int trans_role_timeout_id;
static scan_conn_cfg_t trans_role_scan_cfg;
static void trans_role_timeout_del(void)
{
    if (trans_role_timeout_id) {
        sys_timeout_del(trans_role_timeout_id);
        trans_role_timeout_id = 0;
    }
}

static void trans_role_timeout_handler(int priv);

static void trans_role_timeout_add(u32 ms)
{
    trans_role_timeout_del();
    trans_role_timeout_id = sys_timeout_add(NULL, trans_role_timeout_handler, ms);
}

static void trans_role_scan_cfg_init_once(void)
{
    if (trans_role_inited) {
        return;
    }

    trans_role_scan_cfg.scan_auto_do = 0;
    trans_role_scan_cfg.creat_auto_do = 0;
    trans_role_scan_cfg.scan_type = SCAN_ACTIVE;
    trans_role_scan_cfg.scan_filter = 1;
    trans_role_scan_cfg.scan_interval = ADV_SCAN_MS(24);
    trans_role_scan_cfg.scan_window = ADV_SCAN_MS(8);

    trans_role_scan_cfg.creat_conn_interval = (6 * 4);
    trans_role_scan_cfg.creat_conn_latency = 0;
    trans_role_scan_cfg.creat_conn_super_timeout = 400;

    trans_role_scan_cfg.creat_state_timeout_ms = 8000;
    trans_role_scan_cfg.conn_update_accept = 1;

    ble_gatt_client_set_scan_config(&trans_role_scan_cfg);
    trans_role_inited = 1;
}

static void trans_role_enter_central(void)
{
    if (!trans_role_enabled) {
        return;
    }
    //printf("\n[ROLE] ========== Switching to CLIENT (CENTRAL) Mode ==========\n");
    //printf("[ROLE] Disabling SERVER advertising...\n");
    trans_role_mode = TRANS_ROLE_MODE_CENTRAL;
    ble_gatt_server_adv_enable(0);

    //printf("[ROLE] Initializing CLIENT scan config...\n");
    trans_role_scan_cfg_init_once();

    //printf("[ROLE] Enabling MIDI scan and connection...\n");
    trans_client_set_midi_scan_connect_enable(1);
    ble_gatt_client_scan_enable(1);

    //printf("[ROLE] Setting scan timeout: %d ms\n", TRANS_ROLE_SCAN_TIME_MS);
    trans_role_timeout_add(TRANS_ROLE_SCAN_TIME_MS);
    //printf("[ROLE] CLIENT mode activated, scanning for MIDI devices...\n");
    //printf("[ROLE] ===========================================================\n\n");
}

static void trans_role_enter_peripheral(void)
{
    if (!trans_role_enabled) {
        return;
    }
    trans_role_mode = TRANS_ROLE_MODE_PERIPHERAL;
    trans_client_set_midi_scan_connect_enable(0);
    ble_gatt_client_scan_enable(0);
    trans_adv_config_set();
    ble_gatt_server_adv_enable(1);
    trans_role_timeout_add(TRANS_ROLE_ADV_TIME_MS);
}

static void trans_role_timeout_handler(int priv)
{
    if (!trans_role_enabled) {
        return;
    }
    // 如果已经连接，不进行角色切换
    if (trans_comm_connected_handle) {
        printf("[ROLE] Already connected (handle=0x%04x), skip role switch\n", trans_comm_connected_handle);
        return;
    }

    if (trans_role_mode == TRANS_ROLE_MODE_CENTRAL) {
        // CLIENT模式超时处理
        // 检查是否正在建立连接
        if (ble_gatt_client_get_work_state() == BLE_ST_CREATE_CONN) {
            printf("[ROLE] CLIENT is creating connection, wait %d ms...\n", TRANS_ROLE_CONN_WAIT_MS);
            trans_role_timeout_add(TRANS_ROLE_CONN_WAIT_MS);
            return;
        }

        // CLIENT扫描超时，切换到SERVER广播
        printf("[ROLE] CLIENT scan timeout (%d ms), no MIDI device found\n", TRANS_ROLE_SCAN_TIME_MS);
        trans_role_enter_peripheral();
    } else {
        // SERVER模式超时处理
        // 注意：有配对信息时不会进入此分支（trans_role_enter_peripheral不设置超时）
        // 只有无配对信息时才会超时，切换到CLIENT扫描
        printf("[ROLE] SERVER advertising timeout (%d ms), no paired device, switching to CLIENT scan...\n", TRANS_ROLE_ADV_TIME_MS);
        trans_role_enter_central();
    }
}

static void trans_role_start(void)
{
    if (trans_comm_connected_handle) {
        return;
    }
    /* Give a waiting phone the first connection window after enabling BLE. */
    trans_role_enter_peripheral();
}

static void trans_role_stop(void)
{
    printf("\n[ROLE] ========== Stopping Role Switch Mechanism ==========\n");
    trans_role_timeout_del();
    trans_client_set_midi_scan_connect_enable(0);
    /* Disabling scanning does not cancel a controller create-connection. */
    if (ble_gatt_client_get_work_state() == BLE_ST_CREATE_CONN) {
        printf("[MD400_BT] cancel outgoing connection on stop\r\n");
        ble_gatt_client_create_connection_cannel();
    }
    ble_gatt_client_scan_enable(0);
    ble_gatt_server_adv_enable(0);
    printf("[ROLE] All scan and advertising stopped\n");
    printf("[ROLE] =======================================================\n\n");
}

static void trans_role_on_connected(void)
{
    printf("\n[ROLE] ========== Connection Established ==========\n");
    printf("[ROLE] Current mode: %s\n", trans_role_mode == TRANS_ROLE_MODE_CENTRAL ? "CLIENT" : "SERVER");
    printf("[ROLE] Stopping role switch mechanism...\n");

    trans_role_timeout_del();
    trans_client_set_midi_scan_connect_enable(0);
    ble_gatt_client_scan_enable(0);
    ble_gatt_server_adv_enable(0);

    printf("[ROLE] Connection maintained, scan/advertising disabled\n");
    printf("[ROLE] ===================================================\n\n");
}

static void trans_role_on_disconnected(void)
{
    if (trans_role_mode == TRANS_ROLE_MODE_CENTRAL) {
        printf("[ROLE] CLIENT disconnected, advertise before retry\r\n");
        trans_role_enter_peripheral();
        return;
    }
    trans_role_start();
}
#endif

/*
 * 修改日期: 2026-02-10
 * 修改说明: 统一client/server连接状态(任一角色连接后停止分时切换)
 */
u16 trans_comm_connected_handle_get(void)
{
    return trans_comm_connected_handle;
}

/* ---------- MD400 user BT on/off (persisted) + top-bar indicator ----------
 * The user toggles BLE from the edit preset page (encoder 2). The setting is
 * stored in VM id 104 and applied at bt_ble_init. The top-bar "BT" indicator
 * shows: off = gray, on/advertising = 1Hz blink, connected = solid. */
#define MD400_VM_BT_ENABLE_ID  104
#define MD400_VM_PEDAL_MODE_ID 105
#define MD400_VM_PEDAL_CAL_ID  106

enum {
    MD400_PEDAL_MODE_EXP = 0,
    MD400_PEDAL_MODE_SWT = 1,
};

#define MD400_PEDAL_CAL_MIN_DEFAULT 0
#define MD400_PEDAL_CAL_MAX_DEFAULT 255
#define MD400_PEDAL_CAL_MIN_SPAN    4
#define MD400_PEDAL_SWITCH_OFF_TH 48
#define MD400_PEDAL_SWITCH_ON_TH  80
#define MD400_PEDAL_MIDI_VALUE_DEADBAND 2
#define MD400_PEDAL_MIDI_ENDPOINT_MARGIN 1
#define MD400_PEDAL_UI_VALUE_DEADBAND 2
#define MD400_PEDAL_UI_RAW_DEADBAND 4
#define MD400_PEDAL_EDIT_ENTER_TH 126
#define MD400_PEDAL_EDIT_RELEASE_TH 122
#define MD400_PEDAL_EDIT_EXP_ENTER_HOLD_MS 220
#define MD400_PEDAL_EDIT_SWT_ENTER_HOLD_MS 220
#define MD400_MS32_PROTO_HEAD0    0xA5
#define MD400_MS32_PROTO_HEAD1    0x5A
#define MD400_MS32_PROTO_VER      0x01
#define MD400_MS32_TYPE_PEDAL_MODE 0x30
#define MD400_MS32_MODE_SYNC_RETRY_MS 250
#define MD400_MS32_MODE_SYNC_RETRY_MAX 20
#define MD400_MS32_MODE_SYNC_BACKOFF_MS 1000

void ble_module_enable(u8 en);

static u8 md400_bt_user_enable = 1;
static u8 md400_bt_user_loaded;
static u8 md400_pedal_mode = MD400_PEDAL_MODE_EXP;
static u8 md400_pedal_mode_loaded;
static u8 md400_pedal_cal_min8 = MD400_PEDAL_CAL_MIN_DEFAULT;
static u8 md400_pedal_cal_max8 = MD400_PEDAL_CAL_MAX_DEFAULT;
static u8 md400_pedal_cal_loaded;
static u8 md400_pedal_cal_active;
static u8 md400_pedal_cal_state;
static u8 md400_pedal_cal_capture_min8;
static u8 md400_pedal_cal_capture_max8;
static u8 md400_pedal_settings_seen;
static u8 md400_pedal_settings_last_value;
static u8 md400_pedal_settings_last_raw;
static u8 md400_pedal_settings_last_cal;
static u8 md400_pedal_settings_max_latched;
static u8 md400_pedal_display_seen;
static u8 md400_pedal_display_value;
static u16 md400_pedal_settings_enter_timer;
static u8 md400_pedal_settings_enter_channel;
static u8 md400_pedal_settings_enter_value;
static u8 md400_pedal_settings_enter_ready;
static u8 md400_pedal_settings_enter_mode;
static u8 md400_pedal_switch_state;
static u8 md400_pedal_switch_seen;
static u8 md400_ms32_tx_sequence;
static u8 md400_ms32_mode_sync_active;
static u8 md400_ms32_mode_sync_retry_count;
static u8 md400_ms32_mode_sync_target;
static u8 md400_ms32_mode_sync_timeout_logged;
static u32 md400_ms32_mode_sync_last_ms;

static void md400_pedal_settings_enter_reset(void);
static u32 md400_ms32_mode_sync_start_ack_count;

typedef struct {
    u8 magic[4];   /* 'M','D','4','B' */
    u8 enabled;
    u8 rsvd[3];
} md400_bt_enable_record_t;

typedef struct {
    u8 magic[4];   /* 'M','D','4','P' */
    u8 mode;
    u8 rsvd[3];
} md400_pedal_mode_record_t;

typedef struct {
    u8 magic[4];   /* 'M','D','4','C' */
    u8 min8;
    u8 max8;
    u8 rsvd[2];
} md400_pedal_cal_record_t;

static void md400_bt_indicator_sync(void)
{
    u8 state = 0;

    if (md400_bt_user_enable) {
        state = trans_comm_connected_handle ? 2 : 1;
    }
    md400_display_bt_state_update(state);
}

static u8 md400_bt_user_enable_load(void)
{
    md400_bt_enable_record_t rec;
    int ret;

    if (!md400_bt_user_loaded) {
        md400_bt_user_loaded = 1;
        ret = syscfg_read(MD400_VM_BT_ENABLE_ID, &rec, sizeof(rec));
        if ((ret == (int)sizeof(rec)) &&
            (rec.magic[0] == 'M') && (rec.magic[1] == 'D') &&
            (rec.magic[2] == '4') && (rec.magic[3] == 'B')) {
            md400_bt_user_enable = rec.enabled ? 1 : 0;
        }
    }
    return md400_bt_user_enable;
}

static void md400_bt_set_user_enable_now(u8 en)
{
    md400_bt_enable_record_t rec;
    int ret;

    en = en ? 1 : 0;
    md400_bt_user_loaded = 1;
    md400_bt_user_enable = en;
    rec.magic[0] = 'M';
    rec.magic[1] = 'D';
    rec.magic[2] = '4';
    rec.magic[3] = 'B';
    rec.enabled = en;
    rec.rsvd[0] = rec.rsvd[1] = rec.rsvd[2] = 0;
    ret = syscfg_write(MD400_VM_BT_ENABLE_ID, &rec, sizeof(rec));
    if (ret != (int)sizeof(rec)) {
        printf("[MD400_BT] enable store fail ret=%d\r\n", ret);
    }
    ble_module_enable(en);
    md400_bt_indicator_sync();
    printf("[MD400_BT] user enable=%u\r\n", en);
}

static void md400_bt_set_user_enable(u8 en)
{
    en = en ? 1 : 0;
    (void)md400_bt_user_enable_load();
    if (md400_bt_user_enable != en) {
        /* Show the edit-page ON/OFF text immediately. VM and the radio stay
         * on the persist task so this can run from an MS32 UART callback. */
        md400_bt_user_enable = en;
        md400_bt_indicator_sync();
    } else if (!md400_persist_in_task) {
        return;
    }
    if (md400_persist_in_task) {
        md400_bt_set_user_enable_now(en);
        return;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_BT_ENABLE, en, 0)) {
        printf("[MD400_PERSIST] BT enable queue failed value=%u\r\n",
               (unsigned)en);
    }
}

u8 md400_bt_uart_get_user_enable(void)
{
    return md400_bt_user_enable_load();
}

static u8 md400_pedal_mode_load(void)
{
    md400_pedal_mode_record_t rec;
    int ret;

    if (!md400_pedal_mode_loaded) {
        md400_pedal_mode_loaded = 1;
        ret = syscfg_read(MD400_VM_PEDAL_MODE_ID, &rec, sizeof(rec));
        if ((ret == (int)sizeof(rec)) &&
            (rec.magic[0] == 'M') && (rec.magic[1] == 'D') &&
            (rec.magic[2] == '4') && (rec.magic[3] == 'P') &&
            (rec.mode <= MD400_PEDAL_MODE_SWT)) {
            md400_pedal_mode = rec.mode;
        }
    }
    return md400_pedal_mode;
}

static void md400_ms32_send_pedal_mode_to_mcu(u8 mode)
{
    u8 frame[8];
    u8 seq;
    u8 payload;
    u8 checksum;
    static u8 log_count;

    if (!ble_midi_uart_bus) {
        return;
    }

    payload = mode ? MD400_PEDAL_MODE_SWT : MD400_PEDAL_MODE_EXP;
    seq = md400_ms32_tx_sequence++;
    checksum = MD400_MS32_PROTO_VER ^ MD400_MS32_TYPE_PEDAL_MODE ^
               seq ^ 1 ^ payload;

    frame[0] = MD400_MS32_PROTO_HEAD0;
    frame[1] = MD400_MS32_PROTO_HEAD1;
    frame[2] = MD400_MS32_PROTO_VER;
    frame[3] = MD400_MS32_TYPE_PEDAL_MODE;
    frame[4] = seq;
    frame[5] = 1;
    frame[6] = payload;
    frame[7] = checksum;

    ble_midi_uart_bus->write(frame, sizeof(frame));

    if (log_count < 32) {
        printf("[MD400_MS32] pedal mode tx=%s\r\n", payload ? "SWT" : "EXP");
    }
    log_count++;
}

static u32 md400_ms32_pedal_mode_ack_count_snapshot(u8 *mode)
{
    u32 count = 0;
    u8 ack_mode = 0x7f;

    md400_ms32_get_pedal_mode_ack_snapshot(&ack_mode, &count, NULL);
    if (mode) {
        *mode = ack_mode;
    }
    return count;
}

static void md400_ms32_pedal_mode_sync_tick(void)
{
    u32 now;
    u32 interval;
    u32 ack_count;
    u8 ack_mode;

    if (!md400_ms32_mode_sync_active || !ble_midi_uart_bus) {
        return;
    }

    ack_count = md400_ms32_pedal_mode_ack_count_snapshot(&ack_mode);
    if ((ack_count > md400_ms32_mode_sync_start_ack_count) &&
        (ack_mode == md400_ms32_mode_sync_target)) {
        md400_ms32_mode_sync_active = 0;
        return;
    }

    now = sys_timer_get_ms();
    if (md400_ms32_mode_sync_retry_count >= MD400_MS32_MODE_SYNC_RETRY_MAX) {
        if (!md400_ms32_mode_sync_timeout_logged) {
            md400_ms32_mode_sync_timeout_logged = 1;
            printf("[MD400_MS32] pedal mode sync timeout target=%s ack=%s count=%lu\r\n",
                   md400_ms32_mode_sync_target ? "SWT" : "EXP",
                   (ack_mode == 0x7f) ? "none" : (ack_mode ? "SWT" : "EXP"),
                   (unsigned long)ack_count);
        }
        interval = MD400_MS32_MODE_SYNC_BACKOFF_MS;
    } else {
        interval = MD400_MS32_MODE_SYNC_RETRY_MS;
    }

    if (md400_ms32_mode_sync_last_ms &&
        ((now - md400_ms32_mode_sync_last_ms) < interval)) {
        return;
    }

    md400_ms32_mode_sync_last_ms = now ? now : 1;
    if (md400_ms32_mode_sync_retry_count < MD400_MS32_MODE_SYNC_RETRY_MAX) {
        md400_ms32_mode_sync_retry_count++;
    }
    md400_ms32_send_pedal_mode_to_mcu(md400_ms32_mode_sync_target);
}

static void md400_ms32_request_pedal_mode_sync(u8 mode)
{
    mode = mode ? MD400_PEDAL_MODE_SWT : MD400_PEDAL_MODE_EXP;
    md400_ms32_mode_sync_target = mode;
    md400_ms32_mode_sync_start_ack_count = md400_ms32_pedal_mode_ack_count_snapshot(NULL);
    md400_ms32_mode_sync_retry_count = 0;
    md400_ms32_mode_sync_timeout_logged = 0;
    md400_ms32_mode_sync_last_ms = 0;
    md400_ms32_mode_sync_active = 1;
    md400_ms32_pedal_mode_sync_tick();
}

static void md400_pedal_mode_apply_ram(u8 mode)
{
    /*
     * A maximum EXP sample may already have armed the delayed pedal-edit
     * entry. Do not let that stale sample enter the page after EXP/SWT has
     * changed. The new mode must observe its own neutral/released state before
     * a later maximum/press is accepted as an intentional hold gesture.
     */
    md400_pedal_settings_enter_reset();
    md400_pedal_mode = mode;
    md400_pedal_switch_seen = 0;
    md400_pedal_switch_state = 0;
    md400_pedal_display_seen = 0;
}

static void md400_pedal_mode_set_now(u8 mode)
{
    md400_pedal_mode_record_t rec;
    int ret;

    mode = mode ? MD400_PEDAL_MODE_SWT : MD400_PEDAL_MODE_EXP;
    if (md400_pedal_mode_load() != mode) {
        md400_pedal_mode_apply_ram(mode);
    }
    rec.magic[0] = 'M';
    rec.magic[1] = 'D';
    rec.magic[2] = '4';
    rec.magic[3] = 'P';
    rec.mode = mode;
    rec.rsvd[0] = rec.rsvd[1] = rec.rsvd[2] = 0;
    ret = syscfg_write(MD400_VM_PEDAL_MODE_ID, &rec, sizeof(rec));
    if (ret != (int)sizeof(rec)) {
        printf("[MD400_PEDAL] mode store fail ret=%d\r\n", ret);
    }
    printf("[MD400_PEDAL] mode=%s\r\n", mode ? "SWT" : "EXP");
    md400_ms32_request_pedal_mode_sync(mode);
}

static void md400_pedal_mode_set(u8 mode)
{
    mode = mode ? MD400_PEDAL_MODE_SWT : MD400_PEDAL_MODE_EXP;
    (void)md400_pedal_mode_load();
    if (md400_pedal_mode != mode) {
        /* Keep the SWT/EXP card in lock-step with the knob. Flash and the
         * MS32 mode command still run on the persist task. */
        md400_pedal_mode_apply_ram(mode);
    } else if (!md400_persist_in_task) {
        return;
    }
    if (md400_persist_in_task) {
        md400_pedal_mode_set_now(mode);
        return;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_PEDAL_MODE, mode, 0)) {
        printf("[MD400_PERSIST] pedal mode queue failed value=%u\r\n",
               (unsigned)mode);
    }
}

static u16 md400_bt_uart_enable_timer;
static u16 md400_bt_uart_disconnect_timer;

static void md400_bt_uart_enable_timeout(void *priv)
{
    md400_bt_uart_enable_timer = 0;
    md400_bt_set_user_enable((u8)(u32)priv);
}

void md400_bt_uart_set_user_enable(u8 en)
{
    en = en ? 1 : 0;
    if (md400_bt_uart_enable_timer) {
        sys_timeout_del(md400_bt_uart_enable_timer);
        md400_bt_uart_enable_timer = 0;
    }
    md400_bt_uart_enable_timer = sys_timeout_add((void *)(u32)en,
                                                 md400_bt_uart_enable_timeout,
                                                 10);
}

static void md400_bt_uart_disconnect_timeout(void *priv)
{
    (void)priv;
    md400_bt_uart_disconnect_timer = 0;
    if (trans_con_handle) {
        ble_comm_disconnect(trans_con_handle);
    } else {
        printf("[MD400_BT] no active BLE link\r\n");
    }
}

void md400_bt_uart_disconnect(void)
{
    if (md400_bt_uart_disconnect_timer) {
        return;
    }
    md400_bt_uart_disconnect_timer = sys_timeout_add(NULL,
                                                     md400_bt_uart_disconnect_timeout,
                                                     10);
}

void trans_comm_connected_handle_on_connect(u16 conn_handle)
{
    if (!conn_handle) {
        return;
    }
    if (trans_comm_connected_handle && (trans_comm_connected_handle != conn_handle)) {
        ble_comm_disconnect(conn_handle);
        return;
    }
    if (trans_comm_connected_handle != conn_handle) {
        ble_midi_connect_count++;
    }
    trans_comm_connected_handle = conn_handle;
    md400_bt_indicator_sync();
#if defined(TCFG_BT_STATUS_LED_ENABLE) && (TCFG_BT_STATUS_LED_ENABLE)
    // 修改日期: 2026-02-10, 修改说明: 指示灯跟随统一连接状态(client/server任一连接都常亮)
    bt_status_led_set_connected(1);
#endif
#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
    trans_role_on_connected();
#endif
}

void trans_comm_connected_handle_on_disconnect(u16 conn_handle)
{
    if (!conn_handle) {
        return;
    }
    if (trans_comm_connected_handle == conn_handle) {
        trans_comm_connected_handle = 0;
        ble_midi_disconnect_count++;
        md400_bt_indicator_sync();
#if defined(TCFG_BT_STATUS_LED_ENABLE) && (TCFG_BT_STATUS_LED_ENABLE)
        // 修改日期: 2026-02-10, 修改说明: 指示灯跟随统一连接状态(client/server都断开后1Hz闪烁)
        bt_status_led_set_connected(0);
#endif
#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
        trans_role_on_disconnected();
#endif
    }
}

/*
 * 修改日期: 2026-02-16
 * 修改说明: 循环buffer used/free计算与读写(不预留1字节，按功能规划.md)
 */
static u16 midi_ring_used(u16 rp, u16 wp, u16 size, u8 full)
{
    if (full) {
        return size;
    }
    if (rp < wp) {
        return wp - rp;
    }
    if (rp > wp) {
        return size - rp + wp;
    }
    return 0;
}

static u16 midi_ring_free(u16 rp, u16 wp, u16 size, u8 full)
{
    return size - midi_ring_used(rp, wp, size, full);
}

static u16 midi_ring_peek_contig(u16 rp, u16 wp, u16 size, u8 full)
{
    u16 used = midi_ring_used(rp, wp, size, full);
    if (!used) {
        return 0;
    }
    if (full || (rp > wp)) {
        return size - rp;
    }
    return wp - rp;
}

static u16 midi_ring_peek_copy(u8 *dst, u16 dst_max, u8 *src, u16 rp, u16 wp, u16 size, u8 full)
{
    u16 used = midi_ring_used(rp, wp, size, full);
    u16 first;

    if (!dst || !dst_max || !used) {
        return 0;
    }
    if (used > dst_max) {
        used = dst_max;
    }
    first = size - rp;
    if (first >= used) {
        memcpy(dst, &src[rp], used);
    } else {
        memcpy(dst, &src[rp], first);
        memcpy(&dst[first], &src[0], used - first);
    }
    return used;
}

static void midi_ring_consume(u16 size, u16 *rp, u8 *full, u16 len)
{
    if (!len) {
        return;
    }
    if (len >= size) {
        *rp = 0;
        *full = 0;
        return;
    }
    *rp += len;
    if (*rp >= size) {
        *rp -= size;
    }
    *full = 0;
}

static int midi_ring_write_drop_on_full(u8 *buf, u16 size, u16 *rp, u16 *wp, u8 *full, const u8 *data, u16 len)
{
    u16 first;

    if (!data || !len) {
        return -1;
    }

    local_irq_disable();
    if (midi_ring_free(*rp, *wp, size, *full) < len) {
        local_irq_enable();
        return -1;
    }

    first = size - *wp;
    if (first >= len) {
        memcpy(&buf[*wp], data, len);
        *wp += len;
        if (*wp >= size) {
            *wp = 0;
        }
    } else {
        memcpy(&buf[*wp], data, first);
        memcpy(&buf[0], &data[first], len - first);
        *wp = len - first;
    }

    if (*wp == *rp) {
        *full = 1;
    }
    local_irq_enable();
    return 0;
}

// 修改日期: 2026-02-16, 修改说明: 6个MIDI循环buffer写入(空间不足整包丢弃，按功能规划.md)
static int ble_midi_rx_write_drop_on_full(const u8 *data, u16 len)
{
    int ret = midi_ring_write_drop_on_full(ble_midi_rx_buf, BLE_MIDI_RX_BUFF_SIZE, &ble_midi_rx_buf_rp, &ble_midi_rx_buf_wp, &ble_midi_rx_full, data, len);
    if (ret) {
        ble_midi_rx_dropped_bytes += len;
    } else {
        ble_midi_rx_queued_bytes += len;
    }
    return ret;
}

static int ble_midi_tx_write_drop_on_full(const u8 *data, u16 len)
{
    int ret = midi_ring_write_drop_on_full(ble_midi_tx_buf, BLE_MIDI_TX_BUFF_SIZE, &ble_midi_tx_buf_rp, &ble_midi_tx_buf_wp, &ble_midi_tx_full, data, len);
    if (ret) {
        ble_midi_tx_dropped_bytes += len;
    } else {
        ble_midi_tx_queued_bytes += len;
    }
    return ret;
}

static int usb_midi_rx_write_drop_on_full(const u8 *data, u16 len)
{
    return midi_ring_write_drop_on_full(usb_midi_rx_buf, USB_MIDI_RX_BUFF_SIZE, &usb_midi_rx_buf_rp, &usb_midi_rx_buf_wp, &usb_midi_rx_full, data, len);
}

static int usb_midi_tx_write_drop_on_full(const u8 *data, u16 len)
{
#if CONFIG_MD400_ENCODER_CC_ONLY_TEST
    static u32 blocked_cnt;

    if (blocked_cnt < 24) {
        printf("[MD400_DIAG] block raw usb tx#%lu len=%u b0=%02x b1=%02x b2=%02x\r\n",
               (unsigned long)blocked_cnt + 1, (unsigned)len,
               (unsigned)(data && len ? data[0] : 0),
               (unsigned)(data && (len > 1) ? data[1] : 0),
               (unsigned)(data && (len > 2) ? data[2] : 0));
    } else if (blocked_cnt == 24) {
        printf("[MD400_DIAG] raw usb tx block log muted\r\n");
    }
    blocked_cnt++;
    return -1;
#else
    // 修改日期: 2026-02-16, 修改说明: 增加USB MIDI TX缓存写入失败打印, 排查BLE->USB路由丢包
    static u32 usb_midi_tx_drop_cnt;
    // 修改日期: 2026-02-16, 修改说明: 增加USB MIDI TX缓存写入成功打印(节流), 确认BLE数据是否真正进入USB TX缓存
    static u32 usb_midi_tx_write_cnt;
    u16 used_after;
    int ret = midi_ring_write_drop_on_full(usb_midi_tx_buf, USB_MIDI_TX_BUFF_SIZE, &usb_midi_tx_buf_rp, &usb_midi_tx_buf_wp, &usb_midi_tx_full, data, len);
    if (ret) {
        usb_midi_tx_drop_cnt++;
        // if (usb_midi_tx_drop_cnt <= 20) {
        //     printf("[USB_MIDI] txbuf drop#%lu len=%u\r\n", (unsigned long)usb_midi_tx_drop_cnt, (unsigned)len);
        // } else if (usb_midi_tx_drop_cnt == 21) {
        //     printf("[USB_MIDI] txbuf drop log muted\r\n");
        // }
    } else {
        usb_midi_tx_write_cnt++;
        // if (usb_midi_tx_write_cnt <= 20) {
        //     local_irq_disable();
        //     used_after = midi_ring_used(usb_midi_tx_buf_rp, usb_midi_tx_buf_wp, USB_MIDI_TX_BUFF_SIZE, usb_midi_tx_full);
        //     local_irq_enable();
        //     printf("[USB_MIDI] txbuf write#%lu len=%u used=%u b0=%02x\r\n",
        //            (unsigned long)usb_midi_tx_write_cnt, (unsigned)len, (unsigned)used_after,
        //            (unsigned)(data ? data[0] : 0));
        // } else if (usb_midi_tx_write_cnt == 21) {
        //     printf("[USB_MIDI] txbuf write log muted\r\n");
        // }
    }
    return ret;
#endif
}

static int uart_midi_rx_write_drop_on_full(const u8 *data, u16 len)
{
    return midi_ring_write_drop_on_full(uart_midi_rx_buf, UART_MIDI_RX_BUFF_SIZE, &uart_midi_rx_buf_rp, &uart_midi_rx_buf_wp, &uart_midi_rx_full, data, len);
}

static int uart_midi_tx_write_drop_on_full(const u8 *data, u16 len)
{
    int ret = midi_ring_write_drop_on_full(uart_midi_tx_buf, UART_MIDI_TX_BUFF_SIZE, &uart_midi_tx_buf_rp, &uart_midi_tx_buf_wp, &uart_midi_tx_full, data, len);
#if CONFIG_MD400_MIDI_OUT_ENABLE
    if (ret) {
        md400_midi_out_dropped_bytes += len;
    } else {
        md400_midi_out_queued_bytes += len;
        if (data && len) {
            u16 i;
            for (i = 0; i < len; i++) {
                if (md400_midi_out_trace_len < MD400_MIDI_OUT_TRACE_SIZE) {
                    md400_midi_out_trace[md400_midi_out_trace_len++] = data[i];
                } else {
                    memmove(md400_midi_out_trace,
                            &md400_midi_out_trace[1],
                            MD400_MIDI_OUT_TRACE_SIZE - 1);
                    md400_midi_out_trace[MD400_MIDI_OUT_TRACE_SIZE - 1] = data[i];
                }
            }
        }
    }
#endif
    return ret;
}

// 修改日期: 2026-02-18, 修改说明: USB1 MIDI buffer写入函数(空间不足整包丢弃，按功能规划.md)
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
static int usb1_midi_rx_write_drop_on_full(const u8 *data, u16 len)
{
    int ret = midi_ring_write_drop_on_full(usb1_midi_rx_buf, USB1_MIDI_RX_BUFF_SIZE, &usb1_midi_rx_buf_rp, &usb1_midi_rx_buf_wp, &usb1_midi_rx_full, data, len);
    if (ret) {
        usb1_midi_rx_dropped_bytes += len;
    } else {
        usb1_midi_rx_queued_bytes += len;
    }
    return ret;
}

static int usb1_midi_tx_write_drop_on_full(const u8 *data, u16 len)
{
    int ret = midi_ring_write_drop_on_full(usb1_midi_tx_buf, USB1_MIDI_TX_BUFF_SIZE, &usb1_midi_tx_buf_rp, &usb1_midi_tx_buf_wp, &usb1_midi_tx_full, data, len);
    if (ret) {
        usb1_midi_tx_dropped_bytes += len;
    } else {
        usb1_midi_tx_queued_bytes += len;
    }
    return ret;
}

// 修改日期: 2026-02-18, 修改说明: USB1 MIDI RX写入接口(供app_spp_and_le.c调用)
int usb1_midi_rx_write(const u8 *data, u16 len)
{
    return usb1_midi_rx_write_drop_on_full(data, len);
}

/*
 * 修改日期: 2026-02-18
 * 修改说明: USB1 MIDI Host接收解码(USB数据包解码为标准MIDI字节流，参考ac632n_usb_midi_rx)
 */
void usb1_midi_host_rx_decode(const u8 *packet, u32 len)
{
    static const u8 cin_len[16] = {0, 1, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1};
    u8 raw[64];
    u16 raw_len = 0;
    u32 i;

    if (!packet || (len < 4)) {
        return;
    }

    usb1_midi_rx_packet_bytes += len;

    for (i = 0; (i + 3) < len; i += 4) {
        u8 cin = packet[i] & 0x0F;
        u8 n = cin_len[cin];
        u8 k;

        if (!n) {
            continue;
        }
        if ((raw_len + n) > sizeof(raw)) {
            break;
        }
        for (k = 0; k < n; k++) {
            raw[raw_len++] = packet[i + 1 + k];
        }
    }

    if (raw_len) {
        usb1_midi_rx_write_drop_on_full(raw, raw_len);
    }
}

/*
 * 修改日期: 2026-02-18
 * 修改说明: USB1 MIDI Host发送编码(标准MIDI字节流编码为USB数据包，参考ac632n_usb_midi_tx)
 */
static u8 usb1_midi_tx_running_status;
static u8 usb1_midi_tx_sysex_active;

static u8 usb1_midi_msg_total_len(u8 status)
{
    if ((status >= 0x80) && (status <= 0xEF)) {
        return (((status & 0xF0) == 0xC0) || ((status & 0xF0) == 0xD0)) ? 2 : 3;
    }
    switch (status) {
    case 0xF1:
    case 0xF3:
        return 2;
    case 0xF2:
        return 3;
    case 0xF6:
    case 0xF7:
    case 0xF8:
    case 0xFA:
    case 0xFB:
    case 0xFC:
    case 0xFE:
    case 0xFF:
        return 1;
    default:
        return 1;
    }
}

static u8 usb1_midi_cin_from_status(u8 status)
{
    if ((status >= 0x80) && (status <= 0xEF)) {
        return (status >> 4) & 0x0F;
    }
    switch (status) {
    case 0xF1:
    case 0xF3:
        return 0x2;
    case 0xF2:
        return 0x3;
    case 0xF6:
    case 0xF7:
        return 0x5;
    default:
        if (status >= 0xF8) {
            return 0xF;
        }
        return 0xF;
    }
}

u16 usb1_midi_host_tx_encode(u8 *packet, u32 max_len)
{
    u16 out_len = 0;
    u16 consumed = 0;
    u8 raw[96];
    u16 raw_avail;

    if (!packet || !max_len) {
        return 0;
    }

    local_irq_disable();
    raw_avail = midi_ring_peek_copy(raw, sizeof(raw), usb1_midi_tx_buf, usb1_midi_tx_buf_rp, usb1_midi_tx_buf_wp, USB1_MIDI_TX_BUFF_SIZE, usb1_midi_tx_full);
    local_irq_enable();

    while (((out_len + 4) <= max_len) && (consumed < raw_avail)) {
        u16 remain = raw_avail - consumed;
        u8 b;

        if (!remain) {
            break;
        }

        b = raw[consumed];

        if (usb1_midi_tx_sysex_active) {
            u16 take = (remain >= 3) ? 3 : remain;
            u16 k;

            /* A continuing SysEx packet must contain three bytes. Retain a
             * partial group until more data arrives; only F7 permits padding. */
            for (k = 0; k < take; k++) {
                if (raw[consumed + k] == 0xF7) {
                    break;
                }
            }
            if (k < take) {
                packet[out_len++] = 0x5 + k;
                packet[out_len++] = raw[consumed];
                packet[out_len++] = (k >= 1) ? raw[consumed + 1] : 0;
                packet[out_len++] = (k >= 2) ? raw[consumed + 2] : 0;
                consumed += k + 1;
                usb1_midi_tx_sysex_active = 0;
                continue;
            }
            if (remain < 3) {
                break;
            }
            packet[out_len++] = 0x4;
            packet[out_len++] = raw[consumed];
            packet[out_len++] = raw[consumed + 1];
            packet[out_len++] = raw[consumed + 2];
            consumed += 3;
            continue;
        }

        if (b >= 0x80) {
            u8 total = usb1_midi_msg_total_len(b);
            u8 cin = usb1_midi_cin_from_status(b);
            u8 k;

            if (b == 0xF0) {
                usb1_midi_tx_sysex_active = 1;
                usb1_midi_tx_running_status = 0;
                /* Encode F0 with the first SysEx group, never as CIN 0xF. */
                continue;
            }
            /* Routing and queue boundaries need not align with MIDI messages.
             * Never pad an incomplete message or advance beyond the peek. */
            if (total > remain) {
                break;
            }
            if (b < 0xF0) {
                usb1_midi_tx_running_status = b;
            } else if (b < 0xF8) {
                usb1_midi_tx_running_status = 0;
            }

            packet[out_len] = cin;
            packet[out_len + 1] = b;
            for (k = 1; k < total; k++) {
                if ((consumed + k) < raw_avail) {
                    packet[out_len + 1 + k] = raw[consumed + k];
                } else {
                    packet[out_len + 1 + k] = 0;
                }
            }
            for (k = total; k < 3; k++) {
                packet[out_len + 1 + k] = 0;
            }
            consumed += total;
            out_len += 4;
        } else {
            if (usb1_midi_tx_running_status) {
                u8 status = usb1_midi_tx_running_status;
                u8 total = usb1_midi_msg_total_len(status);
                u8 cin = usb1_midi_cin_from_status(status);
                u8 k;

                if ((total - 1) > remain) {
                    break;
                }
                packet[out_len] = cin;
                packet[out_len + 1] = status;
                packet[out_len + 2] = b;
                for (k = 2; k < total; k++) {
                    if ((consumed + k - 1) < raw_avail) {
                        packet[out_len + 1 + k] = raw[consumed + k - 1];
                    } else {
                        packet[out_len + 1 + k] = 0;
                    }
                }
                for (k = total; k < 3; k++) {
                    packet[out_len + 1 + k] = 0;
                }
                consumed += (total - 1);
                out_len += 4;
            } else {
                consumed++;
            }
        }
    }

    if (consumed) {
        local_irq_disable();
        midi_ring_consume(USB1_MIDI_TX_BUFF_SIZE, &usb1_midi_tx_buf_rp, &usb1_midi_tx_full, consumed);
        local_irq_enable();
        usb1_midi_tx_written_bytes += consumed;
        usb1_midi_tx_packet_bytes += out_len;
    }

    return out_len;
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: UART<->BLE MIDI桥接UART中断回调(仅用于唤醒/扩展，当前由轮询泵读取数据)
 */
#endif
static void ble_midi_uart_isr_cb(void *ut_bus, u32 status)
{
    (void)ut_bus;
    (void)status;
}

static void uart_midi_uart_rx_collect_pump(void)
{
    uart_bus_t *ut;
    u32 avail;
    u8 tmp[32];

    if (!ble_midi_uart_bus) {
        return;
    }

    ut = (uart_bus_t *)ble_midi_uart_bus;
    while (1) {
        local_irq_disable();
        avail = ut->kfifo.buf_in - ut->kfifo.buf_out;
        local_irq_enable();

        if (!avail) {
            return;
        }

        if (avail > sizeof(tmp)) {
            avail = sizeof(tmp);
        }

        if (!ble_midi_uart_bus->read(tmp, avail, 1)) {
            return;
        }

        md400_ms32_rx_bytes(tmp, (u16)avail);
    }
}

/*
 * 修改日期: 2026-02-16
 * 修改说明: UART MIDI TX发送(检测uart_midi_tx_buf并通过UART TX发送出去，按功能规划.md)
 */
static void uart_midi_uart_tx_send_pump(void)
{
    const uart_bus_t *tx_bus;
    u16 used;
    u16 send_len;
    u16 first;
    u8 tmp[32];

#if CONFIG_MD400_MIDI_OUT_ENABLE
    tx_bus = md400_midi_out_uart_bus;
#else
    tx_bus = ble_midi_uart_bus;
#endif
    if (!tx_bus) {
        return;
    }

    while (1) {
        local_irq_disable();
        used = midi_ring_used(uart_midi_tx_buf_rp, uart_midi_tx_buf_wp, UART_MIDI_TX_BUFF_SIZE, uart_midi_tx_full);
        if (!used) {
            local_irq_enable();
            return;
        }
        first = midi_ring_peek_contig(uart_midi_tx_buf_rp, uart_midi_tx_buf_wp, UART_MIDI_TX_BUFF_SIZE, uart_midi_tx_full);
        send_len = (used < first) ? used : first;
        if (send_len > sizeof(tmp)) {
            send_len = sizeof(tmp);
        }
        memcpy(tmp, &uart_midi_tx_buf[uart_midi_tx_buf_rp], send_len);
        local_irq_enable();

        tx_bus->write(tmp, send_len);
#if CONFIG_MD400_MIDI_OUT_ENABLE
        md400_midi_out_written_bytes += send_len;
#endif

        local_irq_disable();
        midi_ring_consume(UART_MIDI_TX_BUFF_SIZE, &uart_midi_tx_buf_rp, &uart_midi_tx_full, send_len);
        local_irq_enable();
    }
}

/*
 * 修改日期: 2026-02-18
 * 修改说明: USB1 MIDI TX发送(检测usb1_midi_tx_buf，编码为USB数据包后通过USB1 Host发送出去，按功能规划.md)
 */
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
static void usb1_midi_tx_send_pump(void)
{
    u8 packet[64];
    u16 out_len;
    int ret;

    if (!usb1_host_mounted) {
        return;
    }

    out_len = usb1_midi_host_tx_encode(packet, sizeof(packet));
    if (out_len > 0) {
        ret = usb_midi_host_send(1, packet, out_len);
        if (ret < 0) {
            usb1_midi_tx_send_failures++;
        }
        md400_usb1_host_note_send_result(ret);
    }
}
#endif

/*
 * 修改日期: 2026-02-16
 * 修改说明: 三路RX->两路TX分发(对应TX空间不足时仅跳过该TX写入，不影响其他TX，按功能规划.md)
 */
static void midi_route_one_rx_to_two_tx(u8 *rx_buf, u16 rx_size, u16 *rx_rp, u16 *rx_wp, u8 *rx_full,
                                       int (*tx1_write)(const u8 *data, u16 len),
                                       int (*tx2_write)(const u8 *data, u16 len))
{
    u16 used;
    u16 contig;
    u16 chunk;
    u8 tmp[32];

    while (1) {
        local_irq_disable();
        used = midi_ring_used(*rx_rp, *rx_wp, rx_size, *rx_full);
        if (!used) {
            local_irq_enable();
            return;
        }
        contig = midi_ring_peek_contig(*rx_rp, *rx_wp, rx_size, *rx_full);
        chunk = (used < contig) ? used : contig;
        if (chunk > sizeof(tmp)) {
            chunk = sizeof(tmp);
        }
        memcpy(tmp, &rx_buf[*rx_rp], chunk);
        midi_ring_consume(rx_size, rx_rp, rx_full, chunk);
        local_irq_enable();

        if (tx1_write) {
            tx1_write(tmp, chunk);
        }
        if (tx2_write) {
            tx2_write(tmp, chunk);
        }
    }
}

// 修改日期: 2026-02-18, 修改说明: 四路RX->三路TX分发(对应TX空间不足时仅跳过该TX写入，不影响其他TX，按功能规划.md)
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
static void midi_route_one_rx_to_three_tx(u8 *rx_buf, u16 rx_size, u16 *rx_rp, u16 *rx_wp, u8 *rx_full,
                                          int (*tx1_write)(const u8 *data, u16 len),
                                          int (*tx2_write)(const u8 *data, u16 len),
                                          int (*tx3_write)(const u8 *data, u16 len))
{
    u16 used;
    u16 contig;
    u16 chunk;
    u8 tmp[32];

    while (1) {
        local_irq_disable();
        used = midi_ring_used(*rx_rp, *rx_wp, rx_size, *rx_full);
        if (!used) {
            local_irq_enable();
            return;
        }
        contig = midi_ring_peek_contig(*rx_rp, *rx_wp, rx_size, *rx_full);
        chunk = (used < contig) ? used : contig;
        if (chunk > sizeof(tmp)) {
            chunk = sizeof(tmp);
        }
        memcpy(tmp, &rx_buf[*rx_rp], chunk);
        midi_ring_consume(rx_size, rx_rp, rx_full, chunk);
        local_irq_enable();

        if (tx1_write) {
            tx1_write(tmp, chunk);
        }
        if (tx2_write) {
            tx2_write(tmp, chunk);
        }
        if (tx3_write) {
            tx3_write(tmp, chunk);
        }
    }
}
#endif

static void midi_route_pump(void)
{
    // 修改日期: 2026-02-16, 修改说明: 增加路由关键打印, 确认BLE RX是否进入USB TX缓存
    static u32 midi_route_dbg_cnt;
    // 修改日期: 2026-02-16, 修改说明: 路由打印只在出现有效数据时计数, 避免运行一段时间后被静音
    static u32 midi_route_data_evt_cnt;
    u16 ble_rx_used;
    u16 usb_tx_used;

    local_irq_disable();
    ble_rx_used = midi_ring_used(ble_midi_rx_buf_rp, ble_midi_rx_buf_wp, BLE_MIDI_RX_BUFF_SIZE, ble_midi_rx_full);
    usb_tx_used = midi_ring_used(usb_midi_tx_buf_rp, usb_midi_tx_buf_wp, USB_MIDI_TX_BUFF_SIZE, usb_midi_tx_full);
    local_irq_enable();
    midi_route_dbg_cnt++;
    if (0) { // 修改日期: 2026-02-17, 修改说明: 屏蔽[MIDI_ROUTE]日志输出
        if (ble_rx_used || usb_tx_used) {
            midi_route_data_evt_cnt++;
            if (midi_route_data_evt_cnt <= 50) {
                printf("[MIDI_ROUTE] evt#%lu tick#%lu ble_rx_used=%u usb_tx_used=%u\r\n",
                       (unsigned long)midi_route_data_evt_cnt, (unsigned long)midi_route_dbg_cnt,
                       (unsigned)ble_rx_used, (unsigned)usb_tx_used);
            } else if (midi_route_data_evt_cnt == 51) {
                printf("[MIDI_ROUTE] log muted\r\n");
            }
        }
    }

    // PB2/PB3 are reserved for the MS32 framed encoder UART on MD400.
#if CONFIG_MD400_ENCODER_CC_ONLY_TEST
    return;
#endif
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    midi_route_one_rx_to_two_tx(ble_midi_rx_buf, BLE_MIDI_RX_BUFF_SIZE, &ble_midi_rx_buf_rp, &ble_midi_rx_buf_wp, &ble_midi_rx_full,
                                usb_midi_tx_write_drop_on_full, usb1_midi_tx_write_drop_on_full);
    midi_route_one_rx_to_two_tx(usb_midi_rx_buf, USB_MIDI_RX_BUFF_SIZE, &usb_midi_rx_buf_rp, &usb_midi_rx_buf_wp, &usb_midi_rx_full,
                                ble_midi_tx_write_drop_on_full, usb1_midi_tx_write_drop_on_full);
    midi_route_one_rx_to_two_tx(usb1_midi_rx_buf, USB1_MIDI_RX_BUFF_SIZE, &usb1_midi_rx_buf_rp, &usb1_midi_rx_buf_wp, &usb1_midi_rx_full,
                                usb_midi_tx_write_drop_on_full, ble_midi_tx_write_drop_on_full);
#else
    midi_route_one_rx_to_two_tx(ble_midi_rx_buf, BLE_MIDI_RX_BUFF_SIZE, &ble_midi_rx_buf_rp, &ble_midi_rx_buf_wp, &ble_midi_rx_full,
                                usb_midi_tx_write_drop_on_full, NULL);
    midi_route_one_rx_to_two_tx(usb_midi_rx_buf, USB_MIDI_RX_BUFF_SIZE, &usb_midi_rx_buf_rp, &usb_midi_rx_buf_wp, &usb_midi_rx_full,
                                ble_midi_tx_write_drop_on_full, NULL);
#endif
}

/*
 * 修改日期: 2026-02-16
 * 修改说明: 大循环泵(采集UART RX、四路RX->三路TX分发、UART TX发送、USB1 TX发送，按功能规划.md)
 */
static void ble_midi_uart_bridge_pump_tick(void)
{
    static u32 pump_tick_cnt = 0;
    pump_tick_cnt++;

    if (0) {
        if (pump_tick_cnt <= 10) {
            printf("[PUMP] tick#%lu\r\n", (unsigned long)pump_tick_cnt);
        } else if (pump_tick_cnt == 11) {
            printf("[PUMP] tick log muted\r\n");
        }
    }

#if CONFIG_MD400_MS32_ENCODER_ENABLE || CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
    uart_midi_uart_rx_collect_pump();
    md400_ms32_pedal_mode_sync_tick();
#endif
    midi_route_pump();
    ble_midi_tx_send_pump();
    usb_midi_ac632n_kick_tx();
#if CONFIG_MD400_MIDI_OUT_ENABLE || CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
    uart_midi_uart_tx_send_pump();
#endif
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    usb1_midi_tx_send_pump();
#endif

}

/*
 * 修改日期: 2026-06-05
 * 修改说明: 初始化PB2/PB3为MS32编码器/ADC帧UART, 115200 8N1
 */
static void ble_midi_uart_init(void)
{
    struct uart_platform_data_t u_arg = {0};

    if (ble_midi_uart_bus) {
        return;
    }

#if defined(CONFIG_BOARD_AC632N_DEMO)
    u_arg.tx_pin = IO_PORTB_02;
    u_arg.rx_pin = IO_PORTB_03;
#elif defined(AC6321A_BOARD_ONLY) && (AC6321A_BOARD_ONLY)
    u_arg.tx_pin = IO_PORTB_02;
    u_arg.rx_pin = IO_PORTB_03;
#else
    u_arg.tx_pin = IO_PORTA_03;
    u_arg.rx_pin = IO_PORTA_04;
#endif
    u_arg.rx_cbuf = ble_midi_uart_rx_cbuf;
    u_arg.rx_cbuf_size = sizeof(ble_midi_uart_rx_cbuf);
    u_arg.frame_length = 32;
    u_arg.rx_timeout = 4;
    u_arg.isr_cbfun = ble_midi_uart_isr_cb;
    u_arg.baud = 115200;
    u_arg.is_9bit = 0;

    md400_ms32_init();
    ble_midi_uart_bus = uart_dev_open(&u_arg);
    if (ble_midi_uart_bus) {
        printf("[MD400_MS32] uart open ok tx=PB2 rx=PB3 baud=115200\r\n");
        md400_ms32_request_pedal_mode_sync(md400_pedal_mode_load());
    } else {
        printf("[MD400_MS32] uart open failed tx=PB2 rx=PB3 baud=115200\r\n");
    }
}

#if CONFIG_MD400_MIDI_OUT_ENABLE
static void md400_midi_out_uart_init(void)
{
    struct uart_platform_data_t u_arg = {0};

    if (md400_midi_out_uart_bus) {
        return;
    }

    u_arg.tx_pin = IO_PORTB_00;
    u_arg.rx_pin = NO_CONFIG_PORT;
    u_arg.rx_cbuf = NULL;
    u_arg.rx_cbuf_size = 0;
    u_arg.frame_length = 0;
    u_arg.rx_timeout = 0;
    u_arg.isr_cbfun = NULL;
    u_arg.baud = 31250;
    u_arg.is_9bit = 0;

    md400_midi_out_uart_bus = uart_dev_open(&u_arg);
    if (md400_midi_out_uart_bus) {
        printf("[MD400_MIDI_OUT] init tx=PB0 baud=31250\r\n");
    } else {
        md400_midi_out_open_failures++;
        printf("[MD400_MIDI_OUT] init failed tx=PB0 baud=31250\r\n");
    }
}
#endif

/* ble_midi_tx_write_drop_on_full / ble_midi_rx_write_drop_on_full已在前面按功能规划.md实现 */

static md400_ble_decoder_t md400_ble_decoder;
static u16 ble_midi_decode_packet(const u8 *in, u16 in_len, u8 *out, u16 out_max)
{
    return md400_ble_decode(&md400_ble_decoder, in, in_len, out, out_max);
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI编码(为每个MIDI status插入timestamp，简化时间戳为0)
 */
static u16 ble_midi_encode_packet(const u8 *in, u16 in_len, u8 *out, u16 out_max)
{
    u16 i;
    u16 out_len = 0;

    if (!in || !out || !in_len || (out_max < 3)) {
        return 0;
    }

    out[out_len++] = 0x80;
    out[out_len++] = 0x80;

    for (i = 0; i < in_len; i++) {
        u8 b = in[i];
        if (b >= 0x80) {
            if (i) {
                if ((out_len + 1) > out_max) {
                    break;
                }
                out[out_len++] = 0x80;
            }
        }
        if ((out_len + 1) > out_max) {
            break;
        }
        out[out_len++] = b;
    }

    return out_len;
}

/*
 * 修改日期: 2026-02-16
 * 修改说明: USB MIDI收包解码/发包编码(与usb_midi_rx_buf/usb_midi_tx_buf对接，按功能规划.md)
 */
static u8 usb_midi_tx_running_status;
static u8 usb_midi_tx_sysex_active;

#define USB_MIDI_CABLE_CONTROL 0x00
#define USB_MIDI_CABLE_PORT    0x01
#define USB_MIDI_EVENT(cable, cin) ((((cable) & 0x0F) << 4) | ((cin) & 0x0F))

static void md400_usb_midi_tx_stream_clear(void)
{
    local_irq_disable();
    usb_midi_tx_buf_rp = usb_midi_tx_buf_wp;
    usb_midi_tx_full = 0;
    usb_midi_tx_running_status = 0;
    usb_midi_tx_sysex_active = 0;
    local_irq_enable();
}

/* USB reset/close discards the raw encoder state as well as the packet queue. */
void ac632n_usb_midi_tx_reset(void)
{
    md400_usb_midi_tx_stream_clear();
}


enum {
    MD400_MIDI_TYPE_CC = 0,
    MD400_MIDI_TYPE_PC = 1,
    MD400_MIDI_TYPE_NOTE = 2,
    MD400_MIDI_TYPE_PITCH = 3,
    MD400_MIDI_TYPE_PRESSURE = 4,
    MD400_MIDI_TYPE_SYSEX = 5,
    MD400_MIDI_TYPE_OFF = 6,
    MD400_MIDI_TYPE_NOTE_OFF = 7,
    MD400_MIDI_TYPE_CUSTOM = 8,
    MD400_MIDI_TYPE_MAX = MD400_MIDI_TYPE_CUSTOM,
};

static const u8 md400_edit_type_order[] = {
    MD400_MIDI_TYPE_CC,
    MD400_MIDI_TYPE_PC,
    MD400_MIDI_TYPE_NOTE,
    MD400_MIDI_TYPE_NOTE_OFF,
    MD400_MIDI_TYPE_PITCH,
    MD400_MIDI_TYPE_PRESSURE,
    MD400_MIDI_TYPE_SYSEX,
    MD400_MIDI_TYPE_OFF,
};

static u8 md400_edit_type_from_selector(u8 selector)
{
    return md400_edit_type_order[selector % (sizeof(md400_edit_type_order) /
                                            sizeof(md400_edit_type_order[0]))];
}

static u8 md400_edit_selector_from_type(u8 type)
{
    u8 i;

    for (i = 0; i < (sizeof(md400_edit_type_order) /
                    sizeof(md400_edit_type_order[0])); i++) {
        if (md400_edit_type_order[i] == type) {
            return i;
        }
    }

    return 0;
}

enum {
    MD400_TARGET_USB_DEVICE = BIT(0),
    MD400_TARGET_USB_HOST   = BIT(1),
    MD400_TARGET_BLE        = BIT(2),
    MD400_TARGET_MIDI_OUT   = BIT(3),
};

static u8 md400_available_target_mask(void)
{
    u8 mask = MD400_TARGET_USB_DEVICE | MD400_TARGET_BLE;

#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    mask |= MD400_TARGET_USB_HOST;
#endif
#if CONFIG_MD400_MIDI_OUT_ENABLE
    if (md400_midi_out_uart_bus) {
        mask |= MD400_TARGET_MIDI_OUT;
    }
#endif

    return mask;
}

enum {
    MD400_BEHAVIOR_ABSOLUTE = 0,
    MD400_BEHAVIOR_RELATIVE = 1,
    MD400_BEHAVIOR_TOGGLE = MD400_BEHAVIOR_RELATIVE,
    MD400_BEHAVIOR_MOMENTARY = 2,
    MD400_BEHAVIOR_PULSE = 3,
};

enum {
    MD400_LED_MODE_AUTO = 0,
    MD400_LED_MODE_OFF = 1,
    MD400_LED_MODE_MANUAL = 2,
};

enum {
    MD400_UI_MODE_PERFORM = 0,
    MD400_UI_MODE_EDIT = 1,
    MD400_UI_MODE_PLAY = 2,
};

enum {
    MD400_FOOT_MODE_MIDI = 0,
    MD400_FOOT_MODE_PEDAL = 1,
};

enum {
    MD400_PEDAL_EDIT_VIEW_ACTION = 0,
    MD400_PEDAL_EDIT_VIEW_STYLE = 1,
};

enum {
    MD400_EDIT_SUBJECT_ROTATE = 0,
    MD400_EDIT_SUBJECT_SHORT = 1,
    MD400_EDIT_SUBJECT_LONG = 2,
    MD400_EDIT_SUBJECT_DOUBLE = 3,
    MD400_EDIT_SUBJECT_PRESS = 4,
    MD400_EDIT_SUBJECT_COMBO_PRESS = 5,
    MD400_EDIT_SUBJECT_COMBO_SHORT = 6,
    MD400_EDIT_SUBJECT_COMBO_LONG = 7,
    MD400_EDIT_SUBJECT_PEDAL = 8,
    MD400_EDIT_SUBJECT_COUNT,
};

enum {
    MD400_EDIT_PAGE_PRESET = 0,
    MD400_EDIT_PAGE_MESSAGE = 1,
    MD400_EDIT_PAGE_VALUE = 2,
    MD400_EDIT_PAGE_SHORT_MESSAGE = 3,
    MD400_EDIT_PAGE_SHORT_VALUE = 4,
};

#define MD400_EDIT_DEFAULT_SUBJECT MD400_EDIT_SUBJECT_SHORT
#define MD400_EDIT_DISPLAY_SUBJECT_PRESET 9

enum {
    MD400_NB_CMD_PING           = 0x01,
    MD400_NB_CMD_GET_INFO       = 0x02,
    MD400_NB_CMD_GET_STATE      = 0x03,
    MD400_NB_CMD_GET_PRESET     = 0x10,
    MD400_NB_CMD_SET_PRESET     = 0x11,
    MD400_NB_CMD_SAVE_PRESET    = 0x12,
    MD400_NB_CMD_LOAD_PRESET    = 0x13,
    MD400_NB_CMD_GET_PRESET_LABEL = 0x14,
    MD400_NB_CMD_SET_PRESET_LABEL = 0x15,
    MD400_NB_CMD_FACTORY_RESET  = 0x16,
    MD400_NB_CMD_GET_ASSIGNMENT = 0x20,
    MD400_NB_CMD_SET_ASSIGNMENT = 0x21,
    MD400_NB_CMD_GET_GESTURE_ASSIGNMENT = 0x22,
    MD400_NB_CMD_SET_GESTURE_ASSIGNMENT = 0x23,
    MD400_NB_CMD_GET_PEDAL_ASSIGNMENT = 0x24,
    MD400_NB_CMD_SET_PEDAL_ASSIGNMENT = 0x25,
    MD400_NB_CMD_GET_PEDAL_STATE = 0x26,
    MD400_NB_CMD_GET_MIDI_OUT_STATE = 0x27,
    MD400_NB_CMD_GET_USB_HOST_STATE = 0x28,
    MD400_NB_CMD_GET_LED_STATE = 0x29,
    MD400_NB_CMD_GET_EDIT_KEY_STATE = 0x2A,
    MD400_NB_CMD_GET_BLE_MIDI_STATE = 0x2B,
    MD400_NB_CMD_GET_DISPLAY_STATE = 0x2C,
    MD400_NB_CMD_EVENT_INJECT   = 0x30,
    MD400_NB_CMD_GET_EVENT_LOG  = 0x31,
    MD400_NB_CMD_GET_ACTION_STATE = 0x32,
    MD400_NB_CMD_GET_PROTOCOL_STATE = 0x33,
    MD400_NB_CMD_GET_PLAY_LABEL = 0x34,
    MD400_NB_CMD_SET_PLAY_LABEL = 0x35,
    MD400_NB_CMD_GET_SYSEX_TEMPLATE = 0x36,
    MD400_NB_CMD_SET_SYSEX_TEMPLATE = 0x37,
    MD400_NB_CMD_GET_CUSTOM_TEMPLATE = 0x38,
    MD400_NB_CMD_SET_CUSTOM_TEMPLATE = 0x39,
    MD400_NB_CMD_ENTER_EDIT     = 0x40,
    MD400_NB_CMD_EXIT_EDIT      = 0x41,
    MD400_NB_CMD_GET_FOOT_MODE  = 0x42,
    MD400_NB_CMD_SET_FOOT_MODE  = 0x43,
    MD400_NB_CMD_GET_PEDAL_ACTION = 0x44,
    MD400_NB_CMD_SET_PEDAL_ACTION = 0x45,
    MD400_NB_CMD_GET_PEDAL_LABEL = 0x46,
    MD400_NB_CMD_SET_PEDAL_LABEL = 0x47,
    MD400_NB_CMD_VIEW_SNAPSHOT = 0x48,
    MD400_NB_CMD_PRESET_LABELS = 0x49,
    MD400_NB_CMD_SELECT_VIEW = 0x4A,
    MD400_NB_CMD_GET_PEDAL_PRESET_NAME = 0x4B,
    MD400_NB_CMD_SET_PEDAL_PRESET_NAME = 0x4C,
    MD400_NB_CMD_USB_STATE_SUBSCRIBE = 0x4D,
    MD400_NB_CMD_DAW_MODE = 0x4E,
    MD400_NB_CMD_DAW_UPDATE = 0x4F,
    MD400_NB_CMD_DAW_DRAW = 0x50,      /* fire and forget, USB only; see md400_display.c */
    MD400_NB_EVENT_STATE = 0x60,
    MD400_NB_CMD_OTA_BEGIN_BLE  = 0x51,  /* arm pure-BLE JL RCSP OTA boot */
    MD400_NB_CMD_WDT_TEST       = 0x7C,  /* debug build only: force watchdog reset */
    MD400_NB_CMD_DETACH_WEB     = 0x7D,
    MD400_NB_CMD_DETACH_UPDATE  = 0x7E,
    MD400_NB_CMD_RESET          = 0x7F,
};

enum {
    MD400_OTA_REPLY_MODE_SWITCH = 0x01,
    MD400_OTA_REPLY_READY       = 0x02,
};

enum {
    MD400_PRESET_SECTION_ENCODER = 0,
    MD400_PRESET_SECTION_GESTURE = 1,
    MD400_PRESET_SECTION_PEDAL = 2,
    MD400_PRESET_SECTION_LED = 3,
};

enum {
    MD400_NB_STATUS_OK = 0x00,
    MD400_NB_STATUS_BAD_CHECKSUM = 0x01,
    MD400_NB_STATUS_BAD_LENGTH = 0x02,
    MD400_NB_STATUS_BAD_VALUE = 0x03,
    MD400_NB_STATUS_UNSUPPORTED = 0x04,
    MD400_NB_STATUS_BUSY = 0x05,
    MD400_NB_STATUS_FLASH_ERROR = 0x06,
};

enum {
    MD400_EVENT_ENCODER_LEFT = 0x01,
    MD400_EVENT_ENCODER_RIGHT = 0x02,
    MD400_EVENT_ENCODER_PRESS = 0x03,
    MD400_EVENT_ENCODER_RELEASE = 0x04,
    MD400_EVENT_ENCODER_SHORT = 0x05,
    MD400_EVENT_ENCODER_LONG = 0x06,
    MD400_EVENT_ENCODER_DOUBLE = 0x07,
    MD400_EVENT_COMBO_PRESS = 0x08,
    MD400_EVENT_COMBO_SHORT = 0x09,
    MD400_EVENT_COMBO_LONG = 0x0A,
    MD400_EVENT_EDIT_SHORT = 0x10,
    MD400_EVENT_EDIT_LONG = 0x11,
    MD400_EVENT_EDIT_DOUBLE = 0x12,
    MD400_EVENT_PEDAL_VALUE = 0x13,
};

#define MD400_PRESET_COUNT          20
#define MD400_PEDAL_PAGE_COUNT      32
#define MD400_PEDAL_SLOT_COUNT      4
#define MD400_PEDAL_RECORD_PAGE_COUNT 11
#define MD400_PEDAL_RECORD_SLOT_COUNT \
    (MD400_PEDAL_RECORD_PAGE_COUNT * MD400_PEDAL_SLOT_COUNT)
#define MD400_PEDAL_RECORDS_PER_PRESET 3
#define MD400_PEDAL_LABEL_LEN       MD400_DISPLAY_PLAY_LABEL_LEN
#define MD400_PEDAL_STORE_VERSION   3
#define MD400_PEDAL_STORE_VERSION_V2 2
#define MD400_PEDAL_STORE_VERSION_V1 1
#define MD400_PEDAL_HEADER_VERSION  1
#define MD400_VM_PEDAL_BANK_FIRST_ID 128
#define MD400_VM_PEDAL_BANK_HEADER_ID 188
#define MD400_PRESET_LABEL_LEN      8
#define MD400_PLAY_LABEL_LEN        MD400_DISPLAY_PLAY_LABEL_LEN
#define MD400_ENCODER_COUNT         4
#define MD400_ENCODER_GESTURE_COUNT 4
#define MD400_ENCODER_GESTURE_COUNT_V7 3
#define MD400_COMBO_COUNT           3
#define MD400_COMBO_GESTURE_COUNT   3
#define MD400_COMBO_GESTURE_COUNT_V6 2
#define MD400_PRESET_GROUP_SIZE     4
#define MD400_PRESET_GROUP_COUNT    5
#define MD400_PRESET_STORE_VERSION  11
#define MD400_PRESET_STORE_VERSION_V10 10
#define MD400_PRESET_STORE_VERSION_V9 9
#define MD400_PRESET_STORE_VERSION_V8 8
#define MD400_PRESET_STORE_VERSION_V7 7
#define MD400_PRESET_STORE_VERSION_V6 6
#define MD400_PRESET_STORE_VERSION_V5 5
#define MD400_PRESET_STORE_VERSION_V4 4
#define MD400_PRESET_STORE_VERSION_V3 3
#define MD400_ENCODER_PRESS_CC_BASE 65
#define MD400_VM_LEGACY_STORE_ID    41
#define MD400_VM_LEGACY_STORE_ID_B  42
/* 61-127 is the JieLi SDK reserved range (BLE remote DB, BT name/MAC, ...).
 * Labels/templates used to sit at 43/63/83 and collided with those items.
 * Never probe those legacy ranges: even a read can make the SDK interpret a
 * BLE item as one of our large records. New IDs stay in the 128-255 app range,
 * after the pedal bank block. */
#define MD400_VM_PLAY_LABEL_FIRST_ID 189
#define MD400_VM_SYSEX_TEMPLATE_FIRST_ID 209
#define MD400_VM_CUSTOM_TEMPLATE_FIRST_ID 229
#define MD400_VM_PRESET_HEADER_ID   15
#define MD400_VM_PRESET_HEADER_ID_B 16
/* Retained flag (survives cpu_reset via flash) that requests the next boot enter
 * the minimal OTA mode. 103 is OTA, 104 is BT enable, 105 is pedal mode,
 * and 106 is pedal calibration. */
#define MD400_VM_OTA_FLAG_ID        103
#define MD400_EDIT_KEY_PIN          IO_PORTA_02
#define MD400_EDIT_KEY_SCAN_MS      10
#define MD400_EDIT_KEY_DEBOUNCE_MS  30
#define MD400_EDIT_KEY_LONG_MS      650
#define MD400_EDIT_KEY_DOUBLE_MS    350
#define MD400_GESTURE_TICK_MS       20
#define MD400_ENCODER_LONG_MS       900
#define MD400_ENCODER_DOUBLE_MS     350
#define MD400_ENCODER_COMBO_MS      120
#define MD400_EDIT_PRESET_NAV_COMBO_MASK 0x06
/* E1+E2 while the pedal-settings page is showing: leave it for the preset
 * page (the pedal page otherwise captures the edit UI). */
#define MD400_EDIT_PEDAL_EXIT_COMBO_MASK 0x03
#define MD400_NB_MAX_REPLY_PAYLOAD  28
#define MD400_NB_MAX_WIRE_REPLY    112
#define MD400_NB_BLE_FRAME_SIZE    (MD400_NB_MAX_WIRE_REPLY + 10)
#define MD400_NB_STORE_JOB_MAX_PAYLOAD MD400_NB_MAX_REPLY_PAYLOAD
#define MD400_NB_STORE_JOB_DELAY_MS 20
#define MD400_PEDAL_BANK_SAVE_TRIES 3
#define MD400_PEDAL_BANK_SAVE_RETRY_TICKS 3
#define MD400_VM_WRITE_TRIES 3
#define MD400_VM_WRITE_RETRY_TICKS 2
#define MD400_PRESET_LOAD_DEBOUNCE_MS 180
#define MD400_STORE_SAVE_REASON_EDIT 1
#define MD400_STORE_SAVE_REASON_PRESET_SELECT 2
#define MD400_PLAY_LABEL_STORE_VERSION 1
#define MD400_PLAY_LABEL_SLOT_COUNT (MD400_ENCODER_COUNT * 5)
#define MD400_PLAY_LABEL_ROTATE_ACTION 0
#define MD400_PLAY_MSG_INVALID      (-1)
#define MD400_SYSEX_TEMPLATE_STORE_VERSION 1
#define MD400_SYSEX_TEMPLATE_COUNT 4
#define MD400_SYSEX_TEMPLATE_MAX_BODY 128
#define MD400_SYSEX_TEMPLATE_CHUNK 16
#define MD400_SYSEX_TEMPLATE_VALUE_NONE 0x7f
#define MD400_CUSTOM_TEMPLATE_STORE_VERSION 1
#define MD400_CUSTOM_TEMPLATE_COUNT 4
#define MD400_CUSTOM_TEMPLATE_MAX_BODY 128
#define MD400_CUSTOM_TEMPLATE_CHUNK 8
#define MD400_CUSTOM_TEMPLATE_VALUE_NONE 0x7f
#define MD400_FW_INFO_MAJOR         1
#define MD400_FW_INFO_MINOR         0
#define MD400_FW_INFO_PATCH         1

/* NanoBridge version components are raw 7-bit MIDI SysEx data bytes. */
#if MD400_FW_INFO_MAJOR > 127 || MD400_FW_INFO_MINOR > 127 || MD400_FW_INFO_PATCH > 127
#error NEXFOOT firmware version components must fit MIDI SysEx data bytes
#endif
#define MD400_RESET_DELAY_MS        120
#define MD400_UPGRADE_USB_SOFT_KEY  2
#define MD400_SYSEX_DEV_ID          0x7D
#define MD400_SYSEX_VERSION         0x01
#define MD400_INFO_FEATURE_USB_HOST       BIT(0)
#define MD400_INFO_FEATURE_MIDI_OUT       BIT(1)
#define MD400_INFO_FEATURE_LED_STATE      BIT(2)
#define MD400_INFO_FEATURE_UART_DETACH    BIT(3)
#define MD400_INFO_FEATURE_GROUPED_PRESET BIT(4)
#define MD400_INFO_FEATURE_EDIT_KEY_STATE BIT(5)
#define MD400_INFO_FEATURE_BLE_MIDI_STATE BIT(6)
static u8 md400_gesture_feature_enabled(u8 event)
{
#if !CONFIG_MD400_ENCODER_SHORT_PRESS_ENABLE
    if (event == MD400_EVENT_ENCODER_SHORT) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_LONG_PRESS_ENABLE
    if (event == MD400_EVENT_ENCODER_LONG) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_DOUBLE_PRESS_ENABLE
    if (event == MD400_EVENT_ENCODER_DOUBLE) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_COMBO_PRESS_ENABLE
    if ((event == MD400_EVENT_COMBO_PRESS) ||
        (event == MD400_EVENT_COMBO_SHORT) ||
        (event == MD400_EVENT_COMBO_LONG)) {
        return 0;
    }
#endif
    return 1;
}

u8 md400_runtime_preset_store_version(void)
{
    return MD400_PRESET_STORE_VERSION;
}

void md400_runtime_fw_version(u8 *major, u8 *minor, u8 *patch)
{
    if (major) {
        *major = MD400_FW_INFO_MAJOR;
    }
    if (minor) {
        *minor = MD400_FW_INFO_MINOR;
    }
    if (patch) {
        *patch = MD400_FW_INFO_PATCH;
    }
}

u8 md400_runtime_target_mask(void)
{
    return md400_available_target_mask();
}

u8 md400_runtime_feature_flags(void)
{
    u8 flags = MD400_INFO_FEATURE_LED_STATE |
               MD400_INFO_FEATURE_UART_DETACH |
               MD400_INFO_FEATURE_GROUPED_PRESET |
               MD400_INFO_FEATURE_EDIT_KEY_STATE |
               MD400_INFO_FEATURE_BLE_MIDI_STATE;
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    flags |= MD400_INFO_FEATURE_USB_HOST;
#endif
#if CONFIG_MD400_MIDI_OUT_ENABLE
    flags |= MD400_INFO_FEATURE_MIDI_OUT;
#endif
    return flags;
}

typedef struct {
    u8 type;
    u8 channel;
    u8 data1;
    u8 value_min;
    u8 value_max;
    u8 value_default;
    u8 behavior;
    u8 target_mask;
    u8 sysex_id;
} md400_assignment_t;

/* Pedal banks have 44 actions per VM record. JieLi VM's effective item limit
 * includes four bytes of internal overhead, so the old 1021-byte v2 payload
 * could be created but hung when the existing item was rewritten (1025 bytes
 * in the update path). Store color in the three unused high bits of rotate's
 * 48-bit container; the v3 payload is 977 bytes and retains both assignments,
 * color, and the 10-character label. target_mask is intentionally not stored:
 * product assignments always fan out to every available output. */
#define MD400_PEDAL_PACKED_ASSIGNMENT_LEN 6

typedef struct {
    u8 data[MD400_PEDAL_PACKED_ASSIGNMENT_LEN];
} md400_pedal_packed_assignment_t;

static void md400_pedal_assignment_pack(
    md400_pedal_packed_assignment_t *packed,
    const md400_assignment_t *assignment)
{
    u64 bits;
    u8 i;

    if (!packed || !assignment) {
        return;
    }
    bits = ((u64)(assignment->type & 0x0f)) |
           ((u64)(assignment->channel & 0x0f) << 4) |
           ((u64)(assignment->data1 & 0x7f) << 8) |
           ((u64)(assignment->value_min & 0x7f) << 15) |
           ((u64)(assignment->value_max & 0x7f) << 22) |
           ((u64)(assignment->value_default & 0x7f) << 29) |
           ((u64)(assignment->behavior & 0x03) << 36) |
           ((u64)(assignment->sysex_id & 0x7f) << 38);
    for (i = 0; i < MD400_PEDAL_PACKED_ASSIGNMENT_LEN; i++) {
        packed->data[i] = (bits >> (i * 8)) & 0xff;
    }
}

static void md400_pedal_assignment_unpack(
    const md400_pedal_packed_assignment_t *packed,
    md400_assignment_t *assignment)
{
    u64 bits = 0;
    u8 i;

    if (!packed || !assignment) {
        return;
    }
    for (i = 0; i < MD400_PEDAL_PACKED_ASSIGNMENT_LEN; i++) {
        bits |= ((u64)packed->data[i]) << (i * 8);
    }
    memset(assignment, 0, sizeof(*assignment));
    assignment->type = bits & 0x0f;
    assignment->channel = (bits >> 4) & 0x0f;
    assignment->data1 = (bits >> 8) & 0x7f;
    assignment->value_min = (bits >> 15) & 0x7f;
    assignment->value_max = (bits >> 22) & 0x7f;
    assignment->value_default = (bits >> 29) & 0x7f;
    assignment->behavior = (bits >> 36) & 0x03;
    assignment->sysex_id = (bits >> 38) & 0x7f;
    assignment->target_mask = md400_available_target_mask();
}

typedef struct {
    md400_pedal_packed_assignment_t press;
    md400_pedal_packed_assignment_t rotate;
    u8 label[MD400_PEDAL_LABEL_LEN];
} md400_pedal_action_t;

typedef struct {
    md400_pedal_packed_assignment_t press;
    md400_pedal_packed_assignment_t rotate;
    u8 label[MD400_PEDAL_LABEL_LEN];
    u8 color;
} md400_pedal_action_v2_t;

typedef struct {
    md400_assignment_t assignment;
    u8 label[MD400_PEDAL_LABEL_LEN];
    u8 color;
} md400_pedal_action_v1_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 record_index;
    u8 reserved;
    md400_pedal_action_t action[MD400_PEDAL_RECORD_SLOT_COUNT];
    u8 checksum;
} md400_pedal_bank_record_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 record_index;
    u8 reserved;
    md400_pedal_action_v2_t action[MD400_PEDAL_RECORD_SLOT_COUNT];
    u8 checksum;
} md400_pedal_bank_record_v2_t;

/* Only one pedal record version is needed at a time. Sharing the live v3 and
 * v2 migration storage avoids keeping a 977-byte verify record plus a
 * 1021-byte legacy record resident while VM rewrite/defrag needs RAM. */
typedef union {
    md400_pedal_bank_record_t current;
    md400_pedal_bank_record_v2_t v2;
} md400_pedal_bank_storage_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 record_index;
    u8 reserved;
    md400_pedal_action_v1_t action[MD400_PEDAL_RECORD_SLOT_COUNT];
    u8 checksum;
} md400_pedal_bank_record_v1_t;

typedef char md400_pedal_bank_record_must_fit_vm[
    ((sizeof(md400_pedal_bank_record_t) + 4) <= 1024) ? 1 : -1];

static u8 md400_pedal_action_color_get(const md400_pedal_action_t *action)
{
    return action ? ((action->rotate.data[5] >> 5) & 0x07) : 0;
}

static void md400_pedal_action_color_set(md400_pedal_action_t *action,
                                         u8 color)
{
    if (action) {
        action->rotate.data[5] =
            (action->rotate.data[5] & 0x1f) | ((color & 0x07) << 5);
    }
}

typedef struct {
    u8 magic[4];
    u8 version;
    u8 active_preset;
    u8 foot_mode;
    u8 page;
    u8 generation;
    u8 checksum;
} md400_pedal_bank_header_t;

typedef struct {
    u8 label[MD400_PRESET_LABEL_LEN];
    u8 led_mode;
    u8 led_mask;
    md400_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_assignment_t encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT];
    md400_assignment_t combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT];
    md400_assignment_t pedal;
} md400_preset_t;

/* Runtime assignments use nine easy-to-access bytes, but three of those bits
 * are redundant in flash: target_mask is always the compiled product fan-out,
 * and the remaining fields fit in the same six-byte representation already
 * used by the pedal bank. Packing all 30 assignments saves 90 bytes per
 * preset (1.8 KB across 20 presets), which is required because this BD19 VM
 * implementation clamps its two-bank store to 8 KB per bank. */
typedef struct {
    u8 label[MD400_PRESET_LABEL_LEN];
    u8 led_mode;
    u8 led_mask;
    md400_pedal_packed_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_pedal_packed_assignment_t
        encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT];
    md400_pedal_packed_assignment_t
        combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT];
    md400_pedal_packed_assignment_t pedal;
} md400_preset_packed_t;

static void md400_preset_pack(md400_preset_packed_t *packed,
                              const md400_preset_t *preset)
{
    u8 encoder;
    u8 gesture;
    u8 combo;

    if (!packed || !preset) {
        return;
    }
    memcpy(packed->label, preset->label, sizeof(packed->label));
    packed->led_mode = preset->led_mode;
    packed->led_mask = preset->led_mask;
    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        md400_pedal_assignment_pack(&packed->encoder[encoder],
                                    &preset->encoder[encoder]);
        for (gesture = 0; gesture < MD400_ENCODER_GESTURE_COUNT; gesture++) {
            md400_pedal_assignment_pack(
                &packed->encoder_gesture[encoder][gesture],
                &preset->encoder_gesture[encoder][gesture]);
        }
    }
    for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
        for (gesture = 0; gesture < MD400_COMBO_GESTURE_COUNT; gesture++) {
            md400_pedal_assignment_pack(
                &packed->combo_gesture[combo][gesture],
                &preset->combo_gesture[combo][gesture]);
        }
    }
    md400_pedal_assignment_pack(&packed->pedal, &preset->pedal);
}

static void md400_preset_unpack(md400_preset_t *preset,
                                const md400_preset_packed_t *packed)
{
    u8 encoder;
    u8 gesture;
    u8 combo;

    if (!preset || !packed) {
        return;
    }
    memset(preset, 0, sizeof(*preset));
    memcpy(preset->label, packed->label, sizeof(preset->label));
    preset->led_mode = packed->led_mode;
    preset->led_mask = packed->led_mask;
    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        md400_pedal_assignment_unpack(&packed->encoder[encoder],
                                      &preset->encoder[encoder]);
        for (gesture = 0; gesture < MD400_ENCODER_GESTURE_COUNT; gesture++) {
            md400_pedal_assignment_unpack(
                &packed->encoder_gesture[encoder][gesture],
                &preset->encoder_gesture[encoder][gesture]);
        }
    }
    for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
        for (gesture = 0; gesture < MD400_COMBO_GESTURE_COUNT; gesture++) {
            md400_pedal_assignment_unpack(
                &packed->combo_gesture[combo][gesture],
                &preset->combo_gesture[combo][gesture]);
        }
    }
    md400_pedal_assignment_unpack(&packed->pedal, &preset->pedal);
}

static u8 md400_assignment_sysex_template_id(const md400_assignment_t *assignment)
{
    if (!assignment) {
        return 0;
    }
    return assignment->sysex_id & 0x7f;
}

static u8 md400_assignment_sysex_gate_template_id(const md400_assignment_t *assignment, u8 active)
{
    u8 template_id;

    if (!assignment) {
        return 0;
    }

    if (active) {
        return md400_assignment_sysex_template_id(assignment);
    }

    template_id = assignment->data1 & 0x7f;
    if ((template_id >= 1) && (template_id <= MD400_SYSEX_TEMPLATE_COUNT)) {
        return template_id;
    }

    return md400_assignment_sysex_template_id(assignment);
}

static u8 md400_assignment_custom_template_id(const md400_assignment_t *assignment)
{
    if (!assignment) {
        return 0;
    }
    return assignment->sysex_id & 0x7f;
}

static u8 md400_assignment_custom_gate_template_id(const md400_assignment_t *assignment, u8 active)
{
    u8 template_id;

    if (!assignment) {
        return 0;
    }

    if (active) {
        return md400_assignment_custom_template_id(assignment);
    }

    template_id = assignment->data1 & 0x7f;
    if ((template_id >= 1) && (template_id <= MD400_CUSTOM_TEMPLATE_COUNT)) {
        return template_id;
    }

    return md400_assignment_custom_template_id(assignment);
}

static void md400_assignment_rotate_range_default(md400_assignment_t *assignment)
{
    if (!assignment) {
        return;
    }
    assignment->value_min = 0;
    assignment->value_max = 0x7f;
    assignment->value_default = 0;
}

static void md400_preset_rotate_range_defaults(md400_preset_t *preset)
{
    u8 encoder;

    if (!preset) {
        return;
    }

    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        md400_assignment_rotate_range_default(&preset->encoder[encoder]);
    }
}

typedef struct {
    u8 label[MD400_PRESET_LABEL_LEN];
    u8 led_mode;
    u8 led_mask;
    md400_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_assignment_t encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT_V7];
    md400_assignment_t combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT];
    md400_assignment_t pedal;
} md400_preset_v7_t;

typedef struct {
    u8 label[MD400_PRESET_LABEL_LEN];
    u8 led_mode;
    u8 led_mask;
    md400_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_assignment_t encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT_V7];
    md400_assignment_t combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT_V6];
    md400_assignment_t pedal;
} md400_preset_v6_t;

typedef struct {
    u8 label[MD400_PRESET_LABEL_LEN];
    md400_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_assignment_t encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT_V7];
    md400_assignment_t combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT_V6];
    md400_assignment_t pedal;
} md400_preset_v4_t;

typedef struct {
    md400_assignment_t encoder[MD400_ENCODER_COUNT];
    md400_assignment_t encoder_gesture[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT_V7];
    md400_assignment_t combo_gesture[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT_V6];
    md400_assignment_t pedal;
} md400_preset_v3_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 active_preset;
    u8 reserved[2];
    md400_preset_t preset[MD400_PRESET_COUNT];
    u8 checksum;
} md400_preset_store_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 active_preset;
    u8 reserved[2];
    u8 checksum;
} md400_preset_header_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 group_index;
    u8 reserved[2];
    md400_preset_v6_t preset[MD400_PRESET_GROUP_SIZE];
    u8 checksum;
} md400_preset_group_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_preset_packed_t preset;
    u8 checksum;
} md400_preset_record_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_preset_t preset;
    u8 checksum;
} md400_preset_record_v10_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    u8 label[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN];
    u8 checksum;
} md400_play_label_record_t;

typedef struct {
    u8 len;
    u8 value_index;
    u8 body[MD400_SYSEX_TEMPLATE_MAX_BODY];
} md400_sysex_template_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_sysex_template_t slot[MD400_SYSEX_TEMPLATE_COUNT];
    u8 checksum;
} md400_sysex_template_record_t;

typedef struct {
    u8 len;
    u8 value_index;
    u8 body[MD400_CUSTOM_TEMPLATE_MAX_BODY];
} md400_custom_template_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_custom_template_t slot[MD400_CUSTOM_TEMPLATE_COUNT];
    u8 checksum;
} md400_custom_template_record_t;

typedef char md400_template_record_sizes_must_match[
    (sizeof(md400_sysex_template_record_t) ==
     sizeof(md400_custom_template_record_t)) ? 1 : -1];

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_preset_v7_t preset;
    u8 checksum;
} md400_preset_record_v7_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 preset_index;
    u8 reserved[2];
    md400_preset_v6_t preset;
    u8 checksum;
} md400_preset_record_v6_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 group_index;
    u8 reserved[2];
    md400_preset_v4_t preset[MD400_PRESET_GROUP_SIZE];
    u8 checksum;
} md400_preset_group_v4_t;

typedef struct {
    u8 magic[4];
    u8 version;
    u8 group_index;
    u8 reserved[2];
    md400_preset_v3_t preset[MD400_PRESET_GROUP_SIZE];
    u8 checksum;
} md400_preset_group_v3_t;

typedef struct {
    u8 source;
    u8 event;
    u8 control;
    u8 value;
} md400_event_log_t;

typedef struct {
    u8 source; /* 0=USB, 1=BLE; replies never broadcast across transports. */
    u16 handle;
    u16 epoch;
} md400_nb_origin_t;

typedef struct {
    md400_nb_origin_t origin;
    u8 active;
    u8 cmd;
    u8 seq;
    u8 payload_len;
    u8 payload[MD400_NB_STORE_JOB_MAX_PAYLOAD];
} md400_nb_store_job_t;

typedef struct {
    u8 down;
    u8 long_sent;
    u8 click_count;
    u16 short_timer;
    u32 down_ms;
} md400_button_gesture_t;

static const u8 md400_store_magic[4] = {'M', 'D', '4', '0'};
static const u8 md400_header_magic[4] = {'M', 'D', '4', 'H'};
static const u8 md400_group_magic[4] = {'M', 'D', '4', 'G'};
static const u8 md400_record_magic[4] = {'M', 'D', '4', 'P'};
static const u8 md400_play_label_magic[4] = {'M', 'D', '4', 'L'};
static const u8 md400_sysex_template_magic[4] = {'M', 'D', '4', 'S'};
static const u8 md400_custom_template_magic[4] = {'M', 'D', '4', 'C'};
static const u8 md400_pedal_bank_magic[4] = {'N', 'F', 'P', 'A'};
static const u8 md400_pedal_bank_header_magic[4] = {'N', 'F', 'P', 'H'};
static const u16 md400_group_vm_id[2][MD400_PRESET_GROUP_COUNT] = {
    {19, 20, 21, 32, 33},
    {34, 35, 37, 38, 39},
};
static const u16 md400_preset_record_vm_id[MD400_PRESET_COUNT] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 17, 18, 22, 23, 24, 25,
};
static md400_preset_store_t md400_store;
/* Load-path scratch: the legacy full-store image and the per-version group
 * record A/B pairs are only used in mutually exclusive branches of the store
 * load, so they overlay in one union (saves ~5KB of .bss, which is what lets
 * the USB1 host stack fit in RAM). */
typedef union {
    md400_preset_store_t legacy_store;
    struct {
        md400_preset_group_t a;
        md400_preset_group_t b;
    } grp;
    struct {
        md400_preset_group_v4_t a;
        md400_preset_group_v4_t b;
    } grp_v4;
    struct {
        md400_preset_group_v3_t a;
        md400_preset_group_v3_t b;
    } grp_v3;
} md400_load_scratch_t;
/* Heap-transient (was a 5.6 KB static): only the legacy/grouped preset
 * MIGRATION needs it, which modern v11 preset records never take. Keeping it out of
 * .bss returns ~44 pmalloc pages to the pool — the difference between btstack
 * + RCSP fitting or dying in "No enough physics memory" assert loops
 * (hardware-verified). If the transient alloc ever fails, migration is
 * skipped and the store loads v10 records / defaults as usual. */
static md400_load_scratch_t *md400_load_scratch_p;
#define md400_store_read_buf      (md400_load_scratch_p->legacy_store)
#define md400_group_read_buf      (md400_load_scratch_p->grp.a)
#define md400_group_read_buf_b    (md400_load_scratch_p->grp.b)
#define md400_group_v4_read_buf   (md400_load_scratch_p->grp_v4.a)
#define md400_group_v4_read_buf_b (md400_load_scratch_p->grp_v4.b)
#define md400_group_v3_read_buf   (md400_load_scratch_p->grp_v3.a)
#define md400_group_v3_read_buf_b (md400_load_scratch_p->grp_v3.b)
static md400_preset_header_t md400_header_read_buf;
static md400_preset_header_t md400_header_read_buf_b;
static md400_preset_record_t md400_preset_record_buf;
static md400_preset_record_t md400_preset_record_read_buf;
static md400_preset_record_v10_t md400_preset_record_v10_read_buf;
static md400_play_label_record_t md400_play_label_record_buf;
static md400_play_label_record_t md400_play_label_read_buf;
/* SysEx and custom template operations are synchronous under the one persist
 * owner, and their VM record sizes match. A single heap scratch buffer avoids
 * the second 529-byte allocation that exhausts the live BT/USB heap and resets
 * the device when an editor snapshots both template types. */
static void *md400_template_record_buf;
#define md400_sysex_template_buf \
    ((md400_sysex_template_record_t *)md400_template_record_buf)
#define md400_custom_template_buf \
    ((md400_custom_template_record_t *)md400_template_record_buf)
static md400_pedal_bank_storage_t md400_pedal_bank_storage;
#define md400_pedal_bank_record \
    (md400_pedal_bank_storage.current)
#define md400_pedal_bank_record_v2_read_buf \
    (md400_pedal_bank_storage.v2)
typedef struct {
    u8 magic[4], version;
    u8 name[MD400_PRESET_COUNT][MD400_PRESET_LABEL_LEN];
    u8 checksum;
} md400_pedal_names_t;
#define MD400_VM_PEDAL_NAMES_ID 249
static md400_pedal_names_t md400_pedal_names, md400_pedal_names_verify;
static u8 md400_pedal_names_dirty;
static md400_pedal_bank_header_t md400_pedal_bank_header;
static md400_pedal_bank_header_t md400_pedal_bank_header_verify;
static md400_preset_record_v7_t md400_preset_record_v7_read_buf;
static md400_preset_record_v6_t md400_preset_record_v6_read_buf;
static u8 md400_control_ready;
static u8 md400_store_active_slot;
static u16 md400_header_seq_value;
static u8 md400_preset_group_active_slot[MD400_PRESET_GROUP_COUNT];
static u16 md400_preset_group_active_seq[MD400_PRESET_GROUP_COUNT];
static u8 md400_play_label[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN];
static u8 md400_play_label_work[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN];
static u8 md400_sysex_send_buf[MD400_SYSEX_TEMPLATE_MAX_BODY + 2];
static u8 md400_custom_send_buf[MD400_CUSTOM_TEMPLATE_MAX_BODY];
static int md400_play_last_msg = MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_CC, 1);
static u8 md400_play_last_value;
static u8 md400_play_last_action = MD400_PLAY_LABEL_ROTATE_ACTION;
static u8 md400_play_last_control = 1;
static u8 md400_active_preset;
static u8 md400_selected_preset;
static u8 md400_foot_mode;
static u8 md400_pedal_active_preset;
static u8 md400_pedal_selected_preset;
static u8 md400_pedal_page;
static u8 md400_pedal_rotate_value[MD400_PRESET_COUNT]
                                    [MD400_PEDAL_PAGE_COUNT]
                                    [MD400_PEDAL_SLOT_COUNT];
static u8 md400_pedal_edit_slot = 1;
static u8 md400_pedal_edit_view;
static u8 md400_pedal_label_position;
static u8 md400_pedal_record_loaded;
static u8 md400_pedal_record_dirty;
static u16 md400_dbg_vm_id;
static int md400_dbg_vm_write_ret;
static int md400_dbg_vm_verify_ret;
static u8 md400_dbg_vm_valid;
static u8 md400_dbg_vm_match;
static u8 md400_dbg_vm_which;
#define MD400_DBG_VM_PRESET  1
#define MD400_DBG_VM_HEADER  2
#define MD400_DBG_VM_LABEL   3
#define MD400_DBG_VM_SYSEX   4
#define MD400_DBG_VM_CUSTOM  5
#define MD400_DBG_VM_PEDAL   6

static void md400_dbg_vm_fail(u8 which, u16 id, int write_ret, int verify_ret,
                              u8 valid, u8 match)
{
    md400_dbg_vm_which = which;
    md400_dbg_vm_id = id;
    md400_dbg_vm_write_ret = write_ret;
    md400_dbg_vm_verify_ret = verify_ret;
    md400_dbg_vm_valid = valid;
    md400_dbg_vm_match = match;
}
typedef struct {
    u8 message;
    u8 arg0;
    u8 arg1;
} md400_persist_message_t;

static md400_persist_message_t md400_persist_queue[MD400_PERSIST_QUEUE_DEPTH];
static volatile u8 md400_persist_queue_head;
static volatile u8 md400_persist_queue_tail;
static volatile u8 md400_persist_queue_count;
static volatile u8 md400_persist_callback_pending;
static u16 md400_persist_post_retry_timer;
static u8 md400_pedal_exp_dirty;
static md400_assignment_t md400_pedal_exp_snapshot;
static u8 md400_pedal_exp_snapshot_preset;
static u8 md400_pedal_record_preset;
static u8 md400_pedal_record_index;
static u8 md400_pedal_generation = 1;
static u8 md400_ui_mode;
static u8 md400_edit_page;
static u8 md400_edit_target_encoder;
static u8 md400_edit_subject;
/* Subject that was being edited when the pedal-hold captured the edit page,
 * restored when an encoder press leaves the pedal-settings page. */
static u8 md400_edit_subject_before_pedal;
static md400_preset_t md400_edit_snapshot;
static u8 md400_edit_session_active;
static u8 md400_edit_session_dirty;
static u8 md400_encoder_runtime_value[MD400_ENCODER_COUNT];
static u8 md400_pedal_runtime_value;
static u8 md400_pedal_adc_value;
static u8 md400_pedal_seen;
static u8 md400_encoder_note_toggle_state[MD400_ENCODER_COUNT];
static u8 md400_encoder_gesture_toggle_state[MD400_ENCODER_COUNT][MD400_ENCODER_GESTURE_COUNT];
static u8 md400_combo_gesture_toggle_state[MD400_COMBO_COUNT][MD400_COMBO_GESTURE_COUNT];
static u8 md400_encoder_press_momentary_active[MD400_ENCODER_COUNT];
static u8 md400_encoder_short_momentary_active[MD400_ENCODER_COUNT];
static u8 md400_encoder_long_momentary_active[MD400_ENCODER_COUNT];
static u8 md400_combo_long_momentary_active[MD400_COMBO_COUNT];
static md400_event_log_t md400_event_log[16];
static u8 md400_event_log_wp;
static u32 md400_event_log_total;
static md400_button_gesture_t md400_button_gesture[MD400_ENCODER_COUNT];
static u16 md400_gesture_timer;
static u8 md400_button_down_mask;
static u8 md400_combo_mask;
static u8 md400_combo_long_sent;
static u32 md400_combo_start_ms;
static u16 md400_led_pattern_timer;
static u16 md400_preset_load_timer;
static u16 md400_edit_key_timer;
static md400_nb_store_job_t md400_nb_store_job;
static md400_nb_origin_t md400_nb_origin;
static u8 md400_nb_dispatch_active;
static u16 md400_nb_revision;
static u8 md400_nb_change_source = 2;
static md400_nb_stream_t md400_nb_ble_stream;
static u16 md400_nb_ble_epoch;
static u16 md400_nb_ble_mtu = 23;
static u32 md400_nb_ble_last_activity;
static u8 md400_nb_ble_editor;
static u8 md400_nb_notify_pending;
static u8 md400_nb_usb_subscribed, md400_nb_usb_notify_valid;
static u32 md400_nb_usb_last_activity;
static u8 md400_nb_usb_last_state[14];
static u32 md400_nb_notify_last_post;
typedef struct {
    u8 active, len;
    md400_nb_origin_t origin;
    u8 data[MD400_NB_STREAM_LIMIT];
} md400_nb_request_t;
static md400_nb_request_t md400_nb_requests[2];
typedef struct {
    u8 len, pos;
    md400_nb_origin_t origin;
    u8 data[MD400_NB_BLE_FRAME_SIZE];
} md400_nb_ble_reply_t;
static md400_nb_ble_reply_t md400_nb_ble_replies[2];
static u8 md400_nb_ble_tx_head, md400_nb_ble_tx_count;

static u8 md400_header_save_pending;
static u8 md400_store_save_pending;
static u8 md400_store_save_reason;
static u16 md400_edit_key_short_timer;
static u8 md400_edit_key_click_count;
static u8 md400_edit_key_stable;
static u8 md400_edit_key_last_raw;
static u8 md400_edit_key_debounce_ticks;
static u16 md400_edit_key_press_ticks;
static u8 md400_edit_key_long_sent;
static u16 md400_edit_key_short_count;
static u16 md400_edit_key_long_count;
static u16 md400_edit_key_double_count;
static u16 md400_edit_key_edge_count;
static u8 md400_led_policy_mode;
static u8 md400_led_policy_mask;
static u16 md400_nb_rx_valid_count;
static u16 md400_nb_reply_count;
static u16 md400_nb_status_ok_count;
static u16 md400_nb_status_bad_checksum_count;
static u16 md400_nb_status_bad_length_count;
static u16 md400_nb_status_bad_value_count;
static u16 md400_nb_status_unsupported_count;
static u16 md400_nb_status_busy_count;
static u16 md400_nb_status_flash_error_count;

static void md400_control_init(void);
static void md400_nb_sysex_handle(const u8 *sysex, u16 len);
static void md400_nb_process_request(u8 slot);
static void md400_nb_submit(md400_nb_origin_t origin, const u8 *frame, u16 length);
static u8 md400_nb_ble_tx_pump(void);
static void md400_nb_state_payload(u8 *payload);
static void md400_nb_note_change(u8 source);
static void md400_nb_notify_state(void);

static int md400_assignment_send_message(const md400_assignment_t *assignment, u8 control, u8 value);
static int md400_assignment_send_gate(const md400_assignment_t *assignment, u8 control, u8 active);
static void md400_ui_display_sync(void);

/*
 * DAW mode state. A host DAW script enables it with NanoBridge DAW_MODE (4E)
 * and feeds the display with DAW_UPDATE (4F). While active the encoders, edit
 * key and expression jack send the fixed DAW control map (MIDI channel 16)
 * to the transport that enabled DAW mode instead of the preset assignments.
 * All state is RAM only.
 */
static u8 md400_daw_active;       /* display and controls taken over */
static u8 md400_daw_waiting;
static u8 md400_daw_requested;    /* host asked for DAW mode */
static u8 md400_daw_user_off;     /* user left with the edit key; stay off */
static u8 md400_daw_host_seen;    /* host traffic within the timeout */
static u8 md400_daw_source;       /* 0 USB, 1 BLE */
static u8 md400_daw_host_id;
static u8 md400_daw_led_mask;
static u8 md400_daw_enc_prev[4];
static u32 md400_daw_host_ms;
static volatile u8 md400_daw_draw_overflow; /* a DAW_DRAW frame was dropped */
static u32 md400_daw_draw_rx_count;       /* DAW_DRAW frames queued */
static u32 md400_daw_draw_drop_count;     /* DAW_DRAW frames dropped: queue full */
extern u32 md400_usb_rx_limited_count;    /* uac1.c: USB packets dropped by its rate limit */
static u8 md400_daw_pad_mask;      /* switches that play a note while held */
static u8 md400_daw_pad_channel;
static u8 md400_daw_pad_note[4];
static u8 md400_daw_pad_down;      /* notes currently sounding, by switch */
static u16 md400_daw_timer;
static void md400_daw_gesture(u8 event, u8 control);
static void md400_put_u14(u8 *p, u16 v);
static void md400_daw_rotate(u8 encoder, u8 value, u8 event);
static void md400_daw_pedal_value(u8 value);
static u8 md400_daw_edit_key(u8 event);
static void md400_daw_leave(void);
static void md400_daw_timer_tick(void *priv);
static void md400_ui_event(u8 event);
static void md400_edit_normalize_subject_for_page(void);
static u8 md400_edit_page_is_preset_select(void);
static void md400_led_sync(void);
static void md400_led_pattern_tick(void *priv);
static void md400_header_save_schedule(void);
static void md400_store_save_schedule(u8 reason);
static void md400_log_event(u8 source, u8 event, u8 control, u8 value);
static void md400_preset_load_timer_cancel(void);
static void md400_preset_load_schedule(void);
static void md400_pedal_settings_enter_cancel(void);
static int md400_assignment_send_output_value(const md400_assignment_t *assignment,
                                              u8 control, u8 out_value);
static void md400_play_show_assignment(const md400_assignment_t *assignment,
                                       u8 action, u8 control, u8 value);
static void md400_play_show_assignment_output(const md400_assignment_t *assignment,
                                             u8 action, u8 control, u8 value);
static void md400_play_show_assignment_sent(const md400_assignment_t *assignment,
                                            u8 action, u8 control, int msg,
                                            u8 out_value);
static void md400_play_refresh_display(void);
static u8 md400_pedal_bank_load(u8 preset, u8 page);
static int md400_pedal_bank_save(void);
static void md400_pedal_header_save(void);
static void md400_pedal_header_save_now(void);
static void md400_foot_mode_set(u8 mode);
static void md400_foot_mode_set_now(u8 mode);
static void md400_pedal_page_step(int delta);
static void md400_pedal_release_action(u8 slot);
static void md400_persist_app_callback(void *priv);
static u8 md400_nb_play_label_to_payload(const u8 *payload, u8 payload_len,
                                         u8 *reply, u8 *reply_len);
static u8 md400_nb_apply_play_label_payload(const u8 *payload, u8 payload_len);
static u8 md400_nb_sysex_template_to_payload(const u8 *payload, u8 payload_len,
                                             u8 *reply, u8 *reply_len);
static u8 md400_nb_apply_sysex_template_payload(const u8 *payload, u8 payload_len);
static u8 md400_nb_custom_template_to_payload(const u8 *payload, u8 payload_len,
                                              u8 *reply, u8 *reply_len);
static u8 md400_nb_apply_custom_template_payload(const u8 *payload, u8 payload_len);
static u8 md400_runtime_save_active_preset_now(void);
static u8 md400_runtime_load_preset_from_vm_now(u8 preset_number);
static u8 md400_runtime_factory_reset_now(void);

static u8 md400_clip7(u8 value)
{
    return (value > 0x7f) ? 0x7f : value;
}

static u8 md400_pedal_cal_range_valid(u8 min8, u8 max8)
{
    u16 span = (max8 >= min8) ? ((u16)max8 - (u16)min8) :
               ((u16)min8 - (u16)max8);

    return span >= MD400_PEDAL_CAL_MIN_SPAN;
}

static void md400_pedal_cal_load(void)
{
    md400_pedal_cal_record_t rec;
    int ret;

    if (md400_pedal_cal_loaded) {
        return;
    }

    md400_pedal_cal_loaded = 1;
    ret = syscfg_read(MD400_VM_PEDAL_CAL_ID, &rec, sizeof(rec));
    if ((ret == (int)sizeof(rec)) &&
        (rec.magic[0] == 'M') && (rec.magic[1] == 'D') &&
        (rec.magic[2] == '4') && (rec.magic[3] == 'C') &&
        md400_pedal_cal_range_valid(rec.min8, rec.max8)) {
        md400_pedal_cal_min8 = rec.min8;
        md400_pedal_cal_max8 = rec.max8;
    } else {
        md400_pedal_cal_min8 = MD400_PEDAL_CAL_MIN_DEFAULT;
        md400_pedal_cal_max8 = MD400_PEDAL_CAL_MAX_DEFAULT;
    }
}

static u8 md400_pedal_cal_scale(u8 raw8)
{
    u16 span;
    u16 scaled;

    md400_pedal_cal_load();
    if (!md400_pedal_cal_range_valid(md400_pedal_cal_min8,
                                     md400_pedal_cal_max8)) {
        return (raw8 >> 1) & 0x7f;
    }

    if (md400_pedal_cal_max8 >= md400_pedal_cal_min8) {
        if (raw8 <= md400_pedal_cal_min8) {
            return 0;
        }
        if (raw8 >= md400_pedal_cal_max8) {
            return 0x7f;
        }
        span = (u16)md400_pedal_cal_max8 - (u16)md400_pedal_cal_min8;
        scaled = (((u16)raw8 - (u16)md400_pedal_cal_min8) * 127u) +
                 (span / 2u);
    } else {
        if (raw8 >= md400_pedal_cal_min8) {
            return 0;
        }
        if (raw8 <= md400_pedal_cal_max8) {
            return 0x7f;
        }
        span = (u16)md400_pedal_cal_min8 - (u16)md400_pedal_cal_max8;
        scaled = (((u16)md400_pedal_cal_min8 - (u16)raw8) * 127u) +
                 (span / 2u);
    }

    return (u8)(scaled / span);
}

static void md400_pedal_cal_ui_sync(void)
{
    md400_display_pedal_cal_update(md400_pedal_cal_state,
                                   md400_pedal_adc_value,
                                   md400_pedal_cal_capture_min8,
                                   md400_pedal_cal_capture_max8,
                                   md400_pedal_cal_scale(md400_pedal_adc_value));
}

static void md400_pedal_cal_store_range_now(u8 min8, u8 max8)
{
    md400_pedal_cal_record_t rec;
    int ret;

    if (!md400_pedal_cal_range_valid(min8, max8)) {
        min8 = MD400_PEDAL_CAL_MIN_DEFAULT;
        max8 = MD400_PEDAL_CAL_MAX_DEFAULT;
    }

    md400_pedal_cal_loaded = 1;
    md400_pedal_cal_min8 = min8;
    md400_pedal_cal_max8 = max8;
    rec.magic[0] = 'M';
    rec.magic[1] = 'D';
    rec.magic[2] = '4';
    rec.magic[3] = 'C';
    rec.min8 = min8;
    rec.max8 = max8;
    rec.rsvd[0] = rec.rsvd[1] = 0;
    ret = syscfg_write(MD400_VM_PEDAL_CAL_ID, &rec, sizeof(rec));
    if (ret != (int)sizeof(rec)) {
        printf("[MD400_PEDAL] cal store fail ret=%d\r\n", ret);
    }
    printf("[MD400_PEDAL] cal range min=%u max=%u\r\n",
           (unsigned)min8, (unsigned)max8);
}

static void md400_pedal_cal_store_range(u8 min8, u8 max8)
{
    if (md400_persist_in_task) {
        md400_pedal_cal_store_range_now(min8, max8);
        return;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_PEDAL_CAL, min8, max8)) {
        printf("[MD400_PERSIST] pedal calibration queue failed min=%u max=%u\r\n",
               (unsigned)min8, (unsigned)max8);
    }
}

static void md400_pedal_cal_hide(void)
{
    md400_pedal_cal_active = 0;
    md400_pedal_cal_state = MD400_DISPLAY_PEDAL_CAL_HIDDEN;
    md400_pedal_cal_ui_sync();
}

static void md400_pedal_cal_reset(void)
{
    md400_pedal_cal_store_range(MD400_PEDAL_CAL_MIN_DEFAULT,
                                MD400_PEDAL_CAL_MAX_DEFAULT);
    md400_pedal_cal_hide();
}

static void md400_pedal_cal_start(u8 raw8)
{
    md400_pedal_cal_load();
    md400_pedal_cal_active = 1;
    md400_pedal_cal_state = MD400_DISPLAY_PEDAL_CAL_MIN;
    md400_pedal_cal_capture_min8 = raw8;
    md400_pedal_cal_capture_max8 = raw8;
    md400_pedal_cal_ui_sync();
    printf("[MD400_PEDAL] cal guide start raw=%u\r\n", (unsigned)raw8);
}

static void md400_pedal_cal_update(u8 raw8)
{
    (void)raw8;
}

static void md400_pedal_cal_confirm(u8 raw8)
{
    if (md400_pedal_cal_state == MD400_DISPLAY_PEDAL_CAL_MIN) {
        md400_pedal_cal_capture_min8 = raw8;
        md400_pedal_cal_capture_max8 = raw8;
        md400_pedal_cal_state = MD400_DISPLAY_PEDAL_CAL_MAX;
        md400_pedal_cal_active = 1;
        md400_pedal_cal_ui_sync();
        printf("[MD400_PEDAL] cal min=%u\r\n", (unsigned)raw8);
        return;
    }

    if (md400_pedal_cal_state == MD400_DISPLAY_PEDAL_CAL_MAX) {
        md400_pedal_cal_capture_max8 = raw8;
        if (!md400_pedal_cal_range_valid(md400_pedal_cal_capture_min8,
                                         md400_pedal_cal_capture_max8)) {
            md400_pedal_cal_state = MD400_DISPLAY_PEDAL_CAL_ERROR;
            md400_pedal_cal_active = 1;
            md400_pedal_cal_ui_sync();
            printf("[MD400_PEDAL] cal span low min=%u max=%u\r\n",
                   (unsigned)md400_pedal_cal_capture_min8,
                   (unsigned)md400_pedal_cal_capture_max8);
            return;
        }
        md400_pedal_cal_store_range(md400_pedal_cal_capture_min8,
                                    md400_pedal_cal_capture_max8);
        md400_pedal_cal_active = 0;
        md400_pedal_cal_state = MD400_DISPLAY_PEDAL_CAL_OK;
        md400_pedal_cal_ui_sync();
        printf("[MD400_PEDAL] cal saved min=%u max=%u value=%u\r\n",
               (unsigned)md400_pedal_cal_min8,
               (unsigned)md400_pedal_cal_max8,
               (unsigned)md400_pedal_cal_scale(md400_pedal_adc_value));
        return;
    }

    md400_pedal_cal_start(raw8);
}

static u8 md400_pedal_cal_display_value(void)
{
    return md400_pedal_cal_state;
}

static u8 md400_checksum8(const u8 *data, u16 len)
{
    u16 i;
    u8 sum = 0;

    for (i = 0; i < len; i++) {
        sum += data[i];
    }

    return (u8)(0 - sum);
}

static u16 md400_pedal_bank_vm_id(u8 preset, u8 record)
{
    return MD400_VM_PEDAL_BANK_FIRST_ID +
           ((u16)preset * MD400_PEDAL_RECORDS_PER_PRESET) + record;
}

static void md400_pedal_default_label(u8 action_number, u8 *label)
{
    u8 shown = action_number + 1;

    memset(label, 0, MD400_PEDAL_LABEL_LEN);
    label[0] = 'P';
    label[1] = 'C';
    label[2] = ' ';
    label[3] = '0' + (shown / 100);
    label[4] = '0' + ((shown / 10) % 10);
    label[5] = '0' + (shown % 10);
}

static void md400_pedal_action_default(md400_pedal_action_t *action,
                                        u8 action_number)
{
    md400_assignment_t assignment;

    if (!action) {
        return;
    }
    memset(action, 0, sizeof(*action));
    memset(&assignment, 0, sizeof(assignment));
    assignment.type = MD400_MIDI_TYPE_PC;
    assignment.channel = 0;
    assignment.data1 = action_number & 0x7f;
    assignment.value_min = 0;
    assignment.value_max = 0x7f;
    assignment.value_default = action_number & 0x7f;
    assignment.behavior = MD400_BEHAVIOR_ABSOLUTE;
    assignment.target_mask = md400_available_target_mask();
    md400_pedal_assignment_pack(&action->press, &assignment);

    assignment.type = MD400_MIDI_TYPE_CC;
    assignment.data1 = action_number & 0x7f;
    assignment.value_default = 0;
    md400_pedal_assignment_pack(&action->rotate, &assignment);
    md400_pedal_default_label(action_number, action->label);
    md400_pedal_action_color_set(action,
                                 action_number % MD400_PEDAL_SLOT_COUNT);
}

static u8 md400_pedal_record_checksum(const md400_pedal_bank_record_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_pedal_bank_record_t) - 1);
}

static u8 md400_pedal_record_v2_checksum(
    const md400_pedal_bank_record_v2_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_pedal_bank_record_v2_t) - 1);
}

static u8 md400_pedal_record_v1_checksum(
    const md400_pedal_bank_record_v1_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_pedal_bank_record_v1_t) - 1);
}

static u8 md400_pedal_assignment_valid(const md400_assignment_t *assignment)
{
    return assignment &&
           (assignment->type <= MD400_MIDI_TYPE_MAX) &&
           (assignment->channel <= 15) &&
           (assignment->data1 <= 0x7f) &&
           (assignment->value_min <= 0x7f) &&
           (assignment->value_max <= 0x7f) &&
           (assignment->value_default <= 0x7f) &&
           (assignment->behavior <= MD400_BEHAVIOR_PULSE) &&
           (assignment->sysex_id <= 0x7f);
}

static u8 md400_pedal_record_v2_valid(
    const md400_pedal_bank_record_v2_t *record,
    u8 preset, u8 record_index)
{
    md400_assignment_t assignment;
    u8 i;

    if (!record ||
        memcmp(record->magic, md400_pedal_bank_magic,
               sizeof(md400_pedal_bank_magic)) ||
        (record->version != MD400_PEDAL_STORE_VERSION_V2) ||
        (record->preset_index != preset) ||
        (record->record_index != record_index) ||
        (record->reserved != md400_pedal_generation) ||
        (record->checksum != md400_pedal_record_v2_checksum(record))) {
        return 0;
    }
    for (i = 0; i < MD400_PEDAL_RECORD_SLOT_COUNT; i++) {
        md400_pedal_assignment_unpack(&record->action[i].press, &assignment);
        if (!md400_pedal_assignment_valid(&assignment)) {
            return 0;
        }
        md400_pedal_assignment_unpack(&record->action[i].rotate, &assignment);
        if (!md400_pedal_assignment_valid(&assignment) ||
            (record->action[i].color >= MD400_DISPLAY_PEDAL_COLOR_COUNT)) {
            return 0;
        }
    }
    return 1;
}

static void md400_pedal_record_migrate_v2(u8 preset, u8 record_index)
{
    md400_pedal_action_v2_t old_action;
    u8 i;

    /* v3 actions are one byte smaller than v2 actions. Convert forward in the
     * shared union, copying each source action before its destination write. */
    for (i = 0; i < MD400_PEDAL_RECORD_SLOT_COUNT; i++) {
        memcpy(&old_action, &md400_pedal_bank_record_v2_read_buf.action[i],
               sizeof(old_action));
        md400_pedal_bank_record.action[i].press =
            old_action.press;
        md400_pedal_bank_record.action[i].rotate =
            old_action.rotate;
        memcpy(md400_pedal_bank_record.action[i].label,
               old_action.label,
               MD400_PEDAL_LABEL_LEN);
        md400_pedal_action_color_set(&md400_pedal_bank_record.action[i],
                                     old_action.color);
    }
    memcpy(md400_pedal_bank_record.magic, md400_pedal_bank_magic,
           sizeof(md400_pedal_bank_magic));
    md400_pedal_bank_record.version = MD400_PEDAL_STORE_VERSION;
    md400_pedal_bank_record.preset_index = preset;
    md400_pedal_bank_record.record_index = record_index;
    md400_pedal_bank_record.reserved = md400_pedal_generation;
    md400_pedal_bank_record.checksum =
        md400_pedal_record_checksum(&md400_pedal_bank_record);
}

static u8 md400_pedal_record_v1_valid(
    const md400_pedal_bank_record_v1_t *record,
    u8 preset, u8 record_index)
{
    u8 i;

    if (!record ||
        memcmp(record->magic, md400_pedal_bank_magic,
               sizeof(md400_pedal_bank_magic)) ||
        (record->version != MD400_PEDAL_STORE_VERSION_V1) ||
        (record->preset_index != preset) ||
        (record->record_index != record_index) ||
        (record->reserved != md400_pedal_generation) ||
        (record->checksum != md400_pedal_record_v1_checksum(record))) {
        return 0;
    }
    for (i = 0; i < MD400_PEDAL_RECORD_SLOT_COUNT; i++) {
        if (!md400_pedal_assignment_valid(&record->action[i].assignment) ||
            (record->action[i].color >= MD400_DISPLAY_PEDAL_COLOR_COUNT)) {
            return 0;
        }
    }
    return 1;
}

static void md400_pedal_record_migrate_v1(u8 preset, u8 record_index)
{
    md400_pedal_bank_record_v1_t *legacy =
        (md400_pedal_bank_record_v1_t *)&md400_pedal_bank_record;
    md400_pedal_action_v1_t old_action;
    md400_assignment_t rotate;
    int i;

    /* Convert backwards because the v3 actions are larger and share this
     * buffer with the legacy record read from VM. */
    for (i = MD400_PEDAL_RECORD_SLOT_COUNT - 1; i >= 0; i--) {
        memcpy(&old_action, &legacy->action[i], sizeof(old_action));
        memset(&md400_pedal_bank_record.action[i], 0,
               sizeof(md400_pedal_bank_record.action[i]));
        old_action.assignment.target_mask = md400_available_target_mask();
        md400_pedal_assignment_pack(&md400_pedal_bank_record.action[i].press,
                                    &old_action.assignment);
        memset(&rotate, 0, sizeof(rotate));
        rotate.type = MD400_MIDI_TYPE_CC;
        rotate.channel = 0;
        rotate.data1 = (record_index * MD400_PEDAL_RECORD_SLOT_COUNT) + i;
        rotate.value_min = 0;
        rotate.value_max = 0x7f;
        rotate.behavior = MD400_BEHAVIOR_ABSOLUTE;
        rotate.target_mask = md400_available_target_mask();
        md400_pedal_assignment_pack(&md400_pedal_bank_record.action[i].rotate,
                                    &rotate);
        memcpy(md400_pedal_bank_record.action[i].label, old_action.label,
               MD400_PEDAL_LABEL_LEN);
        md400_pedal_action_color_set(&md400_pedal_bank_record.action[i],
                                     old_action.color);
    }
    memcpy(md400_pedal_bank_record.magic, md400_pedal_bank_magic,
           sizeof(md400_pedal_bank_magic));
    md400_pedal_bank_record.version = MD400_PEDAL_STORE_VERSION;
    md400_pedal_bank_record.preset_index = preset;
    md400_pedal_bank_record.record_index = record_index;
    md400_pedal_bank_record.reserved = md400_pedal_generation;
    md400_pedal_bank_record.checksum =
        md400_pedal_record_checksum(&md400_pedal_bank_record);
}

static void md400_pedal_record_defaults(u8 preset, u8 record_index)
{
    u8 i;
    u8 first_action = record_index * MD400_PEDAL_RECORD_SLOT_COUNT;

    memset(&md400_pedal_bank_record, 0, sizeof(md400_pedal_bank_record));
    memcpy(md400_pedal_bank_record.magic, md400_pedal_bank_magic,
           sizeof(md400_pedal_bank_record.magic));
    md400_pedal_bank_record.version = MD400_PEDAL_STORE_VERSION;
    md400_pedal_bank_record.preset_index = preset;
    md400_pedal_bank_record.record_index = record_index;
    md400_pedal_bank_record.reserved = md400_pedal_generation;
    for (i = 0; i < MD400_PEDAL_RECORD_SLOT_COUNT; i++) {
        md400_pedal_action_default(&md400_pedal_bank_record.action[i],
                                   first_action + i);
    }
    md400_pedal_bank_record.checksum =
        md400_pedal_record_checksum(&md400_pedal_bank_record);
}

static u8 md400_pedal_record_valid(const md400_pedal_bank_record_t *record,
                                   u8 preset, u8 record_index)
{
    md400_assignment_t assignment;
    u8 i;

    if (!record ||
        memcmp(record->magic, md400_pedal_bank_magic,
               sizeof(md400_pedal_bank_magic)) ||
        (record->version != MD400_PEDAL_STORE_VERSION) ||
        (record->preset_index != preset) ||
        (record->record_index != record_index) ||
        (record->reserved != md400_pedal_generation) ||
        (record->checksum != md400_pedal_record_checksum(record))) {
        return 0;
    }

    for (i = 0; i < MD400_PEDAL_RECORD_SLOT_COUNT; i++) {
        const md400_pedal_action_t *action = &record->action[i];
        md400_pedal_assignment_unpack(&action->press, &assignment);
        if (!md400_pedal_assignment_valid(&assignment)) {
            return 0;
        }
        md400_pedal_assignment_unpack(&action->rotate, &assignment);
        if (!md400_pedal_assignment_valid(&assignment) ||
            (md400_pedal_action_color_get(action) >=
             MD400_DISPLAY_PEDAL_COLOR_COUNT)) {
            return 0;
        }
    }
    return 1;
}

static int md400_pedal_bank_save(void)
{
    u16 vm_id;
    int ret;
    int verify_ret;
    u8 attempt;
    u8 expected_checksum;

    if (!md400_persist_in_task) {
        printf("[MD400_PERSIST] reject pedal save outside owner\r\n");
        return -1;
    }
    if (!md400_pedal_record_loaded || !md400_pedal_record_dirty) {
        return sizeof(md400_pedal_bank_record);
    }

    md400_pedal_bank_record.checksum =
        md400_pedal_record_checksum(&md400_pedal_bank_record);
    expected_checksum = md400_pedal_bank_record.checksum;
    vm_id = md400_pedal_bank_vm_id(md400_pedal_record_preset,
                                   md400_pedal_record_index);

    for (attempt = 1; attempt <= MD400_PEDAL_BANK_SAVE_TRIES; attempt++) {
        if (attempt > 1) {
            os_time_dly(MD400_PEDAL_BANK_SAVE_RETRY_TICKS);
        }
        ret = syscfg_write(vm_id, &md400_pedal_bank_record,
                           sizeof(md400_pedal_bank_record));
        verify_ret = syscfg_read(vm_id, &md400_pedal_bank_record,
                                 sizeof(md400_pedal_bank_record));
        printf("[NEXFOOT_PEDAL] save try=%u preset=%u rec=%u id=%u need=%u ret=%d verify=%d dirty=%u\r\n",
               (unsigned)attempt,
               (unsigned)(md400_pedal_record_preset + 1),
               (unsigned)md400_pedal_record_index,
               (unsigned)vm_id,
               (unsigned)sizeof(md400_pedal_bank_record),
               ret, verify_ret,
               (unsigned)md400_pedal_record_dirty);
        if ((ret == sizeof(md400_pedal_bank_record)) &&
            (verify_ret == sizeof(md400_pedal_bank_record)) &&
            (md400_pedal_bank_record.checksum == expected_checksum) &&
            md400_pedal_record_valid(&md400_pedal_bank_record,
                                     md400_pedal_record_preset,
                                     md400_pedal_record_index)) {
            md400_pedal_record_dirty = 0;
            printf("[NEXFOOT_PEDAL] save preset=%u pages=%u-%u id=%u len=%u\r\n",
                   (unsigned)(md400_pedal_record_preset + 1),
                   (unsigned)(md400_pedal_record_index *
                              MD400_PEDAL_RECORD_PAGE_COUNT + 1),
                   (unsigned)((((md400_pedal_record_index + 1) *
                                MD400_PEDAL_RECORD_PAGE_COUNT) >
                               MD400_PEDAL_PAGE_COUNT) ?
                              MD400_PEDAL_PAGE_COUNT :
                              ((md400_pedal_record_index + 1) *
                               MD400_PEDAL_RECORD_PAGE_COUNT)),
                   (unsigned)vm_id,
                   (unsigned)sizeof(md400_pedal_bank_record));
            return ret;
        }
    }

    md400_dbg_vm_fail(MD400_DBG_VM_PEDAL, vm_id, ret, verify_ret,
                      md400_pedal_record_valid(&md400_pedal_bank_record,
                                               md400_pedal_record_preset,
                                               md400_pedal_record_index),
                      md400_pedal_bank_record.checksum == expected_checksum);
    printf("[NEXFOOT_PEDAL] save fail preset=%u record=%u id=%u ret=%d verify=%d\r\n",
           (unsigned)(md400_pedal_record_preset + 1),
           (unsigned)md400_pedal_record_index,
           (unsigned)vm_id, ret, verify_ret);
    return -1;
}

static u8 md400_pedal_bank_load(u8 preset, u8 page)
{
    u8 record_index;
    u8 migrated = 0;
    int ret;
    int v2_ret;
    int legacy_ret;

    preset %= MD400_PRESET_COUNT;
    page %= MD400_PEDAL_PAGE_COUNT;
    record_index = page / MD400_PEDAL_RECORD_PAGE_COUNT;
    if (md400_pedal_record_loaded &&
        (md400_pedal_record_preset == preset) &&
        (md400_pedal_record_index == record_index)) {
        return 1;
    }

    if (md400_pedal_record_dirty) {
        printf("[MD400_PERSIST] pedal load blocked preset=%u record=%u dirty=1\r\n",
               (unsigned)(md400_pedal_record_preset + 1),
               (unsigned)md400_pedal_record_index);
        return 0;
    }

    ret = syscfg_read(md400_pedal_bank_vm_id(preset, record_index),
                      &md400_pedal_bank_record,
                      sizeof(md400_pedal_bank_record));
    if ((ret == sizeof(md400_pedal_bank_record)) &&
        md400_pedal_record_valid(&md400_pedal_bank_record, preset,
                                 record_index)) {
        /* Current record loaded. */
    } else {
        v2_ret = syscfg_read(md400_pedal_bank_vm_id(preset, record_index),
                             &md400_pedal_bank_record_v2_read_buf,
                             sizeof(md400_pedal_bank_record_v2_read_buf));
        if ((v2_ret == sizeof(md400_pedal_bank_record_v2_read_buf)) &&
            md400_pedal_record_v2_valid(
                &md400_pedal_bank_record_v2_read_buf, preset, record_index)) {
            md400_pedal_record_migrate_v2(preset, record_index);
            migrated = 1;
            printf("[NEXFOOT_PEDAL] migrate v2 preset=%u record=%u len=%u->%u\r\n",
                   (unsigned)(preset + 1), (unsigned)record_index,
                   (unsigned)sizeof(md400_pedal_bank_record_v2_read_buf),
                   (unsigned)sizeof(md400_pedal_bank_record));
        } else {
            /* JL syscfg may return the requested v3 length even when the
             * stored item is shorter. Retry v1 with its exact size before
             * falling back to defaults. */
            legacy_ret = syscfg_read(
                md400_pedal_bank_vm_id(preset, record_index),
                &md400_pedal_bank_record,
                sizeof(md400_pedal_bank_record_v1_t));
            if ((legacy_ret == sizeof(md400_pedal_bank_record_v1_t)) &&
                md400_pedal_record_v1_valid(
                    (const md400_pedal_bank_record_v1_t *)&md400_pedal_bank_record,
                    preset, record_index)) {
                md400_pedal_record_migrate_v1(preset, record_index);
                migrated = 1;
                printf("[NEXFOOT_PEDAL] migrate v1 preset=%u record=%u len=%u->%u\r\n",
                       (unsigned)(preset + 1), (unsigned)record_index,
                       (unsigned)sizeof(md400_pedal_bank_record_v1_t),
                       (unsigned)sizeof(md400_pedal_bank_record));
            } else {
                md400_pedal_record_defaults(preset, record_index);
                printf("[NEXFOOT_PEDAL] defaults preset=%u pages=%u-%u ret=%d v2=%d v1=%d\r\n",
                       (unsigned)(preset + 1),
                       (unsigned)(record_index *
                                  MD400_PEDAL_RECORD_PAGE_COUNT + 1),
                       (unsigned)((((record_index + 1) *
                                    MD400_PEDAL_RECORD_PAGE_COUNT) >
                                   MD400_PEDAL_PAGE_COUNT) ?
                                  MD400_PEDAL_PAGE_COUNT :
                                  ((record_index + 1) *
                                   MD400_PEDAL_RECORD_PAGE_COUNT)),
                       ret, v2_ret, legacy_ret);
            }
        }
    }
    md400_pedal_record_loaded = 1;
    md400_pedal_record_dirty = migrated;
    md400_pedal_record_preset = preset;
    md400_pedal_record_index = record_index;
    return 1;
}

static md400_pedal_action_t *md400_pedal_action_at(u8 preset, u8 page, u8 slot)
{
    u8 index;

    if ((slot < 1) || (slot > MD400_PEDAL_SLOT_COUNT) ||
        !md400_pedal_bank_load(preset, page)) {
        return NULL;
    }
    index = ((page % MD400_PEDAL_RECORD_PAGE_COUNT) *
             MD400_PEDAL_SLOT_COUNT) + slot - 1;
    return &md400_pedal_bank_record.action[index];
}

static void md400_pedal_action_assignment_get(
    const md400_pedal_action_t *action, u8 rotate,
    md400_assignment_t *assignment)
{
    if (!action || !assignment) {
        return;
    }
    md400_pedal_assignment_unpack(rotate ? &action->rotate : &action->press,
                                  assignment);
}

static void md400_pedal_action_assignment_set(
    md400_pedal_action_t *action, u8 rotate,
    const md400_assignment_t *assignment)
{
    u8 color;

    if (!action || !assignment) {
        return;
    }
    color = md400_pedal_action_color_get(action);
    md400_pedal_assignment_pack(rotate ? &action->rotate : &action->press,
                                assignment);
    if (rotate) {
        md400_pedal_action_color_set(action, color);
    }
    md400_pedal_record_dirty = 1;
}

static void md400_pedal_names_load(void)
{
    int n = syscfg_read(MD400_VM_PEDAL_NAMES_ID, &md400_pedal_names,
                       sizeof(md400_pedal_names));
    if (n != sizeof(md400_pedal_names) ||
        memcmp(md400_pedal_names.magic, "NFPN", 4) ||
        md400_pedal_names.version != 1 ||
        md400_pedal_names.checksum != md400_checksum8(
            (const u8 *)&md400_pedal_names, sizeof(md400_pedal_names) - 1)) {
        memset(&md400_pedal_names, 0, sizeof(md400_pedal_names));
        memcpy(md400_pedal_names.magic, "NFPN", 4);
        md400_pedal_names.version = 1;
    }
    md400_pedal_names_dirty = 0;
}

static int md400_pedal_names_save(void)
{
    if (!md400_pedal_names_dirty) return 1;
    md400_pedal_names.checksum = md400_checksum8(
        (const u8 *)&md400_pedal_names, sizeof(md400_pedal_names) - 1);
    if (syscfg_write(MD400_VM_PEDAL_NAMES_ID, &md400_pedal_names,
                    sizeof(md400_pedal_names)) != sizeof(md400_pedal_names) ||
        syscfg_read(MD400_VM_PEDAL_NAMES_ID, &md400_pedal_names_verify,
                    sizeof(md400_pedal_names_verify)) != sizeof(md400_pedal_names_verify) ||
        memcmp(&md400_pedal_names, &md400_pedal_names_verify, sizeof(md400_pedal_names))) return 0;
    md400_pedal_names_dirty = 0;
    return 1;
}

static u8 md400_pedal_header_checksum(const md400_pedal_bank_header_t *header)
{
    return md400_checksum8((const u8 *)header,
                           sizeof(md400_pedal_bank_header_t) - 1);
}

static u8 md400_pedal_header_valid(const md400_pedal_bank_header_t *header)
{
    return header &&
           !memcmp(header->magic, md400_pedal_bank_header_magic,
                   sizeof(md400_pedal_bank_header_magic)) &&
           (header->version == MD400_PEDAL_HEADER_VERSION) &&
           (header->active_preset < MD400_PRESET_COUNT) &&
           (header->foot_mode <= MD400_FOOT_MODE_PEDAL) &&
           (header->page < MD400_PEDAL_PAGE_COUNT) &&
           header->generation &&
           (header->checksum == md400_pedal_header_checksum(header));
}

static void md400_pedal_header_save_now(void)
{
    int ret;
    int verify_ret;

    memset(&md400_pedal_bank_header, 0, sizeof(md400_pedal_bank_header));
    memcpy(md400_pedal_bank_header.magic, md400_pedal_bank_header_magic,
           sizeof(md400_pedal_bank_header.magic));
    md400_pedal_bank_header.version = MD400_PEDAL_HEADER_VERSION;
    md400_pedal_bank_header.active_preset = md400_pedal_active_preset;
    md400_pedal_bank_header.foot_mode = md400_foot_mode;
    md400_pedal_bank_header.page = md400_pedal_page;
    md400_pedal_bank_header.generation = md400_pedal_generation;
    md400_pedal_bank_header.checksum =
        md400_pedal_header_checksum(&md400_pedal_bank_header);
    ret = syscfg_write(MD400_VM_PEDAL_BANK_HEADER_ID,
                       &md400_pedal_bank_header,
                       sizeof(md400_pedal_bank_header));
    verify_ret = syscfg_read(MD400_VM_PEDAL_BANK_HEADER_ID,
                             &md400_pedal_bank_header_verify,
                             sizeof(md400_pedal_bank_header_verify));
    if ((ret != sizeof(md400_pedal_bank_header)) ||
        (verify_ret != sizeof(md400_pedal_bank_header_verify)) ||
        !md400_pedal_header_valid(&md400_pedal_bank_header_verify) ||
        memcmp(&md400_pedal_bank_header, &md400_pedal_bank_header_verify,
               sizeof(md400_pedal_bank_header))) {
        printf("[NEXFOOT_PEDAL] header save fail ret=%d verify=%d\r\n",
               ret, verify_ret);
    }
}

static void md400_pedal_header_save(void)
{
    if (md400_persist_in_task) {
        md400_pedal_header_save_now();
        return;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_PEDAL_SAVE, 0, 0)) {
        printf("[MD400_PERSIST] pedal header queue failed\r\n");
    }
}

static void md400_pedal_header_load(void)
{
    int ret;
    md400_pedal_names_load();

    ret = syscfg_read(MD400_VM_PEDAL_BANK_HEADER_ID,
                      &md400_pedal_bank_header,
                      sizeof(md400_pedal_bank_header));
    if ((ret == sizeof(md400_pedal_bank_header)) &&
        md400_pedal_header_valid(&md400_pedal_bank_header)) {
        md400_pedal_active_preset =
            md400_pedal_bank_header.active_preset;
        md400_foot_mode = md400_pedal_bank_header.foot_mode;
        md400_pedal_page = md400_pedal_bank_header.page;
        md400_pedal_generation = md400_pedal_bank_header.generation;
    } else {
        md400_pedal_active_preset = 0;
        md400_foot_mode = MD400_FOOT_MODE_MIDI;
        md400_pedal_page = 0;
        md400_pedal_generation = 1;
    }
    md400_pedal_selected_preset = md400_pedal_active_preset;
    md400_pedal_bank_load(md400_pedal_active_preset, md400_pedal_page);
    printf("[NEXFOOT_PEDAL] init mode=%s preset=%u page=%u record_len=%u\r\n",
           md400_foot_mode == MD400_FOOT_MODE_PEDAL ? "PEDAL" : "MIDI",
           (unsigned)(md400_pedal_active_preset + 1),
           (unsigned)(md400_pedal_page + 1),
           (unsigned)sizeof(md400_pedal_bank_record));
}

static int md400_pedal_bank_factory_reset(void)
{
    memset(md400_pedal_names.name, 0, sizeof(md400_pedal_names.name));
    md400_pedal_names_dirty = 1;
    if (!md400_pedal_names_save()) return 0;
    md400_pedal_generation++;
    if (!md400_pedal_generation) {
        md400_pedal_generation = 1;
    }
    md400_pedal_active_preset = 0;
    md400_pedal_selected_preset = 0;
    md400_pedal_page = 0;
    md400_pedal_edit_slot = 1;
    md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
    md400_pedal_label_position = 0;
    md400_foot_mode = MD400_FOOT_MODE_MIDI;
    md400_pedal_record_loaded = 0;
    md400_pedal_record_dirty = 0;
    md400_pedal_header_save();
    md400_pedal_bank_load(0, 0);
    printf("[NEXFOOT_PEDAL] factory reset generation=%u\r\n",
           (unsigned)md400_pedal_generation);
    return 1;
}

static void md400_runtime_action_state_clear(void)
{
    memset(md400_encoder_note_toggle_state, 0, sizeof(md400_encoder_note_toggle_state));
    memset(md400_encoder_gesture_toggle_state, 0, sizeof(md400_encoder_gesture_toggle_state));
    memset(md400_combo_gesture_toggle_state, 0, sizeof(md400_combo_gesture_toggle_state));
    memset(md400_encoder_press_momentary_active, 0, sizeof(md400_encoder_press_momentary_active));
    memset(md400_encoder_short_momentary_active, 0, sizeof(md400_encoder_short_momentary_active));
    memset(md400_encoder_long_momentary_active, 0, sizeof(md400_encoder_long_momentary_active));
    memset(md400_combo_long_momentary_active, 0, sizeof(md400_combo_long_momentary_active));
}

static u8 md400_action_mask4(const u8 *state, u8 count)
{
    u8 mask = 0;
    u8 i;

    if (!state) {
        return 0;
    }
    if (count > 4) {
        count = 4;
    }
    for (i = 0; i < count; i++) {
        if (state[i]) {
            mask |= BIT(i);
        }
    }
    return mask;
}

static u8 md400_ui_performance_active(void)
{
    return (md400_ui_mode == MD400_UI_MODE_PERFORM) ||
           (md400_ui_mode == MD400_UI_MODE_PLAY);
}

static u8 md400_encoder_gesture_toggle_mask(u8 gesture_index)
{
    u8 mask = 0;
    u8 encoder;

    if (gesture_index >= MD400_ENCODER_GESTURE_COUNT) {
        return 0;
    }
    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        if (md400_encoder_gesture_toggle_state[encoder][gesture_index]) {
            mask |= BIT(encoder);
        }
    }
    return mask;
}

static u8 md400_combo_gesture_toggle_mask(u8 gesture_index)
{
    u8 mask = 0;
    u8 combo;

    if (gesture_index >= MD400_COMBO_GESTURE_COUNT) {
        return 0;
    }
    for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
        if (md400_combo_gesture_toggle_state[combo][gesture_index]) {
            mask |= BIT(combo);
        }
    }
    return mask;
}

static u8 md400_combo_mask_from_index(u8 index)
{
    static const u8 masks[MD400_COMBO_COUNT] = {0x03, 0x06, 0x0c};

    return (index < MD400_COMBO_COUNT) ? masks[index] : 0;
}

static u8 md400_toggle_led_mask(void)
{
    u8 mask = 0;
    u8 encoder;
    u8 gesture;
    u8 combo;

    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        for (gesture = 0; gesture < MD400_ENCODER_GESTURE_COUNT; gesture++) {
            if (md400_encoder_gesture_toggle_state[encoder][gesture]) {
                mask |= BIT(encoder);
                break;
            }
        }
    }

    for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
        for (gesture = 0; gesture < MD400_COMBO_GESTURE_COUNT; gesture++) {
            if (md400_combo_gesture_toggle_state[combo][gesture]) {
                mask |= md400_combo_mask_from_index(combo);
                break;
            }
        }
    }

    return mask & 0x0f;
}

static u8 md400_store_checksum(const md400_preset_store_t *store)
{
    return md400_checksum8((const u8 *)store, sizeof(md400_preset_store_t) - 1);
}

static u8 md400_store_valid(const md400_preset_store_t *store)
{
    if (memcmp(store->magic, md400_store_magic, sizeof(md400_store_magic))) {
        return 0;
    }
    if (store->version != MD400_PRESET_STORE_VERSION) {
        return 0;
    }
    if (store->active_preset >= MD400_PRESET_COUNT) {
        return 0;
    }
    return store->checksum == md400_store_checksum(store);
}

static u8 md400_store_v10_valid(const md400_preset_store_t *store)
{
    if (memcmp(store->magic, md400_store_magic, sizeof(md400_store_magic))) {
        return 0;
    }
    if (store->version != MD400_PRESET_STORE_VERSION_V10) {
        return 0;
    }
    if (store->active_preset >= MD400_PRESET_COUNT) {
        return 0;
    }
    return store->checksum == md400_store_checksum(store);
}

static u8 md400_store_v8_valid(const md400_preset_store_t *store)
{
    if (memcmp(store->magic, md400_store_magic, sizeof(md400_store_magic))) {
        return 0;
    }
    if (store->version != MD400_PRESET_STORE_VERSION_V8) {
        return 0;
    }
    if (store->active_preset >= MD400_PRESET_COUNT) {
        return 0;
    }
    return store->checksum == md400_store_checksum(store);
}

static u8 md400_store_v9_valid(const md400_preset_store_t *store)
{
    if (memcmp(store->magic, md400_store_magic, sizeof(md400_store_magic))) {
        return 0;
    }
    if (store->version != MD400_PRESET_STORE_VERSION_V9) {
        return 0;
    }
    if (store->active_preset >= MD400_PRESET_COUNT) {
        return 0;
    }
    return store->checksum == md400_store_checksum(store);
}

static u16 md400_legacy_store_vm_id(u8 slot)
{
    return slot ? MD400_VM_LEGACY_STORE_ID_B : MD400_VM_LEGACY_STORE_ID;
}

static u16 md400_seq_get(const u8 reserved[2])
{
    return (u16)reserved[0] | ((u16)reserved[1] << 8);
}

static void md400_seq_set(u8 reserved[2], u16 seq)
{
    reserved[0] = seq & 0xff;
    reserved[1] = (seq >> 8) & 0xff;
}

static u8 md400_store_seq_newer(u16 candidate, u16 current)
{
    return (candidate != current) && ((u16)(candidate - current) < 0x8000);
}

static u16 md400_store_seq(const md400_preset_store_t *store)
{
    return md400_seq_get(store->reserved);
}

static u8 md400_header_checksum(const md400_preset_header_t *header)
{
    return md400_checksum8((const u8 *)header, sizeof(md400_preset_header_t) - 1);
}

static u8 md400_header_valid(const md400_preset_header_t *header)
{
    if (memcmp(header->magic, md400_header_magic, sizeof(md400_header_magic))) {
        return 0;
    }
    if ((header->version != MD400_PRESET_STORE_VERSION) &&
        (header->version != MD400_PRESET_STORE_VERSION_V10) &&
        (header->version != MD400_PRESET_STORE_VERSION_V9) &&
        (header->version != MD400_PRESET_STORE_VERSION_V8) &&
        (header->version != MD400_PRESET_STORE_VERSION_V7) &&
        (header->version != MD400_PRESET_STORE_VERSION_V6) &&
        (header->version != MD400_PRESET_STORE_VERSION_V5) &&
        (header->version != MD400_PRESET_STORE_VERSION_V4) &&
        (header->version != MD400_PRESET_STORE_VERSION_V3)) {
        return 0;
    }
    if (header->active_preset >= MD400_PRESET_COUNT) {
        return 0;
    }
    return header->checksum == md400_header_checksum(header);
}

static u8 md400_group_checksum(const md400_preset_group_t *group)
{
    return md400_checksum8((const u8 *)group, sizeof(md400_preset_group_t) - 1);
}

static u8 md400_group_valid(const md400_preset_group_t *group, u8 group_index)
{
    if (memcmp(group->magic, md400_group_magic, sizeof(md400_group_magic))) {
        return 0;
    }
    if (group->version != MD400_PRESET_STORE_VERSION_V5) {
        return 0;
    }
    if (group->group_index != group_index) {
        return 0;
    }
    return group->checksum == md400_group_checksum(group);
}

static u8 md400_preset_record_checksum(const md400_preset_record_t *record)
{
    return md400_checksum8((const u8 *)record, sizeof(md400_preset_record_t) - 1);
}

static u8 md400_preset_record_valid(const md400_preset_record_t *record, u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_checksum(record);
}

static u8 md400_preset_record_v10_checksum(const md400_preset_record_v10_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_preset_record_v10_t) - 1);
}

static u8 md400_preset_record_v10_valid(const md400_preset_record_v10_t *record,
                                        u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION_V10) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_v10_checksum(record);
}

static u8 md400_preset_record_v8_valid(const md400_preset_record_v10_t *record,
                                       u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION_V8) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_v10_checksum(record);
}

static u8 md400_preset_record_v9_valid(const md400_preset_record_v10_t *record,
                                       u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION_V9) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_v10_checksum(record);
}

static u8 md400_preset_record_v7_checksum(const md400_preset_record_v7_t *record)
{
    return md400_checksum8((const u8 *)record, sizeof(md400_preset_record_v7_t) - 1);
}

static u8 md400_preset_record_v7_valid(const md400_preset_record_v7_t *record, u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION_V7) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_v7_checksum(record);
}

static u8 md400_preset_record_v6_checksum(const md400_preset_record_v6_t *record)
{
    return md400_checksum8((const u8 *)record, sizeof(md400_preset_record_v6_t) - 1);
}

static u8 md400_preset_record_v6_valid(const md400_preset_record_v6_t *record, u8 preset_index)
{
    if (memcmp(record->magic, md400_record_magic, sizeof(md400_record_magic))) {
        return 0;
    }
    if (record->version != MD400_PRESET_STORE_VERSION_V6) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_preset_record_v6_checksum(record);
}

static u8 md400_group_v4_checksum(const md400_preset_group_v4_t *group)
{
    return md400_checksum8((const u8 *)group, sizeof(md400_preset_group_v4_t) - 1);
}

static u8 md400_group_v4_valid(const md400_preset_group_v4_t *group, u8 group_index)
{
    if (memcmp(group->magic, md400_group_magic, sizeof(md400_group_magic))) {
        return 0;
    }
    if (group->version != MD400_PRESET_STORE_VERSION_V4) {
        return 0;
    }
    if (group->group_index != group_index) {
        return 0;
    }
    return group->checksum == md400_group_v4_checksum(group);
}

static u8 md400_group_v3_checksum(const md400_preset_group_v3_t *group)
{
    return md400_checksum8((const u8 *)group, sizeof(md400_preset_group_v3_t) - 1);
}

static u8 md400_group_v3_valid(const md400_preset_group_v3_t *group, u8 group_index)
{
    if (memcmp(group->magic, md400_group_magic, sizeof(md400_group_magic))) {
        return 0;
    }
    if (group->version != MD400_PRESET_STORE_VERSION_V3) {
        return 0;
    }
    if (group->group_index != group_index) {
        return 0;
    }
    return group->checksum == md400_group_v3_checksum(group);
}

static u16 md400_header_vm_id(u8 slot)
{
    return slot ? MD400_VM_PRESET_HEADER_ID_B : MD400_VM_PRESET_HEADER_ID;
}

static u16 md400_group_record_vm_id(u8 group_index, u8 slot)
{
    if (group_index >= MD400_PRESET_GROUP_COUNT) {
        return 0;
    }
    return md400_group_vm_id[slot ? 1 : 0][group_index];
}

static u16 md400_play_label_vm_id(u8 preset_index)
{
    if (preset_index >= MD400_PRESET_COUNT) {
        preset_index = 0;
    }
    return MD400_VM_PLAY_LABEL_FIRST_ID + preset_index;
}

static u8 md400_play_label_checksum(const md400_play_label_record_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_play_label_record_t) - 1);
}

static u8 md400_play_label_record_valid(const md400_play_label_record_t *record,
                                        u8 preset_index)
{
    if (memcmp(record->magic, md400_play_label_magic,
               sizeof(md400_play_label_magic))) {
        return 0;
    }
    if (record->version != MD400_PLAY_LABEL_STORE_VERSION) {
        return 0;
    }
    if (record->preset_index != preset_index) {
        return 0;
    }
    return record->checksum == md400_play_label_checksum(record);
}

static void md400_play_label_default_text(u8 action, u8 control,
                                          u8 *label)
{
    const char *prefix = "KNOB";
    u8 len = 0;
    u8 i;

    if (!label) {
        return;
    }

    if (action == MD400_EVENT_ENCODER_PRESS) {
        prefix = "PRESS";
    } else if (action == MD400_EVENT_ENCODER_SHORT) {
        prefix = "SHORT";
    } else if (action == MD400_EVENT_ENCODER_LONG) {
        prefix = "LONG";
    } else if (action == MD400_EVENT_ENCODER_DOUBLE) {
        prefix = "DOUBLE";
    }

    memset(label, 0, MD400_PLAY_LABEL_LEN);
    for (i = 0; prefix[i] && (len < (MD400_PLAY_LABEL_LEN - 1)); i++) {
        label[len++] = prefix[i];
    }
    if ((control >= 1) && (control <= MD400_ENCODER_COUNT) &&
        (len < MD400_PLAY_LABEL_LEN)) {
        label[len++] = '0' + control;
    }
}

static u8 md400_play_label_slot(u8 action, u8 control)
{
    u8 base;

    if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
        return 0xff;
    }

    if (action == MD400_PLAY_LABEL_ROTATE_ACTION) {
        base = 0;
    } else if (action == MD400_EVENT_ENCODER_PRESS) {
        base = MD400_ENCODER_COUNT;
    } else if (action == MD400_EVENT_ENCODER_SHORT) {
        base = MD400_ENCODER_COUNT * 2;
    } else if (action == MD400_EVENT_ENCODER_LONG) {
        base = MD400_ENCODER_COUNT * 3;
    } else if (action == MD400_EVENT_ENCODER_DOUBLE) {
        base = MD400_ENCODER_COUNT * 4;
    } else {
        return 0xff;
    }

    return base + control - 1;
}

static void md400_play_label_set_defaults(u8 labels[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN])
{
    u8 control;

    if (!labels) {
        return;
    }
    memset(labels, 0, MD400_PLAY_LABEL_SLOT_COUNT * MD400_PLAY_LABEL_LEN);
    for (control = 1; control <= MD400_ENCODER_COUNT; control++) {
        md400_play_label_default_text(MD400_PLAY_LABEL_ROTATE_ACTION, control,
                                      labels[md400_play_label_slot(MD400_PLAY_LABEL_ROTATE_ACTION, control)]);
        md400_play_label_default_text(MD400_EVENT_ENCODER_PRESS, control,
                                      labels[md400_play_label_slot(MD400_EVENT_ENCODER_PRESS, control)]);
        md400_play_label_default_text(MD400_EVENT_ENCODER_SHORT, control,
                                      labels[md400_play_label_slot(MD400_EVENT_ENCODER_SHORT, control)]);
        md400_play_label_default_text(MD400_EVENT_ENCODER_LONG, control,
                                      labels[md400_play_label_slot(MD400_EVENT_ENCODER_LONG, control)]);
        md400_play_label_default_text(MD400_EVENT_ENCODER_DOUBLE, control,
                                      labels[md400_play_label_slot(MD400_EVENT_ENCODER_DOUBLE, control)]);
    }
}

static void md400_play_label_record_build(u8 preset_index,
                                          md400_play_label_record_t *record,
                                          const u8 labels[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN])
{
    memset(record, 0, sizeof(*record));
    memcpy(record->magic, md400_play_label_magic, sizeof(record->magic));
    record->version = MD400_PLAY_LABEL_STORE_VERSION;
    record->preset_index = preset_index;
    memcpy(record->label, labels,
           MD400_PLAY_LABEL_SLOT_COUNT * MD400_PLAY_LABEL_LEN);
    record->checksum = md400_play_label_checksum(record);
}

static u8 md400_play_label_record_read(u8 preset_index,
                                       u8 labels[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN])
{
    int ret;

    if ((preset_index >= MD400_PRESET_COUNT) || !labels) {
        return 0;
    }
    ret = syscfg_read(md400_play_label_vm_id(preset_index),
                      &md400_play_label_read_buf,
                      sizeof(md400_play_label_read_buf));
    if ((ret == sizeof(md400_play_label_read_buf)) &&
        md400_play_label_record_valid(&md400_play_label_read_buf,
                                      preset_index)) {
        memcpy(labels, md400_play_label_read_buf.label,
               MD400_PLAY_LABEL_SLOT_COUNT * MD400_PLAY_LABEL_LEN);
        return 1;
    }
    return 0;
}

static u8 md400_play_label_record_write(u8 preset_index,
                                        const u8 labels[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN])
{
    int ret;
    int verify_ret;
    u8 attempt;
    u16 vm_id;

    if ((preset_index >= MD400_PRESET_COUNT) || !labels) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_play_label_record_build(preset_index, &md400_play_label_record_buf,
                                  labels);
    vm_id = md400_play_label_vm_id(preset_index);
    ret = syscfg_read(vm_id, &md400_play_label_read_buf,
                      sizeof(md400_play_label_read_buf));
    if ((ret == sizeof(md400_play_label_read_buf)) &&
        !memcmp(&md400_play_label_record_buf, &md400_play_label_read_buf,
                sizeof(md400_play_label_record_buf))) {
        return MD400_NB_STATUS_OK;
    }
    for (attempt = 1; attempt <= MD400_VM_WRITE_TRIES; attempt++) {
        ret = syscfg_write(vm_id, &md400_play_label_record_buf,
                           sizeof(md400_play_label_record_buf));
        verify_ret = syscfg_read(vm_id, &md400_play_label_read_buf,
                                 sizeof(md400_play_label_read_buf));
        if ((ret == sizeof(md400_play_label_record_buf)) &&
            (verify_ret == sizeof(md400_play_label_read_buf)) &&
            !memcmp(&md400_play_label_record_buf, &md400_play_label_read_buf,
                    sizeof(md400_play_label_record_buf))) {
            return MD400_NB_STATUS_OK;
        }
        md400_dbg_vm_fail(MD400_DBG_VM_LABEL, vm_id, ret, verify_ret,
                          md400_play_label_record_valid(&md400_play_label_read_buf,
                                                        preset_index),
                          !memcmp(&md400_play_label_record_buf,
                                  &md400_play_label_read_buf,
                                  sizeof(md400_play_label_record_buf)));
        printf("[MD400_CFG] play label save try=%u preset=%u id=%u ret=%d verify=%d\r\n",
               (unsigned)attempt, (unsigned)(preset_index + 1),
               (unsigned)vm_id, ret, verify_ret);
    }
    return MD400_NB_STATUS_FLASH_ERROR;
}

static void md400_play_label_load_active(void)
{
    if ((md400_active_preset >= MD400_PRESET_COUNT) ||
        !md400_play_label_record_read(md400_active_preset, md400_play_label)) {
        md400_play_label_set_defaults(md400_play_label);
    }
}

static u8 md400_play_label_len(const u8 *label)
{
    u8 i;

    if (!label) {
        return 0;
    }
    for (i = 0; i < MD400_PLAY_LABEL_LEN; i++) {
        if (!label[i]) {
            return i;
        }
    }
    return MD400_PLAY_LABEL_LEN;
}

static void md400_play_label_to_cstr(u8 action, u8 control,
                                     char *out, u8 out_len)
{
    static const char exp_label[] = "EXP";
    u8 slot;
    u8 len;
    u8 i;
    u8 fallback[MD400_PLAY_LABEL_LEN];
    const u8 *label;

    if (!out || !out_len) {
        return;
    }
    memset(out, 0, out_len);
    if (action == MD400_EVENT_PEDAL_VALUE) {
        for (i = 0; exp_label[i] && ((i + 1) < out_len); i++) {
            out[i] = exp_label[i];
        }
        return;
    }
    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        md400_pedal_action_t *pedal_action =
            md400_pedal_action_at(md400_pedal_active_preset,
                                  md400_pedal_page, control);
        if (pedal_action) {
            len = MD400_PEDAL_LABEL_LEN;
            while (len && !pedal_action->label[len - 1]) {
                len--;
            }
            if (len >= out_len) {
                len = out_len - 1;
            }
            for (i = 0; i < len; i++) {
                u8 ch = pedal_action->label[i] & 0x7f;
                out[i] = ((ch >= 0x20) && (ch <= 0x7e)) ? ch : ' ';
            }
        }
        return;
    }
    slot = md400_play_label_slot(action, control);
    if (slot == 0xff) {
        return;
    }

    label = md400_play_label[slot];
    len = md400_play_label_len(label);
    if (!len) {
        md400_play_label_default_text(action, control, fallback);
        label = fallback;
        len = md400_play_label_len(label);
    }
    if (len >= out_len) {
        len = out_len - 1;
    }
    for (i = 0; i < len; i++) {
        u8 ch = label[i] & 0x7f;
        out[i] = ((ch >= 0x20) && (ch <= 0x7e)) ? ch : ' ';
    }
}

static void md400_play_label_source(u8 preset_index,
                                    u8 labels[MD400_PLAY_LABEL_SLOT_COUNT][MD400_PLAY_LABEL_LEN])
{
    if (!labels) {
        return;
    }
    if (preset_index == md400_active_preset) {
        memcpy(labels, md400_play_label,
               MD400_PLAY_LABEL_SLOT_COUNT * MD400_PLAY_LABEL_LEN);
        return;
    }
    if (!md400_play_label_record_read(preset_index, labels)) {
        md400_play_label_set_defaults(labels);
    }
}

static u8 md400_play_label_factory_reset(void)
{
    u8 preset;
    u8 status;
    int ret;

    md400_play_label_set_defaults(md400_play_label_work);
    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        ret = syscfg_read(md400_play_label_vm_id(preset),
                          &md400_play_label_read_buf,
                          sizeof(md400_play_label_read_buf));
        if ((ret != (int)sizeof(md400_play_label_read_buf)) ||
            !md400_play_label_record_valid(&md400_play_label_read_buf, preset)) {
            continue;
        }
        status = md400_play_label_record_write(preset, md400_play_label_work);
        if (status != MD400_NB_STATUS_OK) {
            return status;
        }
    }
    md400_play_label_load_active();
    return MD400_NB_STATUS_OK;
}

static u16 md400_sysex_template_vm_id(u8 preset_index)
{
    return MD400_VM_SYSEX_TEMPLATE_FIRST_ID + preset_index;
}

static u8 md400_sysex_template_checksum(const md400_sysex_template_record_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_sysex_template_record_t) - 1);
}

static void md400_sysex_template_set_defaults(md400_sysex_template_t *templates)
{
    u8 slot;

    if (!templates) {
        return;
    }
    memset(templates, 0,
           sizeof(md400_sysex_template_t) * MD400_SYSEX_TEMPLATE_COUNT);
    for (slot = 0; slot < MD400_SYSEX_TEMPLATE_COUNT; slot++) {
        templates[slot].value_index = MD400_SYSEX_TEMPLATE_VALUE_NONE;
    }
}

static u8 md400_sysex_template_record_valid(const md400_sysex_template_record_t *record,
                                            u8 preset_index)
{
    u8 slot;
    u8 i;

    if (memcmp(record->magic, md400_sysex_template_magic,
               sizeof(md400_sysex_template_magic))) {
        return 0;
    }
    if ((record->version != MD400_SYSEX_TEMPLATE_STORE_VERSION) ||
        (record->preset_index != preset_index)) {
        return 0;
    }
    for (slot = 0; slot < MD400_SYSEX_TEMPLATE_COUNT; slot++) {
        u8 len = record->slot[slot].len;
        u8 value_index = record->slot[slot].value_index;
        if ((len > MD400_SYSEX_TEMPLATE_MAX_BODY) ||
            ((value_index != MD400_SYSEX_TEMPLATE_VALUE_NONE) &&
             (value_index >= len))) {
            return 0;
        }
        for (i = 0; i < len; i++) {
            if (record->slot[slot].body[i] > 0x7f) {
                return 0;
            }
        }
    }
    return record->checksum == md400_sysex_template_checksum(record);
}

static void md400_sysex_template_record_build(u8 preset_index,
                                              md400_sysex_template_record_t *record,
                                              const md400_sysex_template_t *templates)
{
    memset(record, 0, sizeof(*record));
    memcpy(record->magic, md400_sysex_template_magic, sizeof(record->magic));
    record->version = MD400_SYSEX_TEMPLATE_STORE_VERSION;
    record->preset_index = preset_index;
    if (templates) {
        memcpy(record->slot, templates,
               sizeof(md400_sysex_template_t) * MD400_SYSEX_TEMPLATE_COUNT);
    } else {
        md400_sysex_template_set_defaults(record->slot);
    }
    record->checksum = md400_sysex_template_checksum(record);
}

static u8 md400_sysex_template_buf_alloc(void)
{
    if (!md400_template_record_buf) {
        md400_template_record_buf = malloc(sizeof(md400_sysex_template_record_t));
    }
    if (!md400_template_record_buf) {
        printf("[MD400_CFG] sysex template malloc fail size=%u\r\n",
               (unsigned)sizeof(md400_sysex_template_record_t));
        return 0;
    }
    return 1;
}

static void md400_template_record_buf_release(void)
{
    if (md400_template_record_buf) {
        free(md400_template_record_buf);
        md400_template_record_buf = NULL;
    }
}

static u8 md400_sysex_template_load_to_buf(u8 preset_index)
{
    int ret;

    if ((preset_index >= MD400_PRESET_COUNT) ||
        !md400_sysex_template_buf_alloc()) {
        return 0;
    }
    ret = syscfg_read(md400_sysex_template_vm_id(preset_index),
                      md400_sysex_template_buf,
                      sizeof(*md400_sysex_template_buf));
    if ((ret == (int)sizeof(*md400_sysex_template_buf)) &&
        md400_sysex_template_record_valid(md400_sysex_template_buf,
                                          preset_index)) {
        return 1;
    }

    md400_sysex_template_record_build(preset_index, md400_sysex_template_buf,
                                      NULL);
    return 1;
}

static u8 md400_sysex_template_record_write(u8 preset_index,
                                            const md400_sysex_template_t *templates)
{
    int ret;
    int verify_ret;

    if ((preset_index >= MD400_PRESET_COUNT) ||
        !md400_sysex_template_buf_alloc()) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (templates == md400_sysex_template_buf->slot) {
        memcpy(md400_sysex_template_buf->magic, md400_sysex_template_magic,
               sizeof(md400_sysex_template_buf->magic));
        md400_sysex_template_buf->version = MD400_SYSEX_TEMPLATE_STORE_VERSION;
        md400_sysex_template_buf->preset_index = preset_index;
        md400_sysex_template_buf->reserved[0] = 0;
        md400_sysex_template_buf->reserved[1] = 0;
        md400_sysex_template_buf->checksum =
            md400_sysex_template_checksum(md400_sysex_template_buf);
    } else {
        md400_sysex_template_record_build(preset_index,
                                          md400_sysex_template_buf,
                                          templates);
    }
    ret = syscfg_write(md400_sysex_template_vm_id(preset_index),
                       md400_sysex_template_buf,
                       sizeof(*md400_sysex_template_buf));
    verify_ret = syscfg_read(md400_sysex_template_vm_id(preset_index),
                             md400_sysex_template_buf,
                             sizeof(*md400_sysex_template_buf));
    if ((ret == sizeof(*md400_sysex_template_buf)) &&
        (verify_ret == sizeof(*md400_sysex_template_buf)) &&
        md400_sysex_template_record_valid(md400_sysex_template_buf,
                                          preset_index) &&
        !memcmp(md400_sysex_template_buf->slot, templates,
                sizeof(md400_sysex_template_t) * MD400_SYSEX_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_OK;
    }

    md400_dbg_vm_fail(MD400_DBG_VM_SYSEX,
                      md400_sysex_template_vm_id(preset_index),
                      ret, verify_ret,
                      md400_sysex_template_record_valid(md400_sysex_template_buf,
                                                        preset_index),
                      0);
    printf("[MD400_CFG] sysex template save fail preset=%u ret=%d verify=%d\r\n",
           (unsigned)(preset_index + 1), ret, verify_ret);
    return MD400_NB_STATUS_FLASH_ERROR;
}

static u8 md400_sysex_template_factory_reset(void)
{
    u8 preset;
    u8 status;
    int ret;

    if (!md400_sysex_template_buf_alloc()) {
        return MD400_NB_STATUS_OK;
    }
    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        ret = syscfg_read(md400_sysex_template_vm_id(preset),
                          md400_sysex_template_buf,
                          sizeof(*md400_sysex_template_buf));
        if ((ret != (int)sizeof(*md400_sysex_template_buf)) ||
            !md400_sysex_template_record_valid(md400_sysex_template_buf,
                                               preset)) {
            continue;
        }
        md400_sysex_template_set_defaults(md400_sysex_template_buf->slot);
        status = md400_sysex_template_record_write(preset,
                                                   md400_sysex_template_buf->slot);
        if (status != MD400_NB_STATUS_OK) {
            return status;
        }
    }
    return MD400_NB_STATUS_OK;
}

static u16 md400_custom_template_vm_id(u8 preset_index)
{
    return MD400_VM_CUSTOM_TEMPLATE_FIRST_ID + preset_index;
}

static u8 md400_custom_template_checksum(const md400_custom_template_record_t *record)
{
    return md400_checksum8((const u8 *)record,
                           sizeof(md400_custom_template_record_t) - 1);
}

static void md400_custom_template_set_defaults(md400_custom_template_t *templates)
{
    u8 slot;

    if (!templates) {
        return;
    }
    memset(templates, 0,
           sizeof(md400_custom_template_t) * MD400_CUSTOM_TEMPLATE_COUNT);
    for (slot = 0; slot < MD400_CUSTOM_TEMPLATE_COUNT; slot++) {
        templates[slot].value_index = MD400_CUSTOM_TEMPLATE_VALUE_NONE;
    }
}

static u8 md400_custom_template_record_valid(const md400_custom_template_record_t *record,
                                             u8 preset_index)
{
    u8 slot;

    if (memcmp(record->magic, md400_custom_template_magic,
               sizeof(md400_custom_template_magic))) {
        return 0;
    }
    if ((record->version != MD400_CUSTOM_TEMPLATE_STORE_VERSION) ||
        (record->preset_index != preset_index)) {
        return 0;
    }
    for (slot = 0; slot < MD400_CUSTOM_TEMPLATE_COUNT; slot++) {
        u8 len = record->slot[slot].len;
        u8 value_index = record->slot[slot].value_index;
        if ((len > MD400_CUSTOM_TEMPLATE_MAX_BODY) ||
            ((value_index != MD400_CUSTOM_TEMPLATE_VALUE_NONE) &&
             (value_index >= len))) {
            return 0;
        }
    }
    return record->checksum == md400_custom_template_checksum(record);
}

static void md400_custom_template_record_build(u8 preset_index,
                                               md400_custom_template_record_t *record,
                                               const md400_custom_template_t *templates)
{
    memset(record, 0, sizeof(*record));
    memcpy(record->magic, md400_custom_template_magic, sizeof(record->magic));
    record->version = MD400_CUSTOM_TEMPLATE_STORE_VERSION;
    record->preset_index = preset_index;
    if (templates) {
        memcpy(record->slot, templates,
               sizeof(md400_custom_template_t) * MD400_CUSTOM_TEMPLATE_COUNT);
    } else {
        md400_custom_template_set_defaults(record->slot);
    }
    record->checksum = md400_custom_template_checksum(record);
}

static u8 md400_custom_template_buf_alloc(void)
{
    if (!md400_template_record_buf) {
        md400_template_record_buf = malloc(sizeof(md400_custom_template_record_t));
    }
    if (!md400_template_record_buf) {
        printf("[MD400_CFG] custom template malloc fail size=%u\r\n",
               (unsigned)sizeof(md400_custom_template_record_t));
        return 0;
    }
    return 1;
}

static u8 md400_custom_template_load_to_buf(u8 preset_index)
{
    int ret;

    if ((preset_index >= MD400_PRESET_COUNT) ||
        !md400_custom_template_buf_alloc()) {
        return 0;
    }
    ret = syscfg_read(md400_custom_template_vm_id(preset_index),
                      md400_custom_template_buf,
                      sizeof(*md400_custom_template_buf));
    if ((ret == (int)sizeof(*md400_custom_template_buf)) &&
        md400_custom_template_record_valid(md400_custom_template_buf,
                                           preset_index)) {
        return 1;
    }

    md400_custom_template_record_build(preset_index, md400_custom_template_buf,
                                       NULL);
    return 1;
}

static u8 md400_custom_template_record_write(u8 preset_index,
                                             const md400_custom_template_t *templates)
{
    int ret;
    int verify_ret;

    if ((preset_index >= MD400_PRESET_COUNT) ||
        !md400_custom_template_buf_alloc()) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (templates == md400_custom_template_buf->slot) {
        memcpy(md400_custom_template_buf->magic, md400_custom_template_magic,
               sizeof(md400_custom_template_buf->magic));
        md400_custom_template_buf->version = MD400_CUSTOM_TEMPLATE_STORE_VERSION;
        md400_custom_template_buf->preset_index = preset_index;
        md400_custom_template_buf->reserved[0] = 0;
        md400_custom_template_buf->reserved[1] = 0;
        md400_custom_template_buf->checksum =
            md400_custom_template_checksum(md400_custom_template_buf);
    } else {
        md400_custom_template_record_build(preset_index,
                                           md400_custom_template_buf,
                                           templates);
    }
    ret = syscfg_write(md400_custom_template_vm_id(preset_index),
                       md400_custom_template_buf,
                       sizeof(*md400_custom_template_buf));
    verify_ret = syscfg_read(md400_custom_template_vm_id(preset_index),
                             md400_custom_template_buf,
                             sizeof(*md400_custom_template_buf));
    if ((ret == sizeof(*md400_custom_template_buf)) &&
        (verify_ret == sizeof(*md400_custom_template_buf)) &&
        md400_custom_template_record_valid(md400_custom_template_buf,
                                           preset_index) &&
        !memcmp(md400_custom_template_buf->slot, templates,
                sizeof(md400_custom_template_t) * MD400_CUSTOM_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_OK;
    }

    md400_dbg_vm_fail(MD400_DBG_VM_CUSTOM,
                      md400_custom_template_vm_id(preset_index),
                      ret, verify_ret,
                      md400_custom_template_record_valid(md400_custom_template_buf,
                                                         preset_index),
                      0);
    printf("[MD400_CFG] custom template save fail preset=%u ret=%d verify=%d\r\n",
           (unsigned)(preset_index + 1), ret, verify_ret);
    return MD400_NB_STATUS_FLASH_ERROR;
}

static u8 md400_custom_template_factory_reset(void)
{
    u8 preset;
    u8 status;
    int ret;

    if (!md400_custom_template_buf_alloc()) {
        return MD400_NB_STATUS_OK;
    }
    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        ret = syscfg_read(md400_custom_template_vm_id(preset),
                          md400_custom_template_buf,
                          sizeof(*md400_custom_template_buf));
        if ((ret != (int)sizeof(*md400_custom_template_buf)) ||
            !md400_custom_template_record_valid(md400_custom_template_buf,
                                                preset)) {
            continue;
        }
        md400_custom_template_set_defaults(md400_custom_template_buf->slot);
        status = md400_custom_template_record_write(preset,
                                                    md400_custom_template_buf->slot);
        if (status != MD400_NB_STATUS_OK) {
            return status;
        }
    }
    return MD400_NB_STATUS_OK;
}

static void md400_assignment_default(md400_assignment_t *assignment, u8 encoder)
{
    memset(assignment, 0, sizeof(*assignment));
    assignment->type = MD400_MIDI_TYPE_CC;
    assignment->channel = 0;
    assignment->data1 = encoder;
    assignment->value_min = 0;
    assignment->value_max = 0x7f;
    assignment->value_default = 0;
    assignment->behavior = MD400_BEHAVIOR_ABSOLUTE;
    assignment->target_mask = md400_available_target_mask();
    assignment->sysex_id = 0;
}

static void md400_assignment_pedal_default(md400_assignment_t *assignment)
{
    md400_assignment_default(assignment, 11);
}

static void md400_assignment_encoder_press_default(md400_assignment_t *assignment, u8 encoder)
{
    md400_assignment_default(assignment, MD400_ENCODER_PRESS_CC_BASE + encoder);
    assignment->value_default = 0x7f;
    assignment->behavior = MD400_BEHAVIOR_MOMENTARY;
}

static void md400_assignment_off(md400_assignment_t *assignment)
{
    memset(assignment, 0, sizeof(*assignment));
    assignment->type = MD400_MIDI_TYPE_OFF;
    assignment->channel = 0;
    assignment->value_max = 0x7f;
    assignment->target_mask = md400_available_target_mask();
}

static void md400_preset_default_label(md400_preset_t *preset, u8 preset_index)
{
    if (!preset) {
        return;
    }
    memset(preset->label, 0, sizeof(preset->label));
    preset->label[0] = 'P';
    preset->label[1] = '0' + ((preset_index + 1) / 10);
    preset->label[2] = '0' + ((preset_index + 1) % 10);
}

static void md400_preset_default_led(md400_preset_t *preset)
{
    if (!preset) {
        return;
    }
    preset->led_mode = MD400_LED_MODE_AUTO;
    preset->led_mask = 0;
}

static void md400_preset_combo_gestures_off(md400_preset_t *preset)
{
    u8 combo;
    u8 gesture;

    if (!preset) {
        return;
    }

    for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
        for (gesture = 0; gesture < MD400_COMBO_GESTURE_COUNT; gesture++) {
            md400_assignment_off(&preset->combo_gesture[combo][gesture]);
        }
    }
}

static void md400_preset_encoder_press_gestures_default(md400_preset_t *preset)
{
    u8 encoder;

    if (!preset) {
        return;
    }

    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        md400_assignment_encoder_press_default(&preset->encoder_gesture[encoder][MD400_ENCODER_GESTURE_COUNT_V7],
                                               encoder);
    }
}

static void md400_preset_encoder_press_gestures_default_if_off(md400_preset_t *preset)
{
    u8 encoder;
    md400_assignment_t *assignment;

    if (!preset) {
        return;
    }

    for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
        assignment = &preset->encoder_gesture[encoder][MD400_ENCODER_GESTURE_COUNT_V7];
        if (assignment->type == MD400_MIDI_TYPE_OFF) {
            md400_assignment_encoder_press_default(assignment, encoder);
        }
    }
}

static void md400_store_rotate_range_defaults(void)
{
    u8 preset;

    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        md400_preset_rotate_range_defaults(&md400_store.preset[preset]);
    }
}

static u8 md400_store_fix_legacy_rotate_defaults(void)
{
    u8 preset;
    u8 encoder;
    u8 changed = 0;
    md400_assignment_t *assignment;

    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
            assignment = &md400_store.preset[preset].encoder[encoder];
            if ((assignment->value_min == 0x7f) &&
                (assignment->value_max == 0) &&
                (assignment->value_default == 0x7f)) {
                md400_assignment_rotate_range_default(assignment);
                changed++;
            }
        }
    }

    if (changed) {
        printf("[MD400_CFG] fixed legacy rotate defaults count=%u\r\n",
               (unsigned)changed);
    }
    return changed;
}

static void md400_store_encoder_press_gestures_default_if_off(void)
{
    u8 preset;

    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        md400_preset_encoder_press_gestures_default_if_off(&md400_store.preset[preset]);
    }
}

static void md400_preset_copy_v7(md400_preset_t *dst, const md400_preset_v7_t *src)
{
    if (!dst || !src) {
        return;
    }
    memset(dst, 0, sizeof(*dst));
    memcpy(dst->label, src->label, sizeof(dst->label));
    dst->led_mode = src->led_mode;
    dst->led_mask = src->led_mask;
    memcpy(dst->encoder, src->encoder, sizeof(dst->encoder));
    md400_preset_rotate_range_defaults(dst);
    memcpy(dst->encoder_gesture, src->encoder_gesture, sizeof(src->encoder_gesture));
    md400_preset_encoder_press_gestures_default(dst);
    memcpy(dst->combo_gesture, src->combo_gesture, sizeof(dst->combo_gesture));
    memcpy(&dst->pedal, &src->pedal, sizeof(dst->pedal));
}

static void md400_preset_copy_v8(md400_preset_t *dst, const md400_preset_t *src)
{
    if (!dst || !src) {
        return;
    }
    memcpy(dst, src, sizeof(*dst));
    md400_preset_rotate_range_defaults(dst);
    md400_preset_encoder_press_gestures_default_if_off(dst);
}

static void md400_preset_copy_v9(md400_preset_t *dst, const md400_preset_t *src)
{
    if (!dst || !src) {
        return;
    }
    memcpy(dst, src, sizeof(*dst));
    md400_preset_rotate_range_defaults(dst);
}

static void md400_preset_copy_v6(md400_preset_t *dst, const md400_preset_v6_t *src)
{
    if (!dst || !src) {
        return;
    }
    memset(dst, 0, sizeof(*dst));
    memcpy(dst->label, src->label, sizeof(dst->label));
    dst->led_mode = src->led_mode;
    dst->led_mask = src->led_mask;
    memcpy(dst->encoder, src->encoder, sizeof(dst->encoder));
    md400_preset_rotate_range_defaults(dst);
    memcpy(dst->encoder_gesture, src->encoder_gesture, sizeof(src->encoder_gesture));
    md400_preset_encoder_press_gestures_default(dst);
    md400_preset_combo_gestures_off(dst);
    memcpy(dst->combo_gesture, src->combo_gesture, sizeof(src->combo_gesture));
    memcpy(&dst->pedal, &src->pedal, sizeof(dst->pedal));
}

static void md400_preset_copy_v4(md400_preset_t *dst, const md400_preset_v4_t *src)
{
    if (!dst || !src) {
        return;
    }
    memset(dst, 0, sizeof(*dst));
    memcpy(dst->label, src->label, sizeof(dst->label));
    md400_preset_default_led(dst);
    memcpy(dst->encoder, src->encoder, sizeof(dst->encoder));
    md400_preset_rotate_range_defaults(dst);
    memcpy(dst->encoder_gesture, src->encoder_gesture, sizeof(src->encoder_gesture));
    md400_preset_encoder_press_gestures_default(dst);
    md400_preset_combo_gestures_off(dst);
    memcpy(dst->combo_gesture, src->combo_gesture, sizeof(src->combo_gesture));
    memcpy(&dst->pedal, &src->pedal, sizeof(dst->pedal));
}

static void md400_preset_copy_v3(md400_preset_t *dst, const md400_preset_v3_t *src, u8 preset_index)
{
    if (!dst || !src) {
        return;
    }
    memset(dst, 0, sizeof(*dst));
    md400_preset_default_label(dst, preset_index);
    md400_preset_default_led(dst);
    memcpy(dst->encoder, src->encoder, sizeof(dst->encoder));
    md400_preset_rotate_range_defaults(dst);
    memcpy(dst->encoder_gesture, src->encoder_gesture, sizeof(src->encoder_gesture));
    md400_preset_encoder_press_gestures_default(dst);
    md400_preset_combo_gestures_off(dst);
    memcpy(dst->combo_gesture, src->combo_gesture, sizeof(src->combo_gesture));
    memcpy(&dst->pedal, &src->pedal, sizeof(dst->pedal));
}

static void md400_store_set_defaults(void)
{
    u8 preset;
    u8 encoder;
    u8 gesture;
    u8 combo;

    memset(&md400_store, 0, sizeof(md400_store));
    memcpy(md400_store.magic, md400_store_magic, sizeof(md400_store.magic));
    md400_store.version = MD400_PRESET_STORE_VERSION;
    md400_store.active_preset = 0;
    md400_store_active_slot = 0;
    md400_header_seq_value = 0;
    memset(md400_preset_group_active_slot, 0, sizeof(md400_preset_group_active_slot));
    memset(md400_preset_group_active_seq, 0, sizeof(md400_preset_group_active_seq));

    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        md400_preset_default_label(&md400_store.preset[preset], preset);
        md400_preset_default_led(&md400_store.preset[preset]);
        for (encoder = 0; encoder < MD400_ENCODER_COUNT; encoder++) {
            md400_assignment_default(&md400_store.preset[preset].encoder[encoder], encoder + 1);
            md400_assignment_rotate_range_default(&md400_store.preset[preset].encoder[encoder]);
            for (gesture = 0; gesture < MD400_ENCODER_GESTURE_COUNT; gesture++) {
                md400_assignment_off(&md400_store.preset[preset].encoder_gesture[encoder][gesture]);
            }
            md400_assignment_encoder_press_default(
                &md400_store.preset[preset].encoder_gesture[encoder][MD400_ENCODER_GESTURE_COUNT_V7],
                encoder);
        }
        for (combo = 0; combo < MD400_COMBO_COUNT; combo++) {
            for (gesture = 0; gesture < MD400_COMBO_GESTURE_COUNT; gesture++) {
                md400_assignment_off(&md400_store.preset[preset].combo_gesture[combo][gesture]);
            }
        }
        md400_assignment_pedal_default(&md400_store.preset[preset].pedal);
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
}

static void md400_sync_runtime_defaults(void)
{
    u8 i;

    if (md400_store.active_preset >= MD400_PRESET_COUNT) {
        md400_store.active_preset = 0;
    }

    md400_active_preset = md400_store.active_preset;
    md400_selected_preset = md400_active_preset;
    if ((md400_edit_target_encoder < 1) || (md400_edit_target_encoder > MD400_ENCODER_COUNT)) {
        md400_edit_target_encoder = 1;
    }
    if (md400_edit_subject >= MD400_EDIT_SUBJECT_COUNT) {
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    }
    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        md400_encoder_runtime_value[i] =
            md400_clip7(md400_store.preset[md400_active_preset].encoder[i].value_default);
        md400_ms32_set_encoder_value(i + 1, md400_encoder_runtime_value[i]);
    }
    md400_pedal_runtime_value =
        md400_clip7(md400_store.preset[md400_active_preset].pedal.value_default);
    md400_pedal_adc_value = md400_pedal_runtime_value << 1;
    md400_pedal_seen = 0;
    md400_pedal_display_seen = 0;
    md400_pedal_settings_enter_reset();
    md400_pedal_switch_seen = 0;
    md400_pedal_switch_state = 0;
    if (md400_store.preset[md400_active_preset].led_mode > MD400_LED_MODE_MANUAL) {
        md400_store.preset[md400_active_preset].led_mode = MD400_LED_MODE_AUTO;
    }
    md400_store.preset[md400_active_preset].led_mask &= 0x0f;
    md400_play_label_load_active();
    md400_play_show_assignment(&md400_store.preset[md400_active_preset].encoder[0],
                               MD400_PLAY_LABEL_ROTATE_ACTION, 1,
                               md400_encoder_runtime_value[0]);
    md400_runtime_action_state_clear();
}

static void md400_sync_ms32_encoder_runtime_values(void)
{
    u8 i;
    u8 value;

    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
            value = md400_pedal_rotate_value[md400_pedal_active_preset]
                                               [md400_pedal_page][i];
        } else {
            value = md400_encoder_runtime_value[i];
        }
        md400_ms32_set_encoder_value(i + 1, value);
        md400_display_encoder_update(i + 1, value,
                                     md400_ms32_get_encoder_pressed(i + 1));
    }
}

static u8 md400_store_load_legacy_locked(void)
{
    int ret0;
    int ret1;
    u8 valid0;
    u8 valid1;
    u8 candidate_migrated_v10;
    u8 candidate_migrated_v8;
    u8 candidate_migrated_v9;
    u8 migrated_v10 = 0;
    u8 migrated_v8 = 0;
    u8 migrated_v9 = 0;
    u8 use_slot = 0xff;
    u16 best_seq = 0;

    ret0 = syscfg_read(md400_legacy_store_vm_id(0), &md400_store_read_buf, sizeof(md400_store_read_buf));
    valid0 = (ret0 == sizeof(md400_store_read_buf)) && md400_store_valid(&md400_store_read_buf);
    candidate_migrated_v10 = 0;
    candidate_migrated_v8 = 0;
    candidate_migrated_v9 = 0;
    if (!valid0 && (ret0 == sizeof(md400_store_read_buf)) && md400_store_v10_valid(&md400_store_read_buf)) {
        valid0 = 1;
        candidate_migrated_v10 = 1;
    } else if (!valid0 && (ret0 == sizeof(md400_store_read_buf)) && md400_store_v9_valid(&md400_store_read_buf)) {
        valid0 = 1;
        candidate_migrated_v9 = 1;
    } else if (!valid0 && (ret0 == sizeof(md400_store_read_buf)) && md400_store_v8_valid(&md400_store_read_buf)) {
        valid0 = 1;
        candidate_migrated_v8 = 1;
    }
    if (valid0) {
        memcpy(&md400_store, &md400_store_read_buf, sizeof(md400_store));
        best_seq = md400_store_seq(&md400_store);
        use_slot = 0;
        migrated_v10 = candidate_migrated_v10;
        migrated_v8 = candidate_migrated_v8;
        migrated_v9 = candidate_migrated_v9;
    }

    ret1 = syscfg_read(md400_legacy_store_vm_id(1), &md400_store_read_buf, sizeof(md400_store_read_buf));
    valid1 = (ret1 == sizeof(md400_store_read_buf)) && md400_store_valid(&md400_store_read_buf);
    candidate_migrated_v10 = 0;
    candidate_migrated_v8 = 0;
    candidate_migrated_v9 = 0;
    if (!valid1 && (ret1 == sizeof(md400_store_read_buf)) && md400_store_v10_valid(&md400_store_read_buf)) {
        valid1 = 1;
        candidate_migrated_v10 = 1;
    } else if (!valid1 && (ret1 == sizeof(md400_store_read_buf)) && md400_store_v9_valid(&md400_store_read_buf)) {
        valid1 = 1;
        candidate_migrated_v9 = 1;
    } else if (!valid1 && (ret1 == sizeof(md400_store_read_buf)) && md400_store_v8_valid(&md400_store_read_buf)) {
        valid1 = 1;
        candidate_migrated_v8 = 1;
    }
    if (valid1 && ((use_slot == 0xff) || md400_store_seq_newer(md400_store_seq(&md400_store_read_buf), best_seq))) {
        memcpy(&md400_store, &md400_store_read_buf, sizeof(md400_store));
        best_seq = md400_store_seq(&md400_store);
        use_slot = 1;
        migrated_v10 = candidate_migrated_v10;
        migrated_v8 = candidate_migrated_v8;
        migrated_v9 = candidate_migrated_v9;
    }

    if (use_slot != 0xff) {
        if (migrated_v10 || migrated_v9 || migrated_v8) {
            md400_store.version = MD400_PRESET_STORE_VERSION;
            if (migrated_v9 || migrated_v8) {
                md400_store_rotate_range_defaults();
            }
            if (migrated_v8) {
                md400_store_encoder_press_gestures_default_if_off();
            }
        }
        printf("[MD400_CFG] legacy presets active=%u slot=%u seq=%u migrated_v10=%u migrated_v9=%u migrated_v8=%u ret=%d/%d\r\n",
               (unsigned)(md400_store.active_preset + 1), (unsigned)(use_slot + 1),
               (unsigned)best_seq, (unsigned)migrated_v10,
               (unsigned)migrated_v9, (unsigned)migrated_v8, ret0, ret1);
        return 1;
    }

    printf("[MD400_CFG] no legacy preset store ret=%d/%d\r\n", ret0, ret1);
    return 0;
}

static u8 md400_header_load(void)
{
    int ret0;
    int ret1;
    u8 valid0;
    u8 valid1;
    u8 use_slot = 0xff;
    md400_preset_header_t *header;

    ret0 = syscfg_read(md400_header_vm_id(0), &md400_header_read_buf, sizeof(md400_header_read_buf));
    ret1 = syscfg_read(md400_header_vm_id(1), &md400_header_read_buf_b, sizeof(md400_header_read_buf_b));
    valid0 = (ret0 == sizeof(md400_header_read_buf)) && md400_header_valid(&md400_header_read_buf);
    valid1 = (ret1 == sizeof(md400_header_read_buf_b)) && md400_header_valid(&md400_header_read_buf_b);

    if (valid0 && valid1) {
        use_slot = md400_store_seq_newer(md400_seq_get(md400_header_read_buf_b.reserved),
                                         md400_seq_get(md400_header_read_buf.reserved)) ? 1 : 0;
    } else if (valid0) {
        use_slot = 0;
    } else if (valid1) {
        use_slot = 1;
    }

    if (use_slot == 0) {
        header = &md400_header_read_buf;
    } else if (use_slot == 1) {
        header = &md400_header_read_buf_b;
    } else {
        return 0;
    }

    md400_store.active_preset = header->active_preset;
    md400_store_active_slot = use_slot;
    md400_header_seq_value = md400_seq_get(header->reserved);
    printf("[MD400_CFG] header active=%u slot=%u seq=%u ret=%d/%d\r\n",
           (unsigned)(md400_store.active_preset + 1),
           (unsigned)(md400_store_active_slot + 1),
           (unsigned)md400_header_seq_value, ret0, ret1);
    return 1;
}

static u8 md400_preset_groups_load(void)
{
    int ret0;
    int ret1;
    u8 valid0;
    u8 valid1;
    u8 use_slot;
    u8 group_index;
    u8 preset;
    u8 group_offset;
    u8 loaded = 0;
    md400_preset_group_t *group;
    md400_preset_group_v4_t *group_v4;
    md400_preset_group_v3_t *group_v3;

    for (group_index = 0; group_index < MD400_PRESET_GROUP_COUNT; group_index++) {
        use_slot = 0xff;
        ret0 = syscfg_read(md400_group_record_vm_id(group_index, 0), &md400_group_read_buf, sizeof(md400_group_read_buf));
        ret1 = syscfg_read(md400_group_record_vm_id(group_index, 1), &md400_group_read_buf_b, sizeof(md400_group_read_buf_b));
        valid0 = (ret0 == sizeof(md400_group_read_buf)) && md400_group_valid(&md400_group_read_buf, group_index);
        valid1 = (ret1 == sizeof(md400_group_read_buf_b)) && md400_group_valid(&md400_group_read_buf_b, group_index);

        if (valid0 && valid1) {
            use_slot = md400_store_seq_newer(md400_seq_get(md400_group_read_buf_b.reserved),
                                             md400_seq_get(md400_group_read_buf.reserved)) ? 1 : 0;
        } else if (valid0) {
            use_slot = 0;
        } else if (valid1) {
            use_slot = 1;
        }

        if (use_slot == 0) {
            group = &md400_group_read_buf;
        } else if (use_slot == 1) {
            group = &md400_group_read_buf_b;
        } else {
            ret0 = syscfg_read(md400_group_record_vm_id(group_index, 0), &md400_group_v4_read_buf, sizeof(md400_group_v4_read_buf));
            ret1 = syscfg_read(md400_group_record_vm_id(group_index, 1), &md400_group_v4_read_buf_b, sizeof(md400_group_v4_read_buf_b));
            valid0 = (ret0 == sizeof(md400_group_v4_read_buf)) &&
                     md400_group_v4_valid(&md400_group_v4_read_buf, group_index);
            valid1 = (ret1 == sizeof(md400_group_v4_read_buf_b)) &&
                     md400_group_v4_valid(&md400_group_v4_read_buf_b, group_index);

            if (valid0 && valid1) {
                use_slot = md400_store_seq_newer(md400_seq_get(md400_group_v4_read_buf_b.reserved),
                                                 md400_seq_get(md400_group_v4_read_buf.reserved)) ? 1 : 0;
            } else if (valid0) {
                use_slot = 0;
            } else if (valid1) {
                use_slot = 1;
            }

            if (use_slot == 0) {
                group_v4 = &md400_group_v4_read_buf;
            } else if (use_slot == 1) {
                group_v4 = &md400_group_v4_read_buf_b;
            } else {
                group_v4 = NULL;
            }

            if (group_v4) {
                for (group_offset = 0; group_offset < MD400_PRESET_GROUP_SIZE; group_offset++) {
                    preset = (group_index * MD400_PRESET_GROUP_SIZE) + group_offset;
                    if (preset < MD400_PRESET_COUNT) {
                        md400_preset_copy_v4(&md400_store.preset[preset], &group_v4->preset[group_offset]);
                    }
                }
                md400_preset_group_active_slot[group_index] = use_slot;
                md400_preset_group_active_seq[group_index] = md400_seq_get(group_v4->reserved);
                loaded++;
                printf("[MD400_CFG] migrated v4 group=%u slot=%u seq=%u\r\n",
                       (unsigned)(group_index + 1), (unsigned)(use_slot + 1),
                       (unsigned)md400_preset_group_active_seq[group_index]);
                continue;
            }

            ret0 = syscfg_read(md400_group_record_vm_id(group_index, 0), &md400_group_v3_read_buf, sizeof(md400_group_v3_read_buf));
            ret1 = syscfg_read(md400_group_record_vm_id(group_index, 1), &md400_group_v3_read_buf_b, sizeof(md400_group_v3_read_buf_b));
            valid0 = (ret0 == sizeof(md400_group_v3_read_buf)) &&
                     md400_group_v3_valid(&md400_group_v3_read_buf, group_index);
            valid1 = (ret1 == sizeof(md400_group_v3_read_buf_b)) &&
                     md400_group_v3_valid(&md400_group_v3_read_buf_b, group_index);

            if (valid0 && valid1) {
                use_slot = md400_store_seq_newer(md400_seq_get(md400_group_v3_read_buf_b.reserved),
                                                 md400_seq_get(md400_group_v3_read_buf.reserved)) ? 1 : 0;
            } else if (valid0) {
                use_slot = 0;
            } else if (valid1) {
                use_slot = 1;
            }

            if (use_slot == 0) {
                group_v3 = &md400_group_v3_read_buf;
            } else if (use_slot == 1) {
                group_v3 = &md400_group_v3_read_buf_b;
            } else {
                continue;
            }

            for (group_offset = 0; group_offset < MD400_PRESET_GROUP_SIZE; group_offset++) {
                preset = (group_index * MD400_PRESET_GROUP_SIZE) + group_offset;
                if (preset < MD400_PRESET_COUNT) {
                    md400_preset_copy_v3(&md400_store.preset[preset], &group_v3->preset[group_offset], preset);
                }
            }
            md400_preset_group_active_slot[group_index] = use_slot;
            md400_preset_group_active_seq[group_index] = md400_seq_get(group_v3->reserved);
            loaded++;
            printf("[MD400_CFG] migrated v3 group=%u slot=%u seq=%u\r\n",
                   (unsigned)(group_index + 1), (unsigned)(use_slot + 1),
                   (unsigned)md400_preset_group_active_seq[group_index]);
            continue;
        }

        for (group_offset = 0; group_offset < MD400_PRESET_GROUP_SIZE; group_offset++) {
            preset = (group_index * MD400_PRESET_GROUP_SIZE) + group_offset;
            if (preset < MD400_PRESET_COUNT) {
                md400_preset_copy_v6(&md400_store.preset[preset], &group->preset[group_offset]);
            }
        }
        md400_preset_group_active_slot[group_index] = use_slot;
        md400_preset_group_active_seq[group_index] = md400_seq_get(group->reserved);
        loaded++;
    }

    printf("[MD400_CFG] loaded preset groups=%u\r\n", (unsigned)loaded);
    return loaded;
}

/* Transient-scratch wrapper: the legacy/grouped migration buffers live on the
 * heap only while the migration runs. If the heap cannot spare the ~5.6 KB
 * (e.g. late at runtime with btstack up), skip migration — v11 presets have no
 * legacy records and load their individual records regardless. */
static u8 md400_store_load_legacy(void)
{
    extern u32 pmalloc_get_unused_pages(void);
    u8 loaded;

    /* The ROM allocator asserts (rather than returning NULL) when the page
     * pool is exhausted, so check headroom first. 5.6 KB needs ~44 pages. */
    if (pmalloc_get_unused_pages() < 48) {
        printf("[MD400_CFG] heap too low for legacy scratch; migration skipped\r\n");
        return 0;
    }
    md400_load_scratch_p = zalloc(sizeof(*md400_load_scratch_p));
    if (!md400_load_scratch_p) {
        printf("[MD400_CFG] legacy scratch alloc failed; migration skipped\r\n");
        return 0;
    }
    loaded = md400_store_load_legacy_locked();
    free(md400_load_scratch_p);
    md400_load_scratch_p = NULL;
    return loaded;
}

static u8 md400_preset_records_load(void)
{
    int ret;
    u8 preset;
    u8 loaded = 0;
    u8 migrated_v10 = 0;
    u8 migrated_v9 = 0;
    u8 migrated_v8 = 0;
    u8 migrated_v7 = 0;
    u8 migrated_v6 = 0;

    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        ret = syscfg_read(md400_preset_record_vm_id[preset],
                          &md400_preset_record_read_buf,
                          sizeof(md400_preset_record_read_buf));
        if ((ret == sizeof(md400_preset_record_read_buf)) &&
            md400_preset_record_valid(&md400_preset_record_read_buf, preset)) {
            md400_preset_unpack(&md400_store.preset[preset],
                                &md400_preset_record_read_buf.preset);
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_read_buf.reserved);
            loaded++;
            continue;
        }

        ret = syscfg_read(md400_preset_record_vm_id[preset],
                          &md400_preset_record_v10_read_buf,
                          sizeof(md400_preset_record_v10_read_buf));
        if ((ret == sizeof(md400_preset_record_v10_read_buf)) &&
            md400_preset_record_v10_valid(&md400_preset_record_v10_read_buf,
                                          preset)) {
            memcpy(&md400_store.preset[preset],
                   &md400_preset_record_v10_read_buf.preset,
                   sizeof(md400_store.preset[preset]));
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_v10_read_buf.reserved);
            loaded++;
            migrated_v10++;
            continue;
        }

        if ((ret == sizeof(md400_preset_record_v10_read_buf)) &&
            md400_preset_record_v9_valid(&md400_preset_record_v10_read_buf, preset)) {
            md400_preset_copy_v9(&md400_store.preset[preset],
                                 &md400_preset_record_v10_read_buf.preset);
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_v10_read_buf.reserved);
            loaded++;
            migrated_v9++;
            continue;
        }

        if ((ret == sizeof(md400_preset_record_v10_read_buf)) &&
            md400_preset_record_v8_valid(&md400_preset_record_v10_read_buf, preset)) {
            md400_preset_copy_v8(&md400_store.preset[preset],
                                 &md400_preset_record_v10_read_buf.preset);
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_v10_read_buf.reserved);
            loaded++;
            migrated_v8++;
            continue;
        }

        ret = syscfg_read(md400_preset_record_vm_id[preset],
                          &md400_preset_record_v7_read_buf,
                          sizeof(md400_preset_record_v7_read_buf));
        if ((ret == sizeof(md400_preset_record_v7_read_buf)) &&
            md400_preset_record_v7_valid(&md400_preset_record_v7_read_buf, preset)) {
            md400_preset_copy_v7(&md400_store.preset[preset],
                                 &md400_preset_record_v7_read_buf.preset);
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_v7_read_buf.reserved);
            loaded++;
            migrated_v7++;
            continue;
        }

        ret = syscfg_read(md400_preset_record_vm_id[preset],
                          &md400_preset_record_v6_read_buf,
                          sizeof(md400_preset_record_v6_read_buf));
        if ((ret == sizeof(md400_preset_record_v6_read_buf)) &&
            md400_preset_record_v6_valid(&md400_preset_record_v6_read_buf, preset)) {
            md400_preset_copy_v6(&md400_store.preset[preset],
                                 &md400_preset_record_v6_read_buf.preset);
            md400_preset_group_active_seq[preset / MD400_PRESET_GROUP_SIZE] =
                md400_seq_get(md400_preset_record_v6_read_buf.reserved);
            loaded++;
            migrated_v6++;
        }
    }

    printf("[MD400_CFG] loaded preset records=%u migrated_v10=%u migrated_v9=%u migrated_v8=%u migrated_v7=%u migrated_v6=%u\r\n",
           (unsigned)loaded, (unsigned)migrated_v10,
           (unsigned)migrated_v9, (unsigned)migrated_v8,
           (unsigned)migrated_v7, (unsigned)migrated_v6);
    return loaded;
}

static void md400_store_load(void)
{
    md400_store_set_defaults();
    md400_store_load_legacy();
    md400_header_load();
    md400_preset_groups_load();
    md400_preset_records_load();
    md400_store_fix_legacy_rotate_defaults();
    md400_store.checksum = md400_store_checksum(&md400_store);
    md400_sync_runtime_defaults();
}

static int md400_header_save(void)
{
    int ret;
    int verify_ret;
    int active_ret;
    u8 target_slot;
    u8 attempt;
    u16 seq;
    md400_preset_header_t *header = &md400_header_read_buf;

    target_slot = md400_store_active_slot ^ 1;
    seq = md400_header_seq_value + 1;

    memset(header, 0, sizeof(*header));
    memcpy(header->magic, md400_header_magic, sizeof(header->magic));
    header->version = MD400_PRESET_STORE_VERSION;
    header->active_preset = md400_active_preset;
    md400_seq_set(header->reserved, seq);
    header->checksum = md400_header_checksum(header);

    active_ret = syscfg_read(md400_header_vm_id(md400_store_active_slot),
                             &md400_header_read_buf_b,
                             sizeof(md400_header_read_buf_b));
    if ((active_ret == sizeof(md400_header_read_buf_b)) &&
        md400_header_valid(&md400_header_read_buf_b) &&
        (md400_header_read_buf_b.version == header->version) &&
        (md400_header_read_buf_b.active_preset == header->active_preset)) {
        return sizeof(*header);
    }

    for (attempt = 1; attempt <= MD400_VM_WRITE_TRIES; attempt++) {
        ret = syscfg_write(md400_header_vm_id(target_slot), header,
                           sizeof(*header));
        verify_ret = syscfg_read(md400_header_vm_id(target_slot),
                                 &md400_header_read_buf_b,
                                 sizeof(md400_header_read_buf_b));
        if ((ret == sizeof(*header)) &&
            (verify_ret == sizeof(md400_header_read_buf_b)) &&
            md400_header_valid(&md400_header_read_buf_b) &&
            !memcmp(&md400_header_read_buf_b, header, sizeof(*header))) {
            md400_store_active_slot = target_slot;
            md400_header_seq_value = seq;
            return ret;
        }
        md400_dbg_vm_fail(MD400_DBG_VM_HEADER, md400_header_vm_id(target_slot),
                          ret, verify_ret,
                          md400_header_valid(&md400_header_read_buf_b),
                          !memcmp(&md400_header_read_buf_b, header,
                                  sizeof(*header)));
        printf("[MD400_CFG] header save try=%u slot=%u ret=%d verify=%d\r\n",
               (unsigned)attempt, (unsigned)(target_slot + 1),
               ret, verify_ret);
    }
    return -1;
}

static int md400_preset_group_save(u8 preset_index)
{
    int ret;
    int verify_ret;
    int active_ret;
    u16 seq;
    u8 group_index;
    u8 attempt;
    md400_preset_record_t *record = &md400_preset_record_buf;

    if (preset_index >= MD400_PRESET_COUNT) {
        return -1;
    }

    group_index = preset_index / MD400_PRESET_GROUP_SIZE;
    seq = md400_preset_group_active_seq[group_index] + 1;

    memset(record, 0, sizeof(*record));
    memcpy(record->magic, md400_record_magic, sizeof(record->magic));
    record->version = MD400_PRESET_STORE_VERSION;
    record->preset_index = preset_index;
    md400_seq_set(record->reserved, seq);
    md400_preset_pack(&record->preset, &md400_store.preset[preset_index]);
    record->checksum = md400_preset_record_checksum(record);

    active_ret = syscfg_read(md400_preset_record_vm_id[preset_index],
                             &md400_preset_record_read_buf,
                             sizeof(md400_preset_record_read_buf));
    if ((active_ret == sizeof(md400_preset_record_read_buf)) &&
        md400_preset_record_valid(&md400_preset_record_read_buf, preset_index) &&
        !memcmp(&md400_preset_record_read_buf.preset, &record->preset, sizeof(record->preset))) {
        return sizeof(*record);
    }

    for (attempt = 1; attempt <= MD400_VM_WRITE_TRIES; attempt++) {
        ret = syscfg_write(md400_preset_record_vm_id[preset_index], record,
                           sizeof(*record));
        verify_ret = syscfg_read(md400_preset_record_vm_id[preset_index],
                                 &md400_preset_record_read_buf,
                                 sizeof(md400_preset_record_read_buf));
        if ((ret == sizeof(*record)) &&
            (verify_ret == sizeof(md400_preset_record_read_buf)) &&
            md400_preset_record_valid(&md400_preset_record_read_buf,
                                      preset_index) &&
            !memcmp(&md400_preset_record_read_buf, record, sizeof(*record))) {
            md400_preset_group_active_seq[group_index] = seq;
            return ret;
        }
        md400_dbg_vm_fail(MD400_DBG_VM_PRESET,
                          md400_preset_record_vm_id[preset_index],
                          ret, verify_ret,
                          md400_preset_record_valid(&md400_preset_record_read_buf,
                                                    preset_index),
                          !memcmp(&md400_preset_record_read_buf, record,
                                  sizeof(*record)));
        printf("[MD400_CFG] preset save try=%u preset=%u id=%u ret=%d verify=%d\r\n",
               (unsigned)attempt, (unsigned)(preset_index + 1),
               (unsigned)md400_preset_record_vm_id[preset_index],
               ret, verify_ret);
    }
    return -1;
}

static int md400_store_save(void)
{
    int preset_ret;
    int header_ret;

    if (md400_active_preset >= MD400_PRESET_COUNT) {
        return -1;
    }

    md400_store.active_preset = md400_active_preset;
    md400_store.checksum = md400_store_checksum(&md400_store);
    preset_ret = md400_preset_group_save(md400_active_preset);
    header_ret = md400_header_save();

    if ((preset_ret > 0) && (header_ret > 0)) {
        printf("[MD400_CFG] save ok preset=%u group=%u gslot=%u gseq=%u hslot=%u hseq=%u\r\n",
               (unsigned)(md400_active_preset + 1),
               (unsigned)((md400_active_preset / MD400_PRESET_GROUP_SIZE) + 1),
               (unsigned)(md400_preset_group_active_slot[md400_active_preset / MD400_PRESET_GROUP_SIZE] + 1),
               (unsigned)md400_preset_group_active_seq[md400_active_preset / MD400_PRESET_GROUP_SIZE],
               (unsigned)(md400_store_active_slot + 1),
               (unsigned)md400_header_seq_value);
        return sizeof(md400_store);
    }

    return -1;
}

static int md400_factory_reset_save(void)
{
    u8 saved_store_slot;
    u16 saved_header_seq;
    u8 saved_group_slot[MD400_PRESET_GROUP_COUNT];
    u16 saved_group_seq[MD400_PRESET_GROUP_COUNT];
    u8 preset;
    int ret;

    saved_store_slot = md400_store_active_slot;
    saved_header_seq = md400_header_seq_value;
    memcpy(saved_group_slot, md400_preset_group_active_slot, sizeof(saved_group_slot));
    memcpy(saved_group_seq, md400_preset_group_active_seq, sizeof(saved_group_seq));

    md400_store_set_defaults();
    md400_store_active_slot = saved_store_slot;
    md400_header_seq_value = saved_header_seq;
    memcpy(md400_preset_group_active_slot, saved_group_slot, sizeof(md400_preset_group_active_slot));
    memcpy(md400_preset_group_active_seq, saved_group_seq, sizeof(md400_preset_group_active_seq));

    md400_active_preset = 0;
    md400_selected_preset = 0;
    md400_store.active_preset = 0;
    md400_ui_mode = MD400_UI_MODE_PERFORM;
    md400_edit_page = 0;
    md400_edit_target_encoder = 1;
    md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    md400_edit_session_active = 0;
    md400_edit_session_dirty = 0;
    md400_sync_runtime_defaults();

    printf("[MD400_CFG] factory reset begin\r\n");
    for (preset = 0; preset < MD400_PRESET_COUNT; preset++) {
        printf("[MD400_CFG] factory reset preset %u\r\n",
               (unsigned)(preset + 1));
        ret = md400_preset_group_save(preset);
        if (ret <= 0) {
            printf("[MD400_CFG] factory reset preset save fail preset=%u ret=%d\r\n",
                   (unsigned)(preset + 1), ret);
            md400_store_load();
            md400_ui_mode = MD400_UI_MODE_PERFORM;
            md400_edit_page = 0;
            md400_edit_target_encoder = 1;
            md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
            md400_edit_session_active = 0;
            md400_edit_session_dirty = 0;
            md400_ui_display_sync();
            return -1;
        }
        os_time_dly(1);
    }

    printf("[MD400_CFG] factory reset play labels\r\n");
    ret = md400_play_label_factory_reset();
    if (ret != MD400_NB_STATUS_OK) {
        printf("[MD400_CFG] factory reset play label save fail status=%d\r\n", ret);
        md400_store_load();
        md400_ui_mode = MD400_UI_MODE_PERFORM;
        md400_edit_page = 0;
        md400_edit_target_encoder = 1;
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_session_active = 0;
        md400_edit_session_dirty = 0;
        md400_ui_display_sync();
        return -1;
    }

    printf("[MD400_CFG] factory reset sysex templates\r\n");
    ret = md400_sysex_template_factory_reset();
    if (ret != MD400_NB_STATUS_OK) {
        printf("[MD400_CFG] factory reset sysex template save fail status=%d\r\n", ret);
        md400_store_load();
        md400_ui_mode = MD400_UI_MODE_PERFORM;
        md400_edit_page = 0;
        md400_edit_target_encoder = 1;
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_session_active = 0;
        md400_edit_session_dirty = 0;
        md400_ui_display_sync();
        return -1;
    }

    printf("[MD400_CFG] factory reset custom templates\r\n");
    ret = md400_custom_template_factory_reset();
    if (ret != MD400_NB_STATUS_OK) {
        printf("[MD400_CFG] factory reset custom template save fail status=%d\r\n", ret);
        md400_store_load();
        md400_ui_mode = MD400_UI_MODE_PERFORM;
        md400_edit_page = 0;
        md400_edit_target_encoder = 1;
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_session_active = 0;
        md400_edit_session_dirty = 0;
        md400_ui_display_sync();
        return -1;
    }

    printf("[MD400_CFG] factory reset header\r\n");
    ret = md400_header_save();
    if (ret <= 0) {
        printf("[MD400_CFG] factory reset header save fail ret=%d\r\n", ret);
        md400_store_load();
        md400_ui_mode = MD400_UI_MODE_PERFORM;
        md400_edit_page = 0;
        md400_edit_target_encoder = 1;
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_session_active = 0;
        md400_edit_session_dirty = 0;
        md400_ui_display_sync();
        return -1;
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
    md400_sync_runtime_defaults();
    printf("[MD400_CFG] factory reset pedal mode/cal/bank\r\n");
    md400_pedal_mode_set(MD400_PEDAL_MODE_EXP);
    md400_pedal_cal_reset();
    if (!md400_pedal_bank_factory_reset()) return -1;
    md400_ui_display_sync();
    printf("[MD400_CFG] factory reset ok presets=%u active=1 hslot=%u hseq=%u\r\n",
           (unsigned)MD400_PRESET_COUNT,
           (unsigned)(md400_store_active_slot + 1),
           (unsigned)md400_header_seq_value);
    return sizeof(md400_store);
}

static int md400_preset_save_selected_from_current(void)
{
    md400_preset_t saved_target;
    u8 saved_label[MD400_PRESET_LABEL_LEN];
    u8 previous_active;
    int ret;

    if ((md400_active_preset >= MD400_PRESET_COUNT) ||
        (md400_selected_preset >= MD400_PRESET_COUNT)) {
        return -1;
    }

    previous_active = md400_active_preset;
    memcpy(&saved_target, &md400_store.preset[md400_selected_preset], sizeof(saved_target));
    memcpy(saved_label, md400_store.preset[md400_selected_preset].label, sizeof(saved_label));

    if (md400_selected_preset != md400_active_preset) {
        memcpy(&md400_store.preset[md400_selected_preset],
               &md400_store.preset[md400_active_preset],
               sizeof(md400_preset_t));
        memcpy(md400_store.preset[md400_selected_preset].label, saved_label, sizeof(saved_label));
    }

    md400_active_preset = md400_selected_preset;
    md400_store.active_preset = md400_active_preset;
    md400_sync_runtime_defaults();
    ret = md400_store_save();
    if (ret == sizeof(md400_store)) {
        return ret;
    }

    memcpy(&md400_store.preset[md400_selected_preset], &saved_target, sizeof(saved_target));
    md400_active_preset = previous_active;
    md400_store.active_preset = md400_active_preset;
    md400_sync_runtime_defaults();
    md400_store.checksum = md400_store_checksum(&md400_store);
    return ret;
}

static void md400_preset_save_selected_schedule(void)
{
    u8 saved_label[MD400_PRESET_LABEL_LEN];

    if ((md400_active_preset >= MD400_PRESET_COUNT) ||
        (md400_selected_preset >= MD400_PRESET_COUNT)) {
        return;
    }

    memcpy(saved_label, md400_store.preset[md400_selected_preset].label, sizeof(saved_label));
    if (md400_selected_preset != md400_active_preset) {
        memcpy(&md400_store.preset[md400_selected_preset],
               &md400_store.preset[md400_active_preset],
               sizeof(md400_preset_t));
        memcpy(md400_store.preset[md400_selected_preset].label, saved_label, sizeof(saved_label));
    }

    md400_active_preset = md400_selected_preset;
    md400_store.active_preset = md400_active_preset;
    md400_store.checksum = md400_store_checksum(&md400_store);
    md400_sync_runtime_defaults();
    md400_store_save_schedule(MD400_STORE_SAVE_REASON_PRESET_SELECT);
}

static void md400_edit_session_begin(void)
{
    if (md400_active_preset >= MD400_PRESET_COUNT) {
        md400_active_preset = 0;
        md400_store.active_preset = 0;
    }
    memcpy(&md400_edit_snapshot,
           &md400_store.preset[md400_active_preset],
           sizeof(md400_edit_snapshot));
    md400_edit_session_active = 1;
    md400_edit_session_dirty = 0;
    printf("[MD400_UI] edit session begin preset=%u\r\n", (unsigned)(md400_active_preset + 1));
}

static void md400_edit_session_mark_dirty(void)
{
    if (md400_edit_session_active) {
        md400_edit_session_dirty = 1;
        if (!md400_nb_dispatch_active) md400_nb_note_change(2);
    }
}

static void md400_edit_session_save(void)
{
    if (md400_active_preset < MD400_PRESET_COUNT) {
        memcpy(&md400_edit_snapshot,
               &md400_store.preset[md400_active_preset],
               sizeof(md400_edit_snapshot));
    }
    md400_edit_session_dirty = 0;
    md400_store_save_schedule(MD400_STORE_SAVE_REASON_EDIT);
    md400_display_show_save_check();
    printf("[MD400_UI] edit save scheduled dirty=%u\r\n", (unsigned)md400_edit_session_dirty);
}

static void md400_edit_session_cancel(void)
{
    if (!md400_edit_session_active) {
        return;
    }
    if (md400_edit_session_dirty && (md400_active_preset < MD400_PRESET_COUNT)) {
        memcpy(&md400_store.preset[md400_active_preset],
               &md400_edit_snapshot,
               sizeof(md400_edit_snapshot));
        md400_store.checksum = md400_store_checksum(&md400_store);
        md400_sync_runtime_defaults();
        printf("[MD400_UI] edit cancel preset=%u\r\n", (unsigned)(md400_active_preset + 1));
    }
    md400_edit_session_dirty = 0;
}

static void md400_edit_session_end(u8 save)
{
    if (!md400_edit_session_active) {
        return;
    }
    md400_preset_load_timer_cancel();
    if (save) {
        md400_edit_session_save();
    } else {
        md400_edit_session_cancel();
    }
    md400_edit_session_active = 0;
}

static void md400_preset_load_selected_from_edit(void)
{
    u8 previous_active;
    u8 target_preset;

    md400_preset_load_timer_cancel();
    if ((md400_selected_preset >= MD400_PRESET_COUNT) ||
        (md400_active_preset >= MD400_PRESET_COUNT)) {
        return;
    }
    target_preset = md400_selected_preset;
    if (target_preset == md400_active_preset) {
        printf("[MD400_UI] preset select already active=%u\r\n",
               (unsigned)(md400_active_preset + 1));
        return;
    }

    previous_active = md400_active_preset;
    md400_edit_session_end(0);
    md400_active_preset = target_preset;
    md400_store.active_preset = md400_active_preset;
    md400_sync_runtime_defaults();
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_edit_session_begin();
        md400_edit_page = MD400_EDIT_PAGE_PRESET;
    }
    md400_header_save_schedule();
    md400_ui_display_sync();
    printf("[MD400_UI] preset load selected=%u previous=%u\r\n",
           (unsigned)(md400_active_preset + 1),
           (unsigned)(previous_active + 1));
}

static void md400_preset_load_deferred(void *priv)
{
    (void)priv;
    md400_preset_load_timer = 0;
    if ((md400_ui_mode == MD400_UI_MODE_EDIT) &&
        (md400_edit_page == MD400_EDIT_PAGE_PRESET)) {
        md400_preset_load_selected_from_edit();
    }
}

static void md400_preset_load_timer_cancel(void)
{
    if (md400_preset_load_timer) {
        sys_timeout_del(md400_preset_load_timer);
        md400_preset_load_timer = 0;
    }
}

static void md400_preset_load_schedule(void)
{
    md400_preset_load_timer_cancel();
    md400_preset_load_timer = sys_timeout_add(NULL,
                                              md400_preset_load_deferred,
                                              MD400_PRESET_LOAD_DEBOUNCE_MS);
}

static void md400_edit_return_to_preset_page(u8 event, u8 control)
{
    if (md400_ui_mode != MD400_UI_MODE_EDIT) {
        return;
    }
    md400_preset_load_timer_cancel();
    md400_edit_page = MD400_EDIT_PAGE_PRESET;
    md400_pedal_settings_enter_reset();
    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        md400_pedal_selected_preset = md400_pedal_active_preset;
        md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_subject_before_pedal = MD400_EDIT_DEFAULT_SUBJECT;
        md400_ui_display_sync();
        md400_log_event(0x03, event, control,
                        md400_pedal_selected_preset + 1);
        printf("[NEXFOOT_PEDAL] preset page combo=%02x active=%u page=%u\r\n",
               (unsigned)control,
               (unsigned)(md400_pedal_active_preset + 1),
               (unsigned)(md400_pedal_page + 1));
        return;
    }
    md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    md400_selected_preset = md400_active_preset;
    md400_ui_display_sync();
    md400_log_event(0x03, event, control, md400_selected_preset + 1);
    printf("[MD400_UI] preset page nav combo=%02x active=%u\r\n",
           (unsigned)control, (unsigned)(md400_active_preset + 1));
}

static md400_assignment_t *md400_get_assignment(u8 encoder)
{
    md400_control_init();

    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return NULL;
    }
    if (md400_active_preset >= MD400_PRESET_COUNT) {
        md400_active_preset = 0;
    }

    return &md400_store.preset[md400_active_preset].encoder[encoder - 1];
}

static md400_assignment_t *md400_get_pedal_assignment(void)
{
    md400_control_init();

    if (md400_active_preset >= MD400_PRESET_COUNT) {
        md400_active_preset = 0;
    }

    return &md400_store.preset[md400_active_preset].pedal;
}

static md400_assignment_t *md400_get_pedal_edit_assignment(void)
{
    md400_control_init();

    if (md400_foot_mode != MD400_FOOT_MODE_PEDAL) {
        return md400_get_pedal_assignment();
    }
    if (md400_pedal_active_preset >= MD400_PRESET_COUNT) {
        md400_pedal_active_preset = 0;
    }
    return &md400_store.preset[md400_pedal_active_preset].pedal;
}

static void md400_pedal_exp_cancel(void)
{
    if (md400_pedal_exp_dirty) {
        md400_store.preset[md400_pedal_exp_snapshot_preset].pedal =
            md400_pedal_exp_snapshot;
        md400_store.checksum = md400_store_checksum(&md400_store);
        md400_pedal_exp_dirty = 0;
    }
}

static u8 md400_combo_index_from_mask(u8 mask)
{
    switch (mask & 0x0f) {
    case 0x03:
        return 0;
    case 0x06:
        return 1;
    case 0x0c:
        return 2;
    default:
        return 0xff;
    }
}

static u8 md400_encoder_gesture_index(u8 event)
{
    if (event == MD400_EVENT_ENCODER_SHORT) {
        return 0;
    }
    if (event == MD400_EVENT_ENCODER_LONG) {
        return 1;
    }
    if (event == MD400_EVENT_ENCODER_DOUBLE) {
        return 2;
    }
    if (event == MD400_EVENT_ENCODER_PRESS) {
        return 3;
    }
    return 0xff;
}

static u8 md400_combo_gesture_index(u8 event)
{
    if (event == MD400_EVENT_COMBO_SHORT) {
        return 0;
    }
    if (event == MD400_EVENT_COMBO_LONG) {
        return 1;
    }
    if (event == MD400_EVENT_COMBO_PRESS) {
        return 2;
    }
    return 0xff;
}

static md400_assignment_t *md400_get_gesture_assignment(u8 event, u8 control)
{
    u8 index;
    u8 combo;

    md400_control_init();
    if (md400_active_preset >= MD400_PRESET_COUNT) {
        md400_active_preset = 0;
    }

    index = md400_encoder_gesture_index(event);
    if (index != 0xff) {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_store.preset[md400_active_preset].encoder_gesture[control - 1][index];
    }

    index = md400_combo_gesture_index(event);
    if (index != 0xff) {
        combo = md400_combo_index_from_mask(control);
        if (combo >= MD400_COMBO_COUNT) {
            return NULL;
        }
        return &md400_store.preset[md400_active_preset].combo_gesture[combo][index];
    }

    return NULL;
}

static u8 *md400_gesture_toggle_state_ptr(u8 event, u8 control)
{
    u8 index;
    u8 combo;

    index = md400_encoder_gesture_index(event);
    if (index != 0xff) {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_encoder_gesture_toggle_state[control - 1][index];
    }

    index = md400_combo_gesture_index(event);
    if (index != 0xff) {
        combo = md400_combo_index_from_mask(control);
        if (combo >= MD400_COMBO_COUNT) {
            return NULL;
        }
        return &md400_combo_gesture_toggle_state[combo][index];
    }

    return NULL;
}

static u8 *md400_gesture_momentary_active_ptr(u8 event, u8 control)
{
    u8 combo;

    if (event == MD400_EVENT_ENCODER_PRESS) {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_encoder_press_momentary_active[control - 1];
    }

    if (event == MD400_EVENT_ENCODER_LONG) {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_encoder_long_momentary_active[control - 1];
    }

    if (event == MD400_EVENT_COMBO_LONG) {
        combo = md400_combo_index_from_mask(control);
        if (combo >= MD400_COMBO_COUNT) {
            return NULL;
        }
        return &md400_combo_long_momentary_active[combo];
    }

    return NULL;
}

static void md400_release_encoder_momentary(u8 event, u8 encoder)
{
    md400_assignment_t *assignment;
    u8 *active;

    active = md400_gesture_momentary_active_ptr(event, encoder);
    if (!active || !*active) {
        return;
    }

    assignment = md400_get_gesture_assignment(event, encoder);
    if (assignment && (assignment->type != MD400_MIDI_TYPE_OFF) &&
        (assignment->behavior == MD400_BEHAVIOR_MOMENTARY)) {
        int display_msg = md400_assignment_send_gate(assignment, encoder, 0);
        md400_play_show_assignment_sent(assignment, event, encoder,
                                        display_msg, assignment->value_min);
    }
    *active = 0;
}

static md400_assignment_t *md400_short_momentary_assignment(u8 encoder)
{
    md400_assignment_t *assignment;

    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return NULL;
    }
    assignment = md400_get_gesture_assignment(MD400_EVENT_ENCODER_SHORT, encoder);
    if (!assignment ||
        (assignment->type == MD400_MIDI_TYPE_OFF) ||
        (assignment->behavior != MD400_BEHAVIOR_MOMENTARY)) {
        return NULL;
    }
    return assignment;
}

/* Product Press is SHORT. SHORT is generated after release, so hold Momentary
 * has to borrow the physical down/up pair: Max on press, Min on release. */
static u8 md400_begin_short_momentary(u8 encoder)
{
    md400_assignment_t *assignment = md400_short_momentary_assignment(encoder);
    int display_msg;

    if (!assignment) {
        return 0;
    }
    md400_encoder_short_momentary_active[encoder - 1] = 1;
    display_msg = md400_assignment_send_gate(assignment, encoder, 1);
    md400_play_show_assignment_sent(assignment, MD400_EVENT_ENCODER_SHORT,
                                    encoder, display_msg,
                                    assignment->value_max);
    return 1;
}

static void md400_release_short_momentary(u8 encoder)
{
    md400_assignment_t *assignment;
    int display_msg;

    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return;
    }
    if (!md400_encoder_short_momentary_active[encoder - 1]) {
        return;
    }
    md400_encoder_short_momentary_active[encoder - 1] = 0;
    assignment = md400_short_momentary_assignment(encoder);
    if (!assignment) {
        return;
    }
    display_msg = md400_assignment_send_gate(assignment, encoder, 0);
    md400_play_show_assignment_sent(assignment, MD400_EVENT_ENCODER_SHORT,
                                    encoder, display_msg,
                                    assignment->value_min);
}

static u8 md400_encoder_short_blocked_by_press(u8 event, u8 control)
{
    md400_assignment_t *assignment;

    if ((event != MD400_EVENT_ENCODER_SHORT) ||
        (control < 1) || (control > MD400_ENCODER_COUNT)) {
        return 0;
    }

    assignment = md400_get_gesture_assignment(MD400_EVENT_ENCODER_PRESS, control);
    return assignment && (assignment->type != MD400_MIDI_TYPE_OFF);
}

static void md400_log_event(u8 source, u8 event, u8 control, u8 value)
{
    md400_event_log[md400_event_log_wp].source = source;
    md400_event_log[md400_event_log_wp].event = event;
    md400_event_log[md400_event_log_wp].control = control;
    md400_event_log[md400_event_log_wp].value = value;
    md400_event_log_wp = (md400_event_log_wp + 1) & 0x0f;
    md400_event_log_total++;
}

static u8 md400_combo_adjacent_mask(u8 mask)
{
    if ((mask & 0x03) == 0x03) {
        return 0x03;
    }
    if ((mask & 0x06) == 0x06) {
        return 0x06;
    }
    if ((mask & 0x0c) == 0x0c) {
        return 0x0c;
    }
    return 0;
}

static u8 md400_combo_within_window(u8 mask)
{
    u8 first = 0xff;
    u8 second = 0xff;
    u8 i;

    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        if (mask & BIT(i)) {
            if (first == 0xff) {
                first = i;
            } else {
                second = i;
                break;
            }
        }
    }

    if (second == 0xff) {
        return 0;
    }

    if (md400_button_gesture[first].down_ms >= md400_button_gesture[second].down_ms) {
        return (md400_button_gesture[first].down_ms - md400_button_gesture[second].down_ms) <= MD400_ENCODER_COMBO_MS;
    }
    return (md400_button_gesture[second].down_ms - md400_button_gesture[first].down_ms) <= MD400_ENCODER_COMBO_MS;
}

/* Edit navigation is based on overlapping holds, not on performance-gesture
 * timing. A 120 ms chord window is too short for the two encoder switches to
 * be pressed reliably by one hand, and a missed chord later becomes an E1 or
 * E2 short-page selection. Keep the tight window for assignable performance
 * combos, but accept these two reserved edit-navigation chords whenever both
 * buttons are down together. */
static u8 md400_edit_combo_nav_overlap_allowed(u8 mask)
{
    if (md400_ui_mode != MD400_UI_MODE_EDIT) {
        return 0;
    }
    if (mask == MD400_EDIT_PRESET_NAV_COMBO_MASK) {
        return 1;
    }
    return (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) &&
           (mask == MD400_EDIT_PEDAL_EXIT_COMBO_MASK);
}

static u8 md400_edit_subject_from_event(u8 event)
{
    if (!md400_gesture_feature_enabled(event)) {
        return 0xff;
    }

    switch (event) {
    case MD400_EVENT_ENCODER_PRESS:
#if !CONFIG_MD400_ENCODER_PRESS_EDIT_ENABLE
        return 0xff;
#endif
        return MD400_EDIT_SUBJECT_PRESS;
    case MD400_EVENT_ENCODER_SHORT:
        return MD400_EDIT_SUBJECT_SHORT;
    case MD400_EVENT_ENCODER_LONG:
        return MD400_EDIT_SUBJECT_LONG;
    case MD400_EVENT_ENCODER_DOUBLE:
        return MD400_EDIT_SUBJECT_DOUBLE;
    case MD400_EVENT_COMBO_PRESS:
        return MD400_EDIT_SUBJECT_COMBO_PRESS;
    case MD400_EVENT_COMBO_SHORT:
        return MD400_EDIT_SUBJECT_COMBO_SHORT;
    case MD400_EVENT_COMBO_LONG:
        return MD400_EDIT_SUBJECT_COMBO_LONG;
    default:
        return 0xff;
    }
}

static u8 md400_edit_select_gesture(u8 event, u8 control)
{
    u8 subject;
    u8 combo;

    if (md400_ui_mode != MD400_UI_MODE_EDIT) {
        return 0;
    }

    if (event == MD400_EVENT_ENCODER_SHORT) {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return 0;
        }
        if (md400_edit_page_is_preset_select()) {
            /* In edit mode a single encoder key selects only on release. The
             * down edge is reserved for held-state/combo recognition. */
            md400_preset_load_timer_cancel();
            md400_pedal_cal_hide();
            md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
            md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
        } else if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
            /* Leave the pedal-settings page: return to this encoder's edit,
             * restoring whatever subject the pedal-hold interrupted. Combo
             * subjects target combos, not encoders, so fall back to the
             * default subject for those. */
            subject = md400_edit_subject_before_pedal;
            if (subject >= MD400_EDIT_SUBJECT_COMBO_PRESS) {
                /* Combo subjects target combos, not encoders (and pedal /
                 * stale values make no sense here) — use the default. */
                subject = MD400_EDIT_DEFAULT_SUBJECT;
            }
            md400_edit_subject = subject;
            md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
        }
        md400_edit_target_encoder = control;
        md400_ui_display_sync();
        md400_log_event(0x03, event, control, md400_edit_subject);
        printf("[MD400_UI] edit short event target=%u subject=%u page=%u\r\n",
               (unsigned)md400_edit_target_encoder,
               (unsigned)md400_edit_subject,
               (unsigned)md400_edit_page);
        return 1;
    }

    subject = md400_edit_subject_from_event(event);
    if (subject == 0xff) {
        return 0;
    }

    if ((subject == MD400_EDIT_SUBJECT_COMBO_PRESS) ||
        (subject == MD400_EDIT_SUBJECT_COMBO_SHORT) ||
        (subject == MD400_EDIT_SUBJECT_COMBO_LONG)) {
        combo = md400_combo_index_from_mask(control);
        if (combo >= MD400_COMBO_COUNT) {
            return 0;
        }
        md400_edit_target_encoder = combo + 1;
    } else {
        if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
            return 0;
        }
        md400_edit_target_encoder = control;
    }

    md400_edit_subject = subject;
    md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
    md400_edit_normalize_subject_for_page();
    md400_ui_display_sync();
    md400_log_event(0x03, event, control, md400_edit_subject);
    printf("[MD400_UI] edit gesture event=%u control=%u target=%u subject=%u page=%u\r\n",
           (unsigned)event, (unsigned)control,
           (unsigned)md400_edit_target_encoder,
           (unsigned)md400_edit_subject,
           (unsigned)md400_edit_page);
    return 1;
}

static void md400_release_encoder_long_momentary(u8 encoder)
{
    md400_release_encoder_momentary(MD400_EVENT_ENCODER_LONG, encoder);
}

static void md400_release_combo_long_momentary(u8 mask)
{
    md400_assignment_t *assignment;
    u8 *active;

    active = md400_gesture_momentary_active_ptr(MD400_EVENT_COMBO_LONG, mask);
    if (!active || !*active) {
        return;
    }

    assignment = md400_get_gesture_assignment(MD400_EVENT_COMBO_LONG, mask);
    if (assignment && (assignment->type != MD400_MIDI_TYPE_OFF) &&
        (assignment->behavior == MD400_BEHAVIOR_MOMENTARY)) {
        int display_msg = md400_assignment_send_gate(assignment, mask, 0);
        md400_play_show_assignment_sent(assignment, MD400_EVENT_COMBO_LONG,
                                        mask, display_msg,
                                        assignment->value_min);
    }
    *active = 0;
}

static void md400_emit_gesture(u8 event, u8 control, u8 value)
{
    md400_assignment_t *assignment;
    u8 *toggle_state;
    u8 *momentary_active;
    int display_msg;
    u8 out_value;

    if (md400_daw_active) {
        md400_daw_gesture(event, control);
        return;
    }

    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        if ((event == MD400_EVENT_COMBO_PRESS) ||
            (event == MD400_EVENT_COMBO_LONG)) {
            return;
        }
        if (event == MD400_EVENT_COMBO_SHORT) {
            if (control == 0x03) {
                md400_pedal_page_step(-1);
            } else if (control == 0x0c) {
                md400_pedal_page_step(1);
            } else if ((control == 0x06) &&
                       (md400_ui_mode == MD400_UI_MODE_EDIT)) {
                md400_edit_return_to_preset_page(event, control);
            }
            return;
        }
        if ((event == MD400_EVENT_ENCODER_RELEASE) ||
            ((event == MD400_EVENT_ENCODER_SHORT) &&
             (md400_ui_mode == MD400_UI_MODE_EDIT))) {
            md400_pedal_release_action(control);
        }
        return;
    }

    if ((md400_ui_mode == MD400_UI_MODE_EDIT) &&
        ((event == MD400_EVENT_COMBO_PRESS) ||
         (event == MD400_EVENT_COMBO_SHORT)) &&
        ((control == MD400_EDIT_PRESET_NAV_COMBO_MASK) ||
         ((md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) &&
          (control == MD400_EDIT_PEDAL_EXIT_COMBO_MASK)))) {
        md400_edit_return_to_preset_page(event, control);
        return;
    }

    if (!md400_gesture_feature_enabled(event)) {
        md400_log_event(0x04, event, control, 0);
        printf("[MD400_GEST] disabled event=%u control=%u\r\n",
               (unsigned)event, (unsigned)control);
        return;
    }

    md400_log_event(0x04, event, control, value);
    printf("[MD400_GEST] event=%u control=%u value=%u\r\n",
           (unsigned)event, (unsigned)control, (unsigned)value);

    if (event == MD400_EVENT_ENCODER_RELEASE) {
        md400_release_short_momentary(control);
        md400_release_encoder_momentary(MD400_EVENT_ENCODER_PRESS, control);
        md400_release_encoder_long_momentary(control);
        return;
    }

    if (md400_edit_select_gesture(event, control)) {
        return;
    }

    if (!md400_ui_performance_active()) {
        return;
    }

    if (md400_encoder_short_blocked_by_press(event, control)) {
        printf("[MD400_GEST] short blocked by press control=%u\r\n",
               (unsigned)control);
        return;
    }

    if ((event == MD400_EVENT_ENCODER_PRESS) &&
        md400_begin_short_momentary(control)) {
        return;
    }
    if ((event == MD400_EVENT_ENCODER_SHORT) &&
        md400_short_momentary_assignment(control)) {
        return;
    }

    assignment = md400_get_gesture_assignment(event, control);
    if (assignment && (assignment->type != MD400_MIDI_TYPE_OFF)) {
        if (assignment->behavior == MD400_BEHAVIOR_TOGGLE) {
            toggle_state = md400_gesture_toggle_state_ptr(event, control);
            if (toggle_state) {
                *toggle_state ^= 1;
                out_value = *toggle_state ? assignment->value_max :
                            assignment->value_min;
                display_msg = md400_assignment_send_gate(assignment, control,
                                                         *toggle_state);
                md400_led_sync();
                md400_play_show_assignment_sent(assignment, event, control,
                                                display_msg, out_value);
            }
        } else if (assignment->behavior == MD400_BEHAVIOR_PULSE) {
            md400_assignment_send_gate(assignment, control, 1);
            display_msg = md400_assignment_send_gate(assignment, control, 0);
            md400_play_show_assignment_sent(assignment, event, control,
                                            display_msg,
                                            assignment->value_min);
        } else if (assignment->behavior == MD400_BEHAVIOR_MOMENTARY) {
            momentary_active = md400_gesture_momentary_active_ptr(event, control);
            if (momentary_active) {
                *momentary_active = 1;
                display_msg = md400_assignment_send_gate(assignment, control, 1);
                md400_play_show_assignment_sent(assignment, event, control,
                                                display_msg,
                                                assignment->value_max);
            } else {
                md400_assignment_send_gate(assignment, control, 1);
                display_msg = md400_assignment_send_gate(assignment, control, 0);
                md400_play_show_assignment_sent(assignment, event, control,
                                                display_msg,
                                                assignment->value_min);
            }
        } else {
            display_msg = md400_assignment_send_output_value(
                assignment, control, assignment->value_default);
            md400_play_show_assignment_sent(assignment, event, control,
                                            display_msg,
                                            assignment->value_default);
        }
    }
}

static void md400_encoder_short_timeout(void *priv)
{
    u8 index = (u32)priv;

    if (index >= MD400_ENCODER_COUNT) {
        return;
    }

    md400_button_gesture[index].short_timer = 0;
    if (md400_button_gesture[index].click_count == 1) {
        md400_emit_gesture(MD400_EVENT_ENCODER_SHORT, index + 1, 0);
    }
    md400_button_gesture[index].click_count = 0;
}

static void md400_encoder_click_release(u8 index)
{
    if (index >= MD400_ENCODER_COUNT) {
        return;
    }

#if !CONFIG_MD400_ENCODER_DOUBLE_PRESS_ENABLE
    if (md400_button_gesture[index].short_timer) {
        sys_timeout_del(md400_button_gesture[index].short_timer);
        md400_button_gesture[index].short_timer = 0;
    }
    md400_button_gesture[index].click_count = 0;
    md400_emit_gesture(MD400_EVENT_ENCODER_SHORT, index + 1, 0);
    return;
#endif

    if (md400_button_gesture[index].click_count == 0) {
        md400_button_gesture[index].click_count = 1;
        if (md400_button_gesture[index].short_timer) {
            sys_timeout_del(md400_button_gesture[index].short_timer);
        }
        md400_button_gesture[index].short_timer =
            sys_timeout_add((void *)(u32)index, md400_encoder_short_timeout, MD400_ENCODER_DOUBLE_MS);
    } else {
        if (md400_button_gesture[index].short_timer) {
            sys_timeout_del(md400_button_gesture[index].short_timer);
            md400_button_gesture[index].short_timer = 0;
        }
        md400_button_gesture[index].click_count = 0;
        md400_emit_gesture(MD400_EVENT_ENCODER_DOUBLE, index + 1, 0);
    }
}

static void md400_gesture_tick(void *priv)
{
    u32 now = sys_timer_get_ms();
    u8 i;

    (void)priv;

    if (md400_combo_mask && !md400_combo_long_sent &&
        ((now - md400_combo_start_ms) >= MD400_ENCODER_LONG_MS)) {
        md400_combo_long_sent = 1;
        md400_emit_gesture(MD400_EVENT_COMBO_LONG, md400_combo_mask, 0);
    }

    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        if (md400_button_gesture[i].down &&
            !md400_button_gesture[i].long_sent &&
            !(md400_combo_mask & BIT(i)) &&
            ((now - md400_button_gesture[i].down_ms) >= MD400_ENCODER_LONG_MS)) {
            md400_button_gesture[i].long_sent = 1;
            md400_emit_gesture(MD400_EVENT_ENCODER_LONG, i + 1, 0);
        }
    }
}

static void md400_gesture_start(void)
{
    if (!md400_gesture_timer) {
        md400_gesture_timer = sys_timer_add(NULL, md400_gesture_tick, MD400_GESTURE_TICK_MS);
    }
}

static void md400_encoder_button_gesture(u8 encoder, u8 down, u8 value)
{
    u8 index;
    u8 bit;
    u8 combo;
    u32 now;

    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return;
    }

    /* DAW mode always uses the full press/short/long/combo gesture engine. */
    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) && !md400_daw_active) {
        index = encoder - 1;
        bit = BIT(index);
        if (down) {
            md400_button_gesture[index].down = 1;
            md400_button_down_mask |= bit;
            combo = md400_combo_adjacent_mask(md400_button_down_mask);
            if (!md400_combo_mask && combo) {
                md400_combo_mask = combo;
                md400_combo_long_sent = 0;
                printf("[NEXFOOT_PEDAL] combo down=%02x\r\n",
                       (unsigned)combo);
            }
            return;
        }

        md400_button_gesture[index].down = 0;
        md400_button_down_mask &= ~bit;
        if (md400_combo_mask & bit) {
            if ((md400_button_down_mask & md400_combo_mask) == 0) {
                combo = md400_combo_mask;
                md400_combo_mask = 0;
                md400_emit_gesture(MD400_EVENT_COMBO_SHORT, combo, 0);
            }
            return;
        }
        md400_emit_gesture(MD400_EVENT_ENCODER_RELEASE, encoder, value);
        return;
    }

    md400_gesture_start();
    index = encoder - 1;
    bit = BIT(index);
    now = sys_timer_get_ms();

    if (down) {
        md400_button_gesture[index].down = 1;
        md400_button_gesture[index].long_sent = 0;
        md400_button_gesture[index].down_ms = now;
        md400_button_down_mask |= bit;
        md400_emit_gesture(MD400_EVENT_ENCODER_PRESS, encoder, value);

#if CONFIG_MD400_ENCODER_COMBO_PRESS_ENABLE || CONFIG_MD400_ENCODER_COMBO_NAV_ENABLE
        combo = md400_combo_adjacent_mask(md400_button_down_mask);
        if (!md400_combo_mask && combo &&
            (md400_combo_within_window(combo) ||
             md400_edit_combo_nav_overlap_allowed(combo))) {
            md400_combo_mask = combo;
            md400_combo_long_sent = 0;
            md400_combo_start_ms = now;
            if (combo & BIT(0)) {
                md400_button_gesture[0].click_count = 0;
            }
            if (combo & BIT(1)) {
                md400_button_gesture[1].click_count = 0;
            }
            if (combo & BIT(2)) {
                md400_button_gesture[2].click_count = 0;
            }
            if (combo & BIT(3)) {
                md400_button_gesture[3].click_count = 0;
            }
            md400_emit_gesture(MD400_EVENT_COMBO_PRESS, combo, 0);
        }
#endif
        return;
    }

    md400_button_gesture[index].down = 0;
    md400_button_down_mask &= ~bit;
    md400_emit_gesture(MD400_EVENT_ENCODER_RELEASE, encoder, value);

    if (md400_combo_mask & bit) {
        if ((md400_button_down_mask & md400_combo_mask) == 0) {
            if (!md400_combo_long_sent) {
                md400_emit_gesture(MD400_EVENT_COMBO_SHORT, md400_combo_mask, 0);
            } else {
                md400_release_combo_long_momentary(md400_combo_mask);
            }
            md400_combo_mask = 0;
            md400_combo_long_sent = 0;
        }
        return;
    }

    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        /* Edit buttons are release-driven UI keys. Ignore assignment behavior
         * and press duration here; a recognized combo returned above and owns
         * both releases, while an unpaired key always selects on release. */
        md400_emit_gesture(MD400_EVENT_ENCODER_SHORT, index + 1, 0);
        return;
    }

    if (!md400_button_gesture[index].long_sent &&
        ((now - md400_button_gesture[index].down_ms) < MD400_ENCODER_LONG_MS)) {
        md400_encoder_click_release(index);
    }
}

static u8 md400_scale_value(const md400_assignment_t *assignment, u8 value)
{
    u8 min = assignment->value_min;
    u8 max = assignment->value_max;

    value = md400_clip7(value);

    if (max >= min) {
        u8 span = max - min;
        return min + (u8)((((u16)value * span) + 63) / 127);
    }

    return min - (u8)((((u16)value * (min - max)) + 63) / 127);
}

static u8 md400_status_cin(u8 status)
{
    if ((status >= 0x80) && (status <= 0xEF)) {
        return (status >> 4) & 0x0f;
    }
    if ((status == 0xF1) || (status == 0xF3)) {
        return 0x2;
    }
    if (status == 0xF2) {
        return 0x3;
    }
    if (status == 0xF6) {
        return 0x5;
    }
    return 0x0f;
}

static void md400_send_usb_short(const u8 *midi, u8 len)
{
    u8 packet[4];
#if CONFIG_MD400_USB_MIDI_ONE_PORT_BASELINE
    const u8 cable = USB_MIDI_CABLE_CONTROL;
#else
    const u8 cable = USB_MIDI_CABLE_PORT;
#endif

    if (!midi || !len || (len > 3)) {
        return;
    }

    md400_usb_midi_tx_stream_clear();
    packet[0] = USB_MIDI_EVENT(cable, md400_status_cin(midi[0]));
    packet[1] = midi[0];
    packet[2] = (len >= 2) ? midi[1] : 0;
    packet[3] = (len >= 3) ? midi[2] : 0;
    usb_midi_device_send(0, packet, sizeof(packet));
}

static void md400_send_all_short(const u8 *midi, u8 len)
{
    u8 target_mask;

    if (!midi || !len) {
        return;
    }

    target_mask = md400_available_target_mask();
    if (target_mask & MD400_TARGET_USB_DEVICE) {
        md400_send_usb_short(midi, len);
    }
    if (target_mask & MD400_TARGET_BLE) {
        ble_midi_tx_write_drop_on_full(midi, len);
    }
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    if (target_mask & MD400_TARGET_USB_HOST) {
        usb1_midi_tx_write_drop_on_full(midi, len);
    }
#endif
#if CONFIG_MD400_MIDI_OUT_ENABLE
    if (target_mask & MD400_TARGET_MIDI_OUT) {
        uart_midi_tx_write_drop_on_full(midi, len);
    }
#endif
}

/* ------------------------------------------------------------------------
 * DAW mode: fixed control map and host session handling.
 *
 * Control map (all CC on MIDI channel 16, sent only to the DAW's transport):
 *   CC 16..19  encoder 1..4 rotate, relative (01..3F = +1..+63, 7F..41 = -1..-63)
 *   CC 20..23  encoder 1..4 press state (7F down, 00 up)
 *   CC 24..27  encoder 1..4 short press (7F)
 *   CC 28..31  encoder 1..4 long press (7F, sent while still held)
 *   CC 32..34  combo short 1+2, 2+3, 3+4 (7F)
 *   CC 35..37  combo long  1+2, 2+3, 3+4 (7F)
 *   CC 44      expression pedal (00..7F) or switch (00/7F)
 *   CC 46      edit key short (7F)
 *   CC 47      edit key long (7F)
 *   CC 127     DAW mode state (7F entered, 00 left)
 * Edit key double leaves DAW mode on the device.
 * ---------------------------------------------------------------------- */
#define MD400_DAW_STATUS_CC          0xBF
#define MD400_DAW_CC_ROTATE          16
#define MD400_DAW_CC_PRESS           20
#define MD400_DAW_CC_SHORT           24
#define MD400_DAW_CC_LONG            28
#define MD400_DAW_CC_COMBO_SHORT     32
#define MD400_DAW_CC_COMBO_LONG      35
#define MD400_DAW_CC_EXP             44
#define MD400_DAW_CC_EDIT_SHORT      46
#define MD400_DAW_CC_EDIT_LONG       47
#define MD400_DAW_CC_COMPACT_CUTOFF  118
#define MD400_DAW_CC_WIDGETS         119
#define MD400_DAW_CC_DIAL            124  /* 127: device can composite smooth dial edges */
#define MD400_DAW_CC_CUTOFF          125  /* 127: device can composite the cutoff pane */
#define MD400_DAW_CC_REDRAW          126  /* 127: drawing queue overflowed, host redraws */
#define MD400_DAW_CC_STATE           127
#define MD400_DAW_HOST_ABLETON       1
#define MD400_DAW_HOST_TIMEOUT_MS    10000
#define MD400_DAW_TICK_MS            250
#define MD400_DAW_OP_TEXT            0x01
#define MD400_DAW_OP_CARD            0x02
#define MD400_DAW_OP_LEDS            0x03
#define MD400_DAW_OP_ACCENT          0x04
#define MD400_DAW_OP_PADS            0x05  /* MASK CH N1 N2 N3 N4: switches play notes */

/* Compact controller App ABI. Records 250/251 live in the append-only
 * journal. All reads and installs run on app_core in the NanoBridge handler. */
struct md400_daw_app {
    u8 schema, host, version, wait, rotate, press, tap, hold, expression;
    char title[12];
};
static struct md400_daw_app active_app;
static u8 md400_daw_app_valid(const struct md400_daw_app *app, u8 host)
{
    unsigned i;
    if (app->schema != 1 || app->host != host || !app->version ||
        app->wait > 1 || app->rotate > 124 || app->press > 124 ||
        app->tap > 124 || app->hold > 124 || !app->title[0]) return 0;
    for (i = 0; i < sizeof(app->title); i++)
        if (app->title[i] < 32 || app->title[i] > 126) return 0;
    return 1;
}

static u8 md400_daw_app_command(const u8 *payload, u8 len, u8 *reply, u8 *reply_len)
{
    struct md400_daw_app app;
    u8 host;
    int result;
    if (len != 1 && len != sizeof(app)) return MD400_NB_STATUS_BAD_LENGTH;
    host = payload[len == 1 ? 0 : 1];
    if (host < 1 || host > 2) return MD400_NB_STATUS_BAD_VALUE;
    if (len == sizeof(app)) {
        memcpy(&app, payload, sizeof(app));
        if (!md400_daw_app_valid(&app, host)) return MD400_NB_STATUS_BAD_VALUE;
        if (syscfg_write(249 + host, &app, sizeof(app)) != sizeof(app))
            return MD400_NB_STATUS_FLASH_ERROR;
    }
    result = syscfg_read(249 + host, &app, sizeof(app));
    if (result == MD400_FS_NOT_FOUND && len == 1) {
        reply[0] = 0;
        *reply_len = 1;
    } else if (result == sizeof(app) && md400_daw_app_valid(&app, host)) {
        memcpy(reply, &app, sizeof(app));
        *reply_len = sizeof(app);
    } else return MD400_NB_STATUS_FLASH_ERROR;
    return MD400_NB_STATUS_OK;
}

static void md400_daw_send_cc(u8 cc, u8 value)
{
    u8 midi[3];

    midi[0] = MD400_DAW_STATUS_CC;
    midi[1] = cc & 0x7f;
    midi[2] = value & 0x7f;
    if (md400_daw_source == 1) {
        ble_midi_tx_write_drop_on_full(midi, sizeof(midi));
    } else {
        md400_send_usb_short(midi, sizeof(midi));
    }
}

static void md400_daw_send_note(u8 index, u8 on)
{
    u8 midi[3];

    midi[0] = (u8)((on ? 0x90 : 0x80) | (md400_daw_pad_channel & 0x0f));
    midi[1] = md400_daw_pad_note[index] & 0x7f;
    midi[2] = on ? 100 : 0;
    if (md400_daw_source == 1) {
        ble_midi_tx_write_drop_on_full(midi, sizeof(midi));
    } else {
        md400_send_usb_short(midi, sizeof(midi));
    }
}

/* Release any sounding pad notes, e.g. when the host changes the map. */
static void md400_daw_pads_off(void)
{
    u8 i;

    for (i = 0; i < 4; i++) {
        if (md400_daw_pad_down & BIT(i)) {
            md400_daw_send_note(i, 0);
        }
    }
    md400_daw_pad_down = 0;
}

static u8 md400_daw_combo_index(u8 mask)
{
    if (mask == 0x03) {
        return 0;
    }
    if (mask == 0x06) {
        return 1;
    }
    if (mask == 0x0c) {
        return 2;
    }
    return 0xff;
}

static void md400_daw_gesture(u8 event, u8 control)
{
    u8 combo;

    if (md400_daw_waiting) {
        return;
    }

    if ((event == MD400_EVENT_COMBO_SHORT) || (event == MD400_EVENT_COMBO_LONG)) {
        combo = md400_daw_combo_index(control);
        if (combo != 0xff) {
            md400_daw_send_cc(((event == MD400_EVENT_COMBO_SHORT) ?
                               MD400_DAW_CC_COMBO_SHORT : MD400_DAW_CC_COMBO_LONG) + combo,
                              0x7f);
        }
        return;
    }
    if ((control < 1) || (control > MD400_ENCODER_COUNT)) {
        return;
    }
    control -= 1;
    switch (event) {
    case MD400_EVENT_ENCODER_PRESS:
        if ((md400_daw_pad_mask & BIT(control)) && !(md400_daw_pad_down & BIT(control))) {
            md400_daw_pad_down |= BIT(control);
            md400_daw_send_note(control, 1);
        }
        md400_daw_send_cc(active_app.press + control, 0x7f);
        break;
    case MD400_EVENT_ENCODER_RELEASE:
        if (md400_daw_pad_down & BIT(control)) {
            md400_daw_pad_down &= ~BIT(control);
            md400_daw_send_note(control, 0);
        }
        md400_daw_send_cc(active_app.press + control, 0x00);
        break;
    case MD400_EVENT_ENCODER_SHORT:
        md400_daw_send_cc(active_app.tap + control, 0x7f);
        break;
    case MD400_EVENT_ENCODER_LONG:
        md400_daw_send_cc(active_app.hold + control, 0x7f);
        break;
    default:
        break;
    }
}

static void md400_daw_rotate(u8 encoder, u8 value, u8 event)
{
    u8 index;
    int delta;

    if (md400_daw_waiting || (encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return;
    }
    index = encoder - 1;
    value &= 0x7f;
    delta = (int)value - (int)md400_daw_enc_prev[index];
    if (event == MD400_EVENT_ENCODER_LEFT) {
        if (delta >= 0) {
            delta = -1;
        }
    } else if (event == MD400_EVENT_ENCODER_RIGHT) {
        if (delta <= 0) {
            delta = 1;
        }
    } else {
        return;
    }
    if (delta > 63) {
        delta = 63;
    } else if (delta < -63) {
        delta = -63;
    }
    md400_daw_enc_prev[index] = value;
    /* The MS32 counter is absolute and clamps at 0/127; recentre it before
     * it reaches an end so relative turns never stall. */
    if ((value < 16) || (value > 111)) {
        md400_ms32_set_encoder_value(encoder, 64);
        md400_daw_enc_prev[index] = 64;
    }
    md400_daw_send_cc(active_app.rotate + index,
                      (u8)((delta > 0) ? delta : (128 + delta)));
}

static void md400_daw_pedal_value(u8 value)
{
    value = md400_clip7(value);
    if (md400_pedal_seen && (value == md400_pedal_runtime_value)) {
        return;
    }
    md400_pedal_seen = 1;
    md400_pedal_runtime_value = value;
    if (!md400_daw_waiting) {
        md400_daw_send_cc(active_app.expression, value);
    }
}

static void md400_daw_reset_gestures(void)
{
    md400_button_down_mask = 0;
    md400_combo_mask = 0;
    md400_combo_long_sent = 0;
    memset(md400_button_gesture, 0, sizeof(md400_button_gesture));
}

static void md400_daw_enter(void)
{
    u8 i;

    md400_daw_active = 1;
    md400_daw_waiting = 0;
    md400_daw_reset_gestures();
    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        md400_daw_enc_prev[i] = (u8)md400_ms32_get_encoder_value(i + 1);
    }
    md400_display_daw_set_active(1);
    md400_display_daw_set_waiting(0);
    md400_led_sync();
    md400_daw_send_cc(MD400_DAW_CC_STATE, 0x7f);
    if (!md400_daw_source && md400_display_daw_can_cutoff()) {
        md400_daw_send_cc(MD400_DAW_CC_CUTOFF, 0x7f);
        md400_daw_send_cc(MD400_DAW_CC_DIAL, 0x7f);
        md400_daw_send_cc(MD400_DAW_CC_WIDGETS, 0x7f);
        md400_daw_send_cc(MD400_DAW_CC_COMPACT_CUTOFF, 0x7f);
    }
    printf("[MD400_DAW] enter host=%u source=%u\r\n",
           (unsigned)md400_daw_host_id, (unsigned)md400_daw_source);
}

static void md400_daw_leave(void)
{
    if (!md400_daw_active) {
        return;
    }
    md400_daw_active = 0;
    md400_daw_waiting = 0;
    md400_daw_pads_off();
    md400_daw_pad_mask = 0;
    md400_daw_reset_gestures();
    md400_display_daw_set_active(0);
    md400_sync_ms32_encoder_runtime_values();
    md400_ui_display_sync();
    md400_daw_send_cc(MD400_DAW_CC_STATE, 0x00);
    printf("[MD400_DAW] leave\r\n");
}

static void md400_daw_host_lost(void)
{
    md400_daw_host_seen = 0;
    md400_daw_requested = 0;
    md400_daw_user_off = 0;
    if (md400_daw_active && !md400_daw_source && active_app.wait) {
        md400_daw_waiting = 1;
        md400_daw_pads_off();
        md400_daw_pad_mask = 0;
        md400_daw_led_mask = 0;
        md400_daw_draw_overflow = 0;
        md400_daw_reset_gestures();
        md400_display_daw_set_waiting(1);
        md400_led_sync();
        md400_daw_send_cc(MD400_DAW_CC_STATE, 0);
    } else {
        md400_daw_leave();
    }
}

static u8 md400_daw_host_alive(void)
{
    return md400_daw_host_seen &&
           ((u32)(sys_timer_get_ms() - md400_daw_host_ms) <= MD400_DAW_HOST_TIMEOUT_MS);
}

static void md400_daw_host_touch(void)
{
    md400_daw_host_seen = 1;
    md400_daw_host_ms = sys_timer_get_ms();
}

static void md400_daw_timer_tick(void *priv)
{
    extern u8 md400_board_power_is_softoff(void);

    (void)priv;
    if (md400_daw_draw_overflow) {
        md400_daw_draw_overflow = 0;
        if (md400_daw_active) {
            md400_daw_send_cc(MD400_DAW_CC_REDRAW, 0x7f);
        }
    }
    if (md400_daw_host_seen && !md400_daw_host_alive()) {
        md400_daw_host_lost();
        printf("[MD400_DAW] host timeout\r\n");
        return;
    }
    if (md400_daw_requested && !md400_daw_active && !md400_daw_user_off &&
        (md400_ui_mode != MD400_UI_MODE_EDIT) && !md400_board_power_is_softoff()) {
        md400_daw_enter();
    }
}

/* Edit-key handling while a DAW host is present. Returns 1 if consumed. */
static u8 md400_daw_edit_key(u8 event)
{
    if (md400_daw_active) {
        if (event == MD400_EVENT_EDIT_SHORT && !md400_daw_waiting) {
            md400_daw_send_cc(MD400_DAW_CC_EDIT_SHORT, 0x7f);
        } else if (event == MD400_EVENT_EDIT_LONG && !md400_daw_waiting) {
            md400_daw_send_cc(MD400_DAW_CC_EDIT_LONG, 0x7f);
        } else if (event == MD400_EVENT_EDIT_DOUBLE) {
            /* Foot-mode cycle MIDI -> PEDAL -> DAW -> MIDI. */
            md400_daw_user_off = 1;
            md400_daw_leave();
            if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
                md400_foot_mode_set(MD400_FOOT_MODE_MIDI);
            }
        }
        return 1;
    }
    if ((event == MD400_EVENT_EDIT_DOUBLE) &&
        (md400_ui_mode == MD400_UI_MODE_PERFORM) &&
        (md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        md400_daw_requested && md400_daw_host_alive()) {
        md400_daw_user_off = 0;
        md400_daw_enter();
        return 1;
    }
    return 0;
}

static void md400_edit_key_event(u8 event)
{
    if (md400_daw_edit_key(event)) {
        return;
    }
    md400_ui_event(event);
}

/* NanoBridge DAW_MODE (4E): ENABLE [HOST_ID]. Reply: ACTIVE PENDING. */
static u8 md400_daw_nb_mode(const u8 *payload, u8 payload_len, u8 *reply, u8 *reply_len)
{
    if ((payload_len < 1) || (payload_len > 2) || (payload[0] > 1)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload[0]) {
        u8 host = payload_len == 2 ? payload[1] : 0;
        struct md400_daw_app next;
        if (host == 0) {
            /* Keep generic SDK controllers compatible. */
            struct md400_daw_app generic = {1, 0, 1, 0, 16, 20, 24, 28, 44, "DAW"};
            next = generic;
        } else if (host > 2 || syscfg_read(249 + host, &next, sizeof(next)) != sizeof(next)
                   || !md400_daw_app_valid(&next, host)) return MD400_NB_STATUS_UNSUPPORTED;
        active_app = next;
        md400_display_daw_app_name(next.title);
        md400_daw_pads_off();
        md400_daw_pad_mask = 0;
        md400_daw_draw_overflow = 0;
        md400_daw_host_touch();
        md400_daw_source = md400_nb_origin.source ? 1 : 0;
        md400_daw_host_id = (payload_len > 1) ? payload[1] : 0;
        md400_daw_requested = 1;
        md400_daw_user_off = 0;
        md400_daw_led_mask = 0;
        md400_display_daw_reset();
        if (md400_daw_active) {
            md400_daw_enter();
        } else {
            md400_daw_timer_tick(NULL);
        }
    } else if (md400_daw_waiting && payload_len == 2 && payload[1] != md400_daw_host_id) {
        md400_daw_leave();
    } else if (!md400_daw_waiting) {
        md400_daw_host_lost();
    }
    reply[0] = md400_daw_active && !md400_daw_waiting;
    reply[1] = (md400_daw_requested && !md400_daw_active) ? 1 : 0;
    /* diagnostics: DAW_DRAW frames queued / dropped, USB packets rate-limited */
    md400_put_u14(&reply[2], md400_daw_draw_rx_count & 0x3fff);
    md400_put_u14(&reply[4], md400_daw_draw_drop_count & 0x3fff);
    md400_put_u14(&reply[6], md400_usb_rx_limited_count & 0x3fff);
    *reply_len = 8;
    return MD400_NB_STATUS_OK;
}

/* NanoBridge DAW_UPDATE (4F): a list of ops, applied in order.
 *   01 FIELD LEN ASCII...   text (0 title, 1 status, 2..5 label, 6..9 value)
 *   02 CARD METER R5 G6 B5 FLAGS
 *   03 LED_MASK
 *   04 R5 G6 B5             top-bar accent colour
 * Reply: ACTIVE. A request with SEQ 0 gets no reply (fire and forget). */
static u8 md400_daw_nb_update(const u8 *payload, u8 payload_len, u8 *reply, u8 *reply_len)
{
    u8 i = 0;
    u8 op;
    u16 color;

    if (md400_daw_waiting) {
        reply[0] = 0;
        *reply_len = 1;
        return MD400_NB_STATUS_OK;
    }
    if (!active_app.schema) return MD400_NB_STATUS_UNSUPPORTED;
    md400_daw_host_touch();
    if (!md400_daw_requested && !md400_daw_user_off) {
        /* Host updates imply a DAW session (e.g. the pedal rebooted while the
         * DAW kept running). The watchdog tick enters DAW mode; on entry the
         * host sees CC 127 = 127 and resends its full view. */
        md400_daw_requested = 1;
        md400_daw_source = md400_nb_origin.source ? 1 : 0;
    }
    while (i < payload_len) {
        op = payload[i++];
        if (op == MD400_DAW_OP_TEXT) {
            u8 field;
            u8 len;

            if ((u16)i + 2 > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            field = payload[i];
            len = payload[i + 1];
            i += 2;
            if ((u16)i + len > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            if (!md400_display_daw_set_text(field, &payload[i], len)) {
                return MD400_NB_STATUS_BAD_VALUE;
            }
            i += len;
        } else if (op == MD400_DAW_OP_CARD) {
            if ((u16)i + 6 > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            color = (u16)(((payload[i + 2] & 0x1f) << 11) |
                          ((payload[i + 3] & 0x3f) << 5) |
                          (payload[i + 4] & 0x1f));
            if (!md400_display_daw_set_card(payload[i], payload[i + 1], color,
                                            payload[i + 5])) {
                return MD400_NB_STATUS_BAD_VALUE;
            }
            i += 6;
        } else if (op == MD400_DAW_OP_LEDS) {
            if ((u16)i + 1 > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            if (md400_daw_led_mask != (payload[i] & 0x0f)) {
                md400_daw_led_mask = payload[i] & 0x0f;
                if (md400_daw_active) {
                    md400_led_sync();
                }
            }
            i += 1;
        } else if (op == MD400_DAW_OP_PADS) {
            if ((u16)i + 6 > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            md400_daw_pads_off();
            md400_daw_pad_mask = payload[i] & 0x0f;
            md400_daw_pad_channel = payload[i + 1] & 0x0f;
            md400_daw_pad_note[0] = payload[i + 2] & 0x7f;
            md400_daw_pad_note[1] = payload[i + 3] & 0x7f;
            md400_daw_pad_note[2] = payload[i + 4] & 0x7f;
            md400_daw_pad_note[3] = payload[i + 5] & 0x7f;
            i += 6;
        } else if (op == MD400_DAW_OP_ACCENT) {
            if ((u16)i + 3 > payload_len) {
                return MD400_NB_STATUS_BAD_LENGTH;
            }
            md400_display_daw_set_accent((u16)(((payload[i] & 0x1f) << 11) |
                                               ((payload[i + 1] & 0x3f) << 5) |
                                               (payload[i + 2] & 0x1f)));
            i += 3;
        } else {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }
    reply[0] = md400_daw_active;
    *reply_len = 1;
    return MD400_NB_STATUS_OK;
}

static void md400_send_usb_sysex(const u8 *raw, u8 raw_len)
{
    u8 packet[((MD400_SYSEX_TEMPLATE_MAX_BODY + 4) / 3) * 4];
    u16 raw_pos = 0;
    u16 out_len = 0;
#if CONFIG_MD400_USB_MIDI_ONE_PORT_BASELINE
    const u8 cable = USB_MIDI_CABLE_CONTROL;
#else
    const u8 cable = USB_MIDI_CABLE_PORT;
#endif

    if (!raw || !raw_len || raw_len > MD400_SYSEX_TEMPLATE_MAX_BODY + 2) return;
    while (raw_pos < raw_len) {
        u16 remain = raw_len - raw_pos;
        u8 cin;
        u8 take;


        if (remain >= 3) {
            if (raw[raw_pos] == 0xF7) {
                cin = 0x5;
                take = 1;
            } else if (raw[raw_pos + 1] == 0xF7) {
                cin = 0x6;
                take = 2;
            } else if (raw[raw_pos + 2] == 0xF7) {
                cin = 0x7;
                take = 3;
            } else {
                cin = 0x4;
                take = 3;
            }
        } else if (remain == 2) {
            cin = 0x6;
            take = 2;
        } else {
            cin = 0x5;
            take = 1;
        }

        packet[out_len++] = USB_MIDI_EVENT(cable, cin);
        packet[out_len++] = raw[raw_pos];
        packet[out_len++] = (take >= 2) ? raw[raw_pos + 1] : 0;
        packet[out_len++] = (take >= 3) ? raw[raw_pos + 2] : 0;
        raw_pos += take;
    }

    if (out_len) {
        usb_midi_device_send(0, packet, out_len);
    }
}

static void md400_send_all_sysex(const u8 *raw, u8 raw_len)
{
    u8 target_mask;

    if (!raw || !raw_len) {
        return;
    }
    target_mask = md400_available_target_mask();
    if (target_mask & MD400_TARGET_USB_DEVICE) {
        md400_send_usb_sysex(raw, raw_len);
    }
    if (target_mask & MD400_TARGET_BLE) {
        ble_midi_tx_write_drop_on_full(raw, raw_len);
    }
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    if (target_mask & MD400_TARGET_USB_HOST) {
        usb1_midi_tx_write_drop_on_full(raw, raw_len);
    }
#endif
#if CONFIG_MD400_MIDI_OUT_ENABLE
    if (target_mask & MD400_TARGET_MIDI_OUT) {
        uart_midi_tx_write_drop_on_full(raw, raw_len);
    }
#endif
}

static u8 md400_edit_subject_event(u8 subject)
{
    switch (subject) {
    case MD400_EDIT_SUBJECT_SHORT:
        return MD400_EVENT_ENCODER_SHORT;
    case MD400_EDIT_SUBJECT_LONG:
        return MD400_EVENT_ENCODER_LONG;
    case MD400_EDIT_SUBJECT_DOUBLE:
        return MD400_EVENT_ENCODER_DOUBLE;
    case MD400_EDIT_SUBJECT_PRESS:
        return MD400_EVENT_ENCODER_PRESS;
    case MD400_EDIT_SUBJECT_COMBO_PRESS:
        return MD400_EVENT_COMBO_PRESS;
    case MD400_EDIT_SUBJECT_COMBO_SHORT:
        return MD400_EVENT_COMBO_SHORT;
    case MD400_EDIT_SUBJECT_COMBO_LONG:
        return MD400_EVENT_COMBO_LONG;
    default:
        return 0;
    }
}

static u8 md400_edit_active_gesture_event(void)
{
    if ((md400_edit_subject == MD400_EDIT_SUBJECT_SHORT) &&
        ((md400_edit_page == MD400_EDIT_PAGE_MESSAGE) ||
         (md400_edit_page == MD400_EDIT_PAGE_VALUE))) {
        return MD400_EVENT_ENCODER_PRESS;
    }

    return md400_edit_subject_event(md400_edit_subject);
}

static u8 md400_edit_subject_valid(u8 subject)
{
    if (subject >= MD400_EDIT_SUBJECT_COUNT) {
        return 0;
    }

    if (subject == MD400_EDIT_SUBJECT_PEDAL) {
        return 1;
    }

#if !CONFIG_MD400_ENCODER_SHORT_PRESS_ENABLE
    if (subject == MD400_EDIT_SUBJECT_SHORT) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_PRESS_EDIT_ENABLE
    if (subject == MD400_EDIT_SUBJECT_PRESS) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_DOUBLE_PRESS_ENABLE
    if (subject == MD400_EDIT_SUBJECT_DOUBLE) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_LONG_PRESS_ENABLE
    if (subject == MD400_EDIT_SUBJECT_LONG) {
        return 0;
    }
#endif
#if !CONFIG_MD400_ENCODER_COMBO_PRESS_ENABLE
    if ((subject == MD400_EDIT_SUBJECT_COMBO_PRESS) ||
        (subject == MD400_EDIT_SUBJECT_COMBO_SHORT) ||
        (subject == MD400_EDIT_SUBJECT_COMBO_LONG)) {
        return 0;
    }
#endif
    return 1;
}

static u8 md400_edit_subject_is_combo(u8 subject)
{
    return (subject == MD400_EDIT_SUBJECT_COMBO_PRESS) ||
           (subject == MD400_EDIT_SUBJECT_COMBO_SHORT) ||
           (subject == MD400_EDIT_SUBJECT_COMBO_LONG);
}

static u8 md400_edit_combo_number(void)
{
    if (md400_edit_target_encoder <= 1) {
        return 1;
    }
    if (md400_edit_target_encoder == 2) {
        return 2;
    }
    return 3;
}

static u8 md400_edit_combo_control(void)
{
    switch (md400_edit_combo_number()) {
    case 1:
        return 0x03;
    case 2:
        return 0x06;
    default:
        return 0x0c;
    }
}

static u8 md400_edit_control(void)
{
    if (md400_edit_subject_is_combo(md400_edit_subject)) {
        return md400_edit_combo_control();
    }
    if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
        return 5;
    }
    return md400_edit_target_encoder;
}

static u8 md400_edit_page_count(void)
{
    return (md400_edit_subject == MD400_EDIT_SUBJECT_SHORT) ? 4 : 2;
}

static u8 md400_edit_page_is_preset_select(void)
{
    return md400_edit_page == MD400_EDIT_PAGE_PRESET;
}

static u8 md400_edit_page_is_rotate(void)
{
    if (md400_edit_page_is_preset_select()) {
        return 0;
    }
    return (md400_edit_subject == MD400_EDIT_SUBJECT_ROTATE) ||
           ((md400_edit_subject == MD400_EDIT_SUBJECT_SHORT) &&
            (md400_edit_page >= MD400_EDIT_PAGE_SHORT_MESSAGE));
}

static u8 md400_edit_page_is_message(void)
{
    return (md400_edit_page == MD400_EDIT_PAGE_MESSAGE) ||
           ((md400_edit_subject == MD400_EDIT_SUBJECT_SHORT) &&
            (md400_edit_page == MD400_EDIT_PAGE_SHORT_MESSAGE));
}

static u8 md400_edit_display_page(void)
{
    if (md400_edit_page_is_preset_select()) {
        return 0;
    }
    return ((md400_edit_page - 1) % md400_edit_page_count()) + 1;
}

static void md400_edit_select_from_value(u8 value)
{
    if (!md400_edit_subject_valid(md400_edit_subject)) {
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    }
    md400_edit_page = MD400_EDIT_PAGE_MESSAGE + (value % md400_edit_page_count());
}

static void md400_edit_select_from_event_or_value(u8 value, u8 event)
{
    u8 page_count;
    u8 selector = 0;

    if ((event != MD400_EVENT_ENCODER_LEFT) &&
        (event != MD400_EVENT_ENCODER_RIGHT)) {
        md400_edit_select_from_value(value);
        return;
    }

    if (!md400_edit_subject_valid(md400_edit_subject)) {
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    }

    page_count = md400_edit_page_count();
    if ((md400_edit_page >= MD400_EDIT_PAGE_MESSAGE) &&
        (md400_edit_page <= page_count)) {
        selector = md400_edit_page - MD400_EDIT_PAGE_MESSAGE;
    }

    if (event == MD400_EVENT_ENCODER_LEFT) {
        selector = selector ? (selector - 1) : (page_count - 1);
    } else {
        selector++;
        if (selector >= page_count) {
            selector = 0;
        }
    }

    md400_edit_page = MD400_EDIT_PAGE_MESSAGE + selector;
}

static void md400_edit_normalize_subject_for_page(void)
{
    if (!md400_edit_subject_valid(md400_edit_subject)) {
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    }

    if (md400_edit_page_is_preset_select()) {
        return;
    }
    if ((md400_edit_page < MD400_EDIT_PAGE_MESSAGE) ||
        (md400_edit_page > md400_edit_page_count())) {
        md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
    }
}

static md400_assignment_t *md400_edit_assignment(void)
{
    if ((md400_edit_target_encoder < 1) || (md400_edit_target_encoder > MD400_ENCODER_COUNT)) {
        md400_edit_target_encoder = 1;
    }

    md400_edit_normalize_subject_for_page();
    if (md400_edit_page_is_preset_select()) {
        return NULL;
    }
    if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
        return md400_get_pedal_edit_assignment();
    }
    if (md400_edit_page_is_rotate()) {
        return md400_get_assignment(md400_edit_target_encoder);
    }

    return md400_get_gesture_assignment(md400_edit_active_gesture_event(),
                                        md400_edit_control());
}

static u8 md400_edit_subject_for_page(u8 page)
{
    (void)page;
    if (!md400_edit_subject_valid(md400_edit_subject)) {
        return MD400_EDIT_DEFAULT_SUBJECT;
    }
    return md400_edit_subject;
}

static int md400_edit_display_pack(u8 kind, u8 value)
{
    return (((int)(kind & 0x7f)) << 8) | (value & 0x7f);
}

static u8 md400_edit_display_message_number(const md400_assignment_t *assignment)
{
    if (!assignment) {
        return 0;
    }

    switch (assignment->type) {
    case MD400_MIDI_TYPE_CC:
    case MD400_MIDI_TYPE_NOTE:
    case MD400_MIDI_TYPE_NOTE_OFF:
        return assignment->data1 & 0x7f;
    case MD400_MIDI_TYPE_SYSEX:
        return md400_assignment_sysex_template_id(assignment);
    case MD400_MIDI_TYPE_CUSTOM:
        return md400_assignment_custom_template_id(assignment);
    case MD400_MIDI_TYPE_PC:
    case MD400_MIDI_TYPE_PITCH:
    case MD400_MIDI_TYPE_PRESSURE:
        return assignment->value_default & 0x7f;
    default:
        break;
    }
    return 0;
}

static void md400_edit_set_message_number(md400_assignment_t *assignment,
                                          u8 value)
{
    if (!assignment) {
        return;
    }

    value &= 0x7f;
    switch (assignment->type) {
    case MD400_MIDI_TYPE_CC:
    case MD400_MIDI_TYPE_NOTE:
    case MD400_MIDI_TYPE_NOTE_OFF:
        assignment->data1 = value;
        break;
    case MD400_MIDI_TYPE_SYSEX:
        assignment->sysex_id = value;
        break;
    case MD400_MIDI_TYPE_CUSTOM:
        assignment->sysex_id = value;
        break;
    case MD400_MIDI_TYPE_PC:
    case MD400_MIDI_TYPE_PITCH:
    case MD400_MIDI_TYPE_PRESSURE:
        assignment->value_default = value;
        break;
    default:
        assignment->data1 = value;
        break;
    }
}

static int md400_perform_display_msg(const md400_assignment_t *assignment)
{
    if (!assignment) {
        return MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_OFF, 0);
    }
    return MD400_DISPLAY_MSG_PACK(assignment->type,
                                  md400_edit_display_message_number(assignment));
}

static int md400_play_display_msg_from_short(const u8 *midi, u8 len)
{
    if (!midi || !len) {
        return MD400_PLAY_MSG_INVALID;
    }
    if (len > 3) {
        len = 3;
    }
    return MD400_DISPLAY_MSG_RAW_PACK(len, midi[0],
                                      (len > 1) ? midi[1] : 0,
                                      (len > 2) ? midi[2] : 0);
}

static int md400_play_display_msg_sysex(void)
{
    return MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_SYSEX, 0);
}

static int md400_play_display_msg_from_output(const md400_assignment_t *assignment,
                                              u8 out_value, u8 note_off)
{
    u8 midi[3];

    if (!assignment || (assignment->type == MD400_MIDI_TYPE_OFF)) {
        return MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_OFF, 0);
    }

    out_value = md400_clip7(out_value);
    switch (assignment->type) {
    case MD400_MIDI_TYPE_CC:
        midi[0] = 0xB0 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PC:
        midi[0] = 0xC0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_NOTE:
        midi[0] = (note_off ? 0x80 : 0x90) | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_NOTE_OFF:
        midi[0] = 0x80 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PITCH: {
        u16 bend = ((u16)out_value) << 7;
        midi[0] = 0xE0 | (assignment->channel & 0x0f);
        midi[1] = bend & 0x7f;
        midi[2] = (bend >> 7) & 0x7f;
        return md400_play_display_msg_from_short(midi, 3);
    }
    case MD400_MIDI_TYPE_PRESSURE:
        midi[0] = 0xD0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_SYSEX:
        return md400_play_display_msg_sysex();
    default:
        break;
    }

    return md400_perform_display_msg(assignment);
}

static void md400_play_show_assignment_msg(const md400_assignment_t *assignment,
                                           u8 action, u8 control, int msg,
                                           u8 out_value)
{
    char label[MD400_PLAY_LABEL_LEN + 1];

    if (!assignment) {
        return;
    }

    out_value = md400_clip7(out_value);
    if (msg == MD400_PLAY_MSG_INVALID) {
        msg = md400_play_display_msg_from_output(assignment, out_value, 0);
    }

    md400_play_last_msg = msg;
    md400_play_last_value = out_value;
    md400_play_last_action = action;
    md400_play_last_control = control;

    md400_play_label_to_cstr(action, control, label, sizeof(label));
    md400_display_play_update(md400_play_last_msg, md400_play_last_value, label);
}

static void md400_play_refresh_display(void)
{
    char label[MD400_PLAY_LABEL_LEN + 1];

    md400_play_label_to_cstr(md400_play_last_action, md400_play_last_control,
                             label, sizeof(label));
    md400_display_play_update(md400_play_last_msg, md400_play_last_value, label);
}

static void md400_play_show_assignment_value(const md400_assignment_t *assignment,
                                             u8 action, u8 control, u8 value,
                                             u8 value_is_output)
{
    u8 out_value = 0;
    int msg;

    if (!assignment) {
        return;
    }

    value = md400_clip7(value);
    if (assignment->type != MD400_MIDI_TYPE_OFF) {
        out_value = value_is_output ? value : md400_scale_value(assignment, value);
    }

    msg = md400_play_display_msg_from_output(assignment, out_value, 0);
    md400_play_show_assignment_msg(assignment, action, control, msg, out_value);
}

static void md400_play_show_assignment(const md400_assignment_t *assignment,
                                       u8 action, u8 control, u8 value)
{
    md400_play_show_assignment_value(assignment, action, control, value, 0);
}

static void md400_play_show_assignment_output(const md400_assignment_t *assignment,
                                             u8 action, u8 control, u8 value)
{
    md400_play_show_assignment_value(assignment, action, control, value, 1);
}

static void md400_play_show_assignment_sent(const md400_assignment_t *assignment,
                                            u8 action, u8 control, int msg,
                                            u8 out_value)
{
    md400_play_show_assignment_msg(assignment, action, control, msg, out_value);
}

static void md400_edit_gesture_state_reset(void)
{
    u8 event;
    u8 *state;

    if (md400_edit_page_is_rotate()) {
        return;
    }

    event = md400_edit_active_gesture_event();
    if (!event) {
        return;
    }

    state = md400_gesture_toggle_state_ptr(event, md400_edit_control());
    if (state) {
        *state = 0;
    }
    state = md400_gesture_momentary_active_ptr(event, md400_edit_control());
    if (state) {
        *state = 0;
    }
}

static u8 md400_edit_led_base_mask(void)
{
    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        (md400_edit_subject != MD400_EDIT_SUBJECT_PEDAL)) {
        if ((md400_pedal_edit_slot >= 1) &&
            (md400_pedal_edit_slot <= MD400_PEDAL_SLOT_COUNT)) {
            return BIT(md400_pedal_edit_slot - 1) & 0x0f;
        }
        return 0;
    }

    if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
        return 0;
    }

    if (md400_edit_subject_is_combo(md400_edit_subject)) {
        return md400_edit_combo_control() & 0x0f;
    }

    if ((md400_edit_target_encoder >= 1) &&
        (md400_edit_target_encoder <= MD400_ENCODER_COUNT)) {
        return BIT(md400_edit_target_encoder - 1) & 0x0f;
    }

    return 0;
}

static u8 md400_edit_led_pattern_mask(u8 base)
{
    u8 phase;

    if (!base) {
        return 0;
    }

    if (md400_edit_subject == MD400_EDIT_SUBJECT_DOUBLE) {
        phase = (sys_timer_get_ms() / 120) % 8;
        return ((phase == 0) || (phase == 2)) ? base : 0;
    }

    if (md400_edit_subject == MD400_EDIT_SUBJECT_LONG) {
        return ((sys_timer_get_ms() / 260) & 0x01) ? 0 : base;
    }

    return base;
}

static u8 md400_led_compute_mask(void)
{
    u8 mask = 0;
    u8 i;
    md400_preset_t *preset;

    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        if (md400_edit_page_is_preset_select()) {
            mask = 0;
        } else {
            mask = md400_edit_led_pattern_mask(md400_edit_led_base_mask());
        }
    } else {
        preset = (md400_active_preset < MD400_PRESET_COUNT) ?
                 &md400_store.preset[md400_active_preset] : NULL;
        if (!preset) {
            return 0;
        }
        if (preset->led_mode == MD400_LED_MODE_OFF) {
            mask = 0;
        } else if (preset->led_mode == MD400_LED_MODE_MANUAL) {
            mask = preset->led_mask & 0x0f;
        } else {
            for (i = 0; i < MD400_ENCODER_COUNT; i++) {
                if (md400_ms32_get_encoder_pressed(i + 1)) {
                    mask |= BIT(i);
                }
            }
            /* MIDI toggle latches do not belong on the PEDAL surface. */
            if (md400_foot_mode != MD400_FOOT_MODE_PEDAL) {
                mask |= md400_toggle_led_mask();
            }
        }
    }

    return mask & 0x0f;
}

static u8 md400_edit_counter_value_from_role(u8 role, int row_value)
{
    u8 value = row_value & 0x7f;

    switch (role) {
    case MD400_DISPLAY_ROLE_NONE:
        return 0;
    case MD400_DISPLAY_ROLE_EDIT_CHANNEL:
        return value ? ((value - 1) & 0x0f) : 0;
    case MD400_DISPLAY_ROLE_EDIT_PAGE:
        return value ? ((value - 1) & 0x7f) : 0;
    case MD400_DISPLAY_ROLE_PRESET_SLOT:
        return value ? ((value - 1) & 0x7f) : 0;
    case MD400_DISPLAY_ROLE_PEDAL_SLOT:
    case MD400_DISPLAY_ROLE_PEDAL_LABEL_POS:
        return value ? ((value - 1) & 0x7f) : 0;
    case MD400_DISPLAY_ROLE_PEDAL_COLOR:
        return value % MD400_DISPLAY_PEDAL_COLOR_COUNT;
    case MD400_DISPLAY_ROLE_BT_TOGGLE:
        return value & 0x01;
    case MD400_DISPLAY_ROLE_PEDAL_MODE:
        return value & 0x01;
    case MD400_DISPLAY_ROLE_PEDAL_CAL:
        return 0;
    default:
        return value;
    }
}

static void md400_edit_sync_encoder_counters(int row0, int row1, int row2, int row3,
                                             u8 role0, u8 role1, u8 role2, u8 role3)
{
    md400_ms32_set_encoder_value(1, md400_edit_counter_value_from_role(role0, row0));
    md400_ms32_set_encoder_value(2, md400_edit_counter_value_from_role(role1, row1));
    md400_ms32_set_encoder_value(3, md400_edit_counter_value_from_role(role2, row2));
    md400_ms32_set_encoder_value(4, md400_edit_counter_value_from_role(role3, row3));
}

static const u8 md400_pedal_label_charset[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/_";

static u8 md400_pedal_label_char_selector(u8 ch)
{
    u8 i;

    for (i = 0; i < sizeof(md400_pedal_label_charset) - 1; i++) {
        if (md400_pedal_label_charset[i] == ch) {
            return i;
        }
    }
    return 0;
}

static void md400_pedal_display_sync_data(void)
{
    int rotate_msg[MD400_PEDAL_SLOT_COUNT];
    int press_msg[MD400_PEDAL_SLOT_COUNT];
    md400_assignment_t assignment;
    u8 color[MD400_PEDAL_SLOT_COUNT];
    u8 i;

    memset(color, 0, sizeof(color));
    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        for (i = 0; i < MD400_PEDAL_SLOT_COUNT; i++) {
            md400_pedal_action_t *action =
                md400_pedal_action_at(md400_pedal_active_preset,
                                      md400_pedal_page, i + 1);
            rotate_msg[i] = MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_OFF, 0);
            press_msg[i] = MD400_DISPLAY_MSG_PACK(MD400_MIDI_TYPE_OFF, 0);
            if (!action) {
                continue;
            }
            md400_pedal_action_assignment_get(action, 1, &assignment);
            rotate_msg[i] = md400_perform_display_msg(&assignment);
            md400_pedal_action_assignment_get(action, 0, &assignment);
            press_msg[i] = md400_perform_display_msg(&assignment);
            color[i] = i;
        }
        md400_display_perform_assignments_update(rotate_msg, press_msg);
        md400_display_foot_mode_update(1, md400_pedal_page, color);
    } else {
        md400_display_foot_mode_update(0, 0, NULL);
    }
}

static void md400_led_sync(void)
{
    extern u8 md400_board_power_is_softoff(void);

    if (md400_board_power_is_softoff()) {
        md400_led_policy_mode = MD400_LED_MODE_OFF;
        md400_led_policy_mask = 0;
        md400_led_set_mask(0);
        return;
    }

    if (md400_daw_active) {
        md400_led_policy_mode = md400_ui_mode;
        md400_led_policy_mask = md400_daw_led_mask;
        md400_led_set_mask(md400_daw_led_mask);
        return;
    }

    md400_led_policy_mode = md400_ui_mode;
    md400_led_policy_mask = md400_led_compute_mask();
    md400_led_set_mask(md400_led_policy_mask);
}

static void md400_led_pattern_tick(void *priv)
{
    (void)priv;
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_led_sync();
    }
}

static void md400_ui_display_sync(void)
{
    md400_assignment_t *assignment;
    md400_assignment_t pedal_assignment;
    int rotate_msg[MD400_ENCODER_COUNT];
    int press_msg[MD400_ENCODER_COUNT];
    int row0 = 0;
    int row1 = 0;
    int row2 = 0;
    int row3 = 0;
    u8 i;

    md400_led_sync();
    md400_pedal_display_sync_data();

    if (md400_daw_active) {
        /* The DAW view owns the screen; the normal UI repaints on leave. */
        return;
    }

    if (md400_ui_mode == MD400_UI_MODE_PERFORM) {
        if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
            md400_display_ui_update_roles(
                MD400_UI_MODE_PERFORM, 0, 0, 0, 0,
                MD400_DISPLAY_ROLE_ENCODER1,
                MD400_DISPLAY_ROLE_ENCODER2,
                MD400_DISPLAY_ROLE_ENCODER3,
                MD400_DISPLAY_ROLE_ENCODER4);
            return;
        }
        for (i = 0; i < MD400_ENCODER_COUNT; i++) {
            rotate_msg[i] = md400_perform_display_msg(md400_get_assignment(i + 1));
            press_msg[i] = md400_perform_display_msg(
                md400_get_gesture_assignment(MD400_EVENT_ENCODER_PRESS, i + 1));
        }
        md400_display_perform_assignments_update(rotate_msg, press_msg);
        md400_display_ui_update_roles(MD400_UI_MODE_PERFORM, 0, 0, 0, 0,
                                      MD400_DISPLAY_ROLE_ENCODER1,
                                      MD400_DISPLAY_ROLE_ENCODER2,
                                      MD400_DISPLAY_ROLE_ENCODER3,
                                      MD400_DISPLAY_ROLE_ENCODER4);
        return;
    }

    if (md400_ui_mode == MD400_UI_MODE_PLAY) {
        md400_play_refresh_display();
        md400_display_ui_update_roles(MD400_UI_MODE_PLAY, md400_play_last_msg,
                                      md400_play_last_value, 0, 0,
                                      MD400_DISPLAY_ROLE_PLAY_MESSAGE,
                                      MD400_DISPLAY_ROLE_PLAY_VALUE,
                                      MD400_DISPLAY_ROLE_PLAY_LABEL,
                                      MD400_DISPLAY_ROLE_NONE);
        return;
    }

    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        (md400_edit_subject != MD400_EDIT_SUBJECT_PEDAL)) {
        md400_pedal_action_t *pedal_action;

        md400_display_pedal_edit_update(md400_pedal_edit_slot,
                                        md400_pedal_edit_view);
        if (md400_edit_page_is_preset_select()) {
            row0 = md400_pedal_selected_preset + 1;
            row1 = md400_bt_user_enable_load();
            row2 = md400_pedal_mode_load();
            row3 = md400_pedal_cal_display_value();
            md400_display_ui_update_roles(
                MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_PRESET_SLOT,
                MD400_DISPLAY_ROLE_BT_TOGGLE,
                MD400_DISPLAY_ROLE_PEDAL_MODE,
                MD400_DISPLAY_ROLE_PEDAL_CAL);
            md400_edit_sync_encoder_counters(
                row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_PRESET_SLOT,
                MD400_DISPLAY_ROLE_BT_TOGGLE,
                MD400_DISPLAY_ROLE_PEDAL_MODE,
                MD400_DISPLAY_ROLE_PEDAL_CAL);
            return;
        }

        pedal_action = md400_pedal_action_at(md400_pedal_active_preset,
                                             md400_pedal_page,
                                             md400_pedal_edit_slot);
        if (!pedal_action) {
            return;
        }
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        md400_edit_normalize_subject_for_page();
        md400_pedal_action_assignment_get(
            pedal_action, md400_edit_page_is_rotate(), &pedal_assignment);
        row3 = MD400_DISPLAY_PAGE_SUBJECT_PACK(
            md400_pedal_edit_slot, md400_edit_display_page(),
            md400_edit_subject);
        if (md400_edit_page_is_message()) {
            row0 = md400_edit_selector_from_type(pedal_assignment.type);
            row1 = pedal_assignment.channel + 1;
            row2 = md400_edit_display_message_number(&pedal_assignment);
            md400_display_ui_update_roles(
                MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_TYPE,
                MD400_DISPLAY_ROLE_EDIT_CHANNEL,
                MD400_DISPLAY_ROLE_EDIT_DATA1,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
            md400_edit_sync_encoder_counters(
                row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_TYPE,
                MD400_DISPLAY_ROLE_EDIT_CHANNEL,
                MD400_DISPLAY_ROLE_EDIT_DATA1,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
        } else if (md400_edit_page_is_rotate()) {
            row0 = (pedal_assignment.behavior ==
                    MD400_BEHAVIOR_RELATIVE) ? 1 : 0;
            row1 = pedal_assignment.value_min;
            row2 = pedal_assignment.value_max;
            md400_display_ui_update_roles(
                MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                MD400_DISPLAY_ROLE_EDIT_MIN,
                MD400_DISPLAY_ROLE_EDIT_MAX,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
            md400_edit_sync_encoder_counters(
                row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                MD400_DISPLAY_ROLE_EDIT_MIN,
                MD400_DISPLAY_ROLE_EDIT_MAX,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
        } else {
            /* Pedal FT actions trigger only on release. There is one release
             * value, so do not expose the MIDI gesture BEH/PRS/RLS fields. */
            row0 = pedal_assignment.value_default;
            row1 = 0;
            row2 = 0;
            md400_display_ui_update_roles(
                MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_VALUE,
                MD400_DISPLAY_ROLE_NONE,
                MD400_DISPLAY_ROLE_NONE,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
            md400_edit_sync_encoder_counters(
                row0, row1, row2, row3,
                MD400_DISPLAY_ROLE_EDIT_VALUE,
                MD400_DISPLAY_ROLE_NONE,
                MD400_DISPLAY_ROLE_NONE,
                MD400_DISPLAY_ROLE_EDIT_PAGE);
        }
        return;
    }

    md400_edit_subject = md400_edit_subject_for_page(md400_edit_page);
    if (md400_edit_page_is_preset_select()) {
        if (md400_selected_preset >= MD400_PRESET_COUNT) {
            md400_selected_preset = md400_active_preset;
        }
        row0 = md400_selected_preset + 1;
        row1 = md400_bt_user_enable_load();
        row2 = md400_pedal_mode_load();
        row3 = md400_pedal_cal_display_value();
        md400_display_ui_update_roles(MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                                      MD400_DISPLAY_ROLE_PRESET_SLOT,
                                      MD400_DISPLAY_ROLE_BT_TOGGLE,
                                      MD400_DISPLAY_ROLE_PEDAL_MODE,
                                      MD400_DISPLAY_ROLE_PEDAL_CAL);
        md400_edit_sync_encoder_counters(row0, row1, row2, row3,
                                         MD400_DISPLAY_ROLE_PRESET_SLOT,
                                         MD400_DISPLAY_ROLE_BT_TOGGLE,
                                         MD400_DISPLAY_ROLE_PEDAL_MODE,
                                         MD400_DISPLAY_ROLE_PEDAL_CAL);
        return;
    }

    assignment = md400_edit_assignment();
    if (!assignment) {
        md400_display_ui_update_roles(MD400_UI_MODE_EDIT, 0, 0, 0,
                                      MD400_DISPLAY_PAGE_SUBJECT_PACK(
                                          md400_edit_target_encoder,
                                          md400_edit_display_page(),
                                          md400_edit_subject),
                                      MD400_DISPLAY_ROLE_NONE,
                                      MD400_DISPLAY_ROLE_NONE,
                                      MD400_DISPLAY_ROLE_NONE,
                                      MD400_DISPLAY_ROLE_EDIT_PAGE);
        md400_edit_sync_encoder_counters(
            0, 0, 0,
            MD400_DISPLAY_PAGE_SUBJECT_PACK(md400_edit_target_encoder,
                                            md400_edit_display_page(),
                                            md400_edit_subject),
            MD400_DISPLAY_ROLE_NONE,
            MD400_DISPLAY_ROLE_NONE,
            MD400_DISPLAY_ROLE_NONE,
            MD400_DISPLAY_ROLE_EDIT_PAGE);
        return;
    }

    row3 = MD400_DISPLAY_PAGE_SUBJECT_PACK(md400_edit_target_encoder,
                                           md400_edit_display_page(),
                                           md400_edit_subject);
    if (md400_edit_page_is_message()) {
        row0 = md400_edit_selector_from_type(assignment->type);
        row1 = assignment->channel + 1;
        row2 = md400_edit_display_message_number(assignment);
        md400_display_ui_update_roles(MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                                      MD400_DISPLAY_ROLE_EDIT_TYPE,
                                      MD400_DISPLAY_ROLE_EDIT_CHANNEL,
                                      MD400_DISPLAY_ROLE_EDIT_DATA1,
                                      MD400_DISPLAY_ROLE_EDIT_PAGE);
        md400_edit_sync_encoder_counters(row0, row1, row2, row3,
                                         MD400_DISPLAY_ROLE_EDIT_TYPE,
                                         MD400_DISPLAY_ROLE_EDIT_CHANNEL,
                                         MD400_DISPLAY_ROLE_EDIT_DATA1,
                                         MD400_DISPLAY_ROLE_EDIT_PAGE);
        return;
    } else if (md400_edit_page_is_rotate()) {
        row0 = (assignment->behavior == MD400_BEHAVIOR_RELATIVE) ? 1 : 0;
        row1 = assignment->value_min;
        row2 = assignment->value_max;
        md400_display_ui_update_roles(MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                                      MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                                      MD400_DISPLAY_ROLE_EDIT_MIN,
                                      MD400_DISPLAY_ROLE_EDIT_MAX,
                                      MD400_DISPLAY_ROLE_EDIT_PAGE);
        md400_edit_sync_encoder_counters(row0, row1, row2, row3,
                                         MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                                         MD400_DISPLAY_ROLE_EDIT_MIN,
                                         MD400_DISPLAY_ROLE_EDIT_MAX,
                                         MD400_DISPLAY_ROLE_EDIT_PAGE);
        return;
    } else {
        row0 = assignment->behavior;
        row1 = md400_edit_display_pack(1, assignment->value_max);
        row2 = md400_edit_display_pack(0, assignment->value_min);
        md400_display_ui_update_roles(MD400_UI_MODE_EDIT, row0, row1, row2, row3,
                                      MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                                      MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE,
                                      MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE,
                                      MD400_DISPLAY_ROLE_EDIT_PAGE);
        md400_edit_sync_encoder_counters(row0, row1, row2, row3,
                                         MD400_DISPLAY_ROLE_EDIT_BEHAVIOR,
                                         MD400_DISPLAY_ROLE_EDIT_DETAIL_VALUE,
                                         MD400_DISPLAY_ROLE_EDIT_DETAIL_RELEASE,
                                         MD400_DISPLAY_ROLE_EDIT_PAGE);
        return;
    }
}

static void md400_edit_apply_pedal_cal(u8 value, u8 event)
{
    if (event == MD400_EVENT_ENCODER_LEFT) {
        if (md400_pedal_cal_state != MD400_DISPLAY_PEDAL_CAL_HIDDEN) {
            md400_pedal_cal_hide();
        } else {
            md400_pedal_cal_reset();
        }
    } else if (event == MD400_EVENT_ENCODER_RIGHT) {
        md400_pedal_cal_confirm(md400_pedal_adc_value);
    } else if (value == 0) {
        md400_pedal_cal_hide();
    } else {
        md400_pedal_cal_confirm(md400_pedal_adc_value);
    }
    md400_log_event(0x03, 4, md400_pedal_cal_display_value(), value);
    printf("[MD400_UI] pedal cal state=%u min=%u max=%u raw=%u value=%u event=%u\r\n",
           (unsigned)md400_pedal_cal_state,
           (unsigned)md400_pedal_cal_min8,
           (unsigned)md400_pedal_cal_max8,
           (unsigned)md400_pedal_adc_value,
           (unsigned)md400_pedal_cal_scale(md400_pedal_adc_value),
           (unsigned)event);
}

static void md400_pedal_edit_apply_encoder(u8 editor_encoder, u8 value,
                                            u8 event)
{
    md400_pedal_action_t *action;
    md400_assignment_t assignment;
    u8 changed = 0;
    u8 rotate;
    u8 target;

    value = md400_clip7(value);
    if (md400_edit_page_is_preset_select()) {
        if (editor_encoder == 1) {
            target = md400_pedal_selected_preset;
            if (event == MD400_EVENT_ENCODER_LEFT) {
                target = target ? target - 1 : MD400_PRESET_COUNT - 1;
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                target = (target + 1) % MD400_PRESET_COUNT;
            } else {
                target = value % MD400_PRESET_COUNT;
            }
            md400_pedal_selected_preset = target;
            if (target != md400_pedal_active_preset) {
                md400_pedal_exp_cancel();
            }
            md400_pedal_active_preset = target;
            md400_pedal_bank_load(md400_pedal_active_preset,
                                  md400_pedal_page);
        } else if (editor_encoder == 2) {
            if (event == MD400_EVENT_ENCODER_LEFT) {
                md400_bt_set_user_enable(0);
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                md400_bt_set_user_enable(1);
            } else {
                md400_bt_set_user_enable(value & 0x01);
            }
            md400_log_event(0x03, editor_encoder,
                            md400_bt_user_enable, value);
            printf("[NEXFOOT_PEDAL] bt toggle=%u event=%u\r\n",
                   (unsigned)md400_bt_user_enable, (unsigned)event);
        } else if (editor_encoder == 3) {
            if (event == MD400_EVENT_ENCODER_LEFT) {
                md400_pedal_mode_set(MD400_PEDAL_MODE_EXP);
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                md400_pedal_mode_set(MD400_PEDAL_MODE_SWT);
            } else {
                md400_pedal_mode_set(value & 0x01);
            }
            md400_log_event(0x03, editor_encoder,
                            md400_pedal_mode, value);
            printf("[NEXFOOT_PEDAL] exp input=%s event=%u\r\n",
                   md400_pedal_mode ? "SWT" : "EXP", (unsigned)event);
        } else if (editor_encoder == 4) {
            md400_edit_apply_pedal_cal(value, event);
        }
        md400_ui_display_sync();
        md400_log_event(0x03, editor_encoder,
                        md400_pedal_edit_slot, value);
        return;
    }

    action = md400_pedal_action_at(md400_pedal_active_preset,
                                   md400_pedal_page,
                                   md400_pedal_edit_slot);
    if (!action) {
        return;
    }

    md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
    if (editor_encoder == 4) {
        md400_edit_select_from_event_or_value(value, event);
        md400_ui_display_sync();
        md400_log_event(0x03, editor_encoder,
                        md400_pedal_edit_slot, md400_edit_page);
        printf("[NEXFOOT_PEDAL] edit page preset=%u bank=%u ft=%u page=%u\r\n",
               (unsigned)(md400_pedal_active_preset + 1),
               (unsigned)(md400_pedal_page + 1),
               (unsigned)md400_pedal_edit_slot,
               (unsigned)md400_edit_display_page());
        return;
    }

    rotate = md400_edit_page_is_rotate();
    md400_pedal_action_assignment_get(action, rotate, &assignment);
    if (md400_edit_page_is_message()) {
        if (editor_encoder == 1) {
            assignment.type = md400_edit_type_from_selector(value);
        } else if (editor_encoder == 2) {
            assignment.channel = value & 0x0f;
        } else if (editor_encoder == 3) {
            md400_edit_set_message_number(&assignment, value);
            if (rotate &&
                ((assignment.type == MD400_MIDI_TYPE_PC) ||
                 (assignment.type == MD400_MIDI_TYPE_PITCH) ||
                 (assignment.type == MD400_MIDI_TYPE_PRESSURE))) {
                md400_pedal_rotate_value[md400_pedal_active_preset]
                                          [md400_pedal_page]
                                          [md400_pedal_edit_slot - 1] = value;
                md400_ms32_set_encoder_value(md400_pedal_edit_slot, value);
            }
        }
        changed = 1;
    } else if (rotate) {
        if (editor_encoder == 1) {
            assignment.behavior = (value & 0x01) ?
                                  MD400_BEHAVIOR_RELATIVE :
                                  MD400_BEHAVIOR_ABSOLUTE;
        } else if (editor_encoder == 2) {
            assignment.value_min = value;
        } else if (editor_encoder == 3) {
            assignment.value_max = value;
        }
        changed = 1;
    } else {
        if (editor_encoder == 1) {
            assignment.value_default = value;
            changed = 1;
        }
        assignment.behavior = MD400_BEHAVIOR_ABSOLUTE;
        assignment.value_min = 0;
        assignment.value_max = 0x7f;
    }

    if (changed) {
        assignment.target_mask = md400_available_target_mask();
        md400_pedal_action_assignment_set(action, rotate, &assignment);
    }
    md400_ui_display_sync();
    md400_log_event(0x03, editor_encoder,
                    md400_pedal_edit_slot, value);
    printf("[NEXFOOT_PEDAL] edit preset=%u bank=%u ft=%u page=%u action=%s editor=%u type=%u ch=%u num=%u min=%u max=%u mode=%u dirty=%u\r\n",
           (unsigned)(md400_pedal_active_preset + 1),
           (unsigned)(md400_pedal_page + 1),
           (unsigned)md400_pedal_edit_slot,
           (unsigned)md400_edit_display_page(),
           rotate ? "ROT" : "PRESS",
           (unsigned)editor_encoder,
           (unsigned)assignment.type,
           (unsigned)(assignment.channel + 1),
           (unsigned)md400_edit_display_message_number(&assignment),
           (unsigned)assignment.value_min,
           (unsigned)assignment.value_max,
           (unsigned)assignment.behavior,
           (unsigned)md400_pedal_record_dirty);
}

static void md400_edit_apply_encoder(u8 editor_encoder, u8 value, u8 event)
{
    md400_assignment_t *assignment;
    u8 target;

    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        (md400_edit_subject != MD400_EDIT_SUBJECT_PEDAL)) {
        md400_pedal_edit_apply_encoder(editor_encoder, value, event);
        return;
    }

    value = md400_clip7(value);
    if (md400_edit_page_is_preset_select()) {
        if (editor_encoder == 1) {
            target = (md400_selected_preset < MD400_PRESET_COUNT) ?
                     md400_selected_preset : md400_active_preset;
            if (event == MD400_EVENT_ENCODER_LEFT) {
                target = target ? (target - 1) : (MD400_PRESET_COUNT - 1);
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                target++;
                if (target >= MD400_PRESET_COUNT) {
                    target = 0;
                }
            } else {
                target = value % MD400_PRESET_COUNT;
            }
            md400_selected_preset = target;
            md400_ui_display_sync();
            md400_preset_load_schedule();
            md400_log_event(0x03, editor_encoder, md400_selected_preset + 1, value);
            printf("[MD400_UI] preset slot cursor active=%u selected=%u value=%u event=%u\r\n",
                   (unsigned)(md400_active_preset + 1),
                   (unsigned)(md400_selected_preset + 1),
                   (unsigned)value, (unsigned)event);
        } else if (editor_encoder == 2) {
            /* BT on/off: rotate right = on, rotate left = off */
            if (event == MD400_EVENT_ENCODER_LEFT) {
                md400_bt_set_user_enable(0);
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                md400_bt_set_user_enable(1);
            } else {
                md400_bt_set_user_enable(value & 1);
            }
            md400_ui_display_sync();
            md400_log_event(0x03, editor_encoder, md400_bt_user_enable, value);
            printf("[MD400_UI] bt toggle=%u event=%u\r\n",
                   (unsigned)md400_bt_user_enable, (unsigned)event);
        } else if (editor_encoder == 3) {
            /* Pedal mode: rotate left = expression, rotate right = switch */
            if (event == MD400_EVENT_ENCODER_LEFT) {
                md400_pedal_mode_set(MD400_PEDAL_MODE_EXP);
            } else if (event == MD400_EVENT_ENCODER_RIGHT) {
                md400_pedal_mode_set(MD400_PEDAL_MODE_SWT);
            } else {
                md400_pedal_mode_set(value & 0x01);
            }
            md400_ui_display_sync();
            md400_log_event(0x03, editor_encoder, md400_pedal_mode, value);
            printf("[MD400_UI] pedal mode=%u event=%u\r\n",
                   (unsigned)md400_pedal_mode, (unsigned)event);
        } else if (editor_encoder == 4) {
            md400_edit_apply_pedal_cal(value, event);
            md400_ui_display_sync();
        }
        return;
    }

    if (editor_encoder == 4) {
        md400_edit_select_from_event_or_value(value, event);
        md400_ui_display_sync();
        md400_log_event(0x03, editor_encoder, md400_edit_target_encoder, value);
        printf("[MD400_UI] edit target=%u page=%u selector=%u\r\n",
               (unsigned)md400_edit_target_encoder,
               (unsigned)md400_edit_page, (unsigned)value);
        return;
    }

    assignment = md400_edit_assignment();
    if (!assignment) {
        return;
    } else if (md400_edit_page_is_message()) {
        if (editor_encoder == 1) {
            assignment->type = md400_edit_type_from_selector(value);
        } else if (editor_encoder == 2) {
            assignment->channel = value & 0x0f;
        } else if (editor_encoder == 3) {
            md400_edit_set_message_number(assignment, value);
            if (md400_edit_page_is_rotate() &&
                ((assignment->type == MD400_MIDI_TYPE_PC) ||
                (assignment->type == MD400_MIDI_TYPE_PITCH) ||
                (assignment->type == MD400_MIDI_TYPE_PRESSURE))) {
                md400_encoder_runtime_value[md400_edit_target_encoder - 1] = value;
                md400_ms32_set_encoder_value(md400_edit_target_encoder, value);
            }
        }
    } else if (md400_edit_page_is_rotate()) {
        if (editor_encoder == 1) {
            assignment->behavior = (value & 0x01) ?
                                   MD400_BEHAVIOR_RELATIVE :
                                   MD400_BEHAVIOR_ABSOLUTE;
        } else if (editor_encoder == 2) {
            assignment->value_min = value;
        } else if (editor_encoder == 3) {
            assignment->value_max = value;
        }
    } else {
        if (editor_encoder == 1) {
            assignment->behavior = value % 4;
        } else if (editor_encoder == 2) {
            assignment->value_max = value;
            assignment->value_default = value;
        } else if (editor_encoder == 3) {
            assignment->value_min = value;
        }
    }

    md400_edit_normalize_subject_for_page();
    md400_store.checksum = md400_store_checksum(&md400_store);
    md400_edit_gesture_state_reset();
    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL)) {
        md400_pedal_exp_dirty = 1;
    } else {
        md400_edit_session_mark_dirty();
    }
    md400_ui_display_sync();
    md400_log_event(0x03, editor_encoder, md400_edit_target_encoder, value);
    printf("[MD400_UI] edit target=%u subject=%u page=%u editor=%u value=%u type=%u data1=%u def=%u ch=%u outputs=%02x\r\n",
           (unsigned)md400_edit_target_encoder, (unsigned)md400_edit_subject, (unsigned)md400_edit_page,
           (unsigned)editor_encoder, (unsigned)value,
           (unsigned)assignment->type, (unsigned)assignment->data1,
           (unsigned)assignment->value_default, (unsigned)(assignment->channel + 1),
           (unsigned)md400_available_target_mask());
}

static u8 md400_ui_handle_encoder_rotate(u8 encoder, u8 value, u8 event)
{
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_edit_apply_encoder(encoder, value, event);
        return 1;
    }

    return 0;
}

static const md400_sysex_template_t *md400_sysex_template_from_id(u8 template_id)
{
    if ((template_id < 1) || (template_id > MD400_SYSEX_TEMPLATE_COUNT)) {
        return NULL;
    }
    if ((md400_active_preset >= MD400_PRESET_COUNT) ||
        !md400_sysex_template_load_to_buf(md400_active_preset)) {
        return NULL;
    }
    if (!md400_sysex_template_buf->slot[template_id - 1].len) {
        return NULL;
    }
    return &md400_sysex_template_buf->slot[template_id - 1];
}

static u8 md400_send_assignment_custom_sysex_id(const md400_assignment_t *assignment,
                                               u8 template_id,
                                               u8 out_value)
{
    const md400_sysex_template_t *template;
    u8 len;

    if (!assignment) {
        return 0;
    }

    template = md400_sysex_template_from_id(template_id);
    if (!template) {
        md400_template_record_buf_release();
        return 0;
    }

    len = template->len;
    if (!len || (len > MD400_SYSEX_TEMPLATE_MAX_BODY)) {
        md400_template_record_buf_release();
        return 0;
    }

    md400_sysex_send_buf[0] = 0xF0;
    memcpy(&md400_sysex_send_buf[1], template->body, len);
    if (template->value_index < len) {
        md400_sysex_send_buf[1 + template->value_index] = out_value & 0x7f;
    }
    md400_sysex_send_buf[len + 1] = 0xF7;
    md400_template_record_buf_release();
    md400_send_all_sysex(md400_sysex_send_buf, len + 2);
    return 1;
}

static u8 md400_send_assignment_custom_sysex(const md400_assignment_t *assignment,
                                            u8 out_value)
{
    if (!assignment) {
        return 0;
    }
    return md400_send_assignment_custom_sysex_id(
        assignment,
        md400_assignment_sysex_template_id(assignment),
        out_value);
}

static const md400_custom_template_t *md400_custom_template_from_id(u8 template_id)
{
    if ((template_id < 1) || (template_id > MD400_CUSTOM_TEMPLATE_COUNT)) {
        return NULL;
    }
    if ((md400_active_preset >= MD400_PRESET_COUNT) ||
        !md400_custom_template_load_to_buf(md400_active_preset)) {
        return NULL;
    }
    if (!md400_custom_template_buf->slot[template_id - 1].len) {
        return NULL;
    }
    return &md400_custom_template_buf->slot[template_id - 1];
}

static u8 md400_midi_message_len(u8 status)
{
    if ((status >= 0x80) && (status <= 0xBF)) {
        return 3;
    }
    if ((status >= 0xC0) && (status <= 0xDF)) {
        return 2;
    }
    if ((status >= 0xE0) && (status <= 0xEF)) {
        return 3;
    }
    if ((status == 0xF1) || (status == 0xF3)) {
        return 2;
    }
    if (status == 0xF2) {
        return 3;
    }
    if ((status == 0xF6) || (status >= 0xF8)) {
        return 1;
    }
    return 0;
}

static u8 md400_custom_data_available(const u8 *raw, u8 len, u8 pos, u8 need)
{
    u8 i;

    if ((u16)pos + need > len) {
        return 0;
    }
    for (i = 0; i < need; i++) {
        if (raw[pos + i] >= 0x80) {
            return 0;
        }
    }
    return 1;
}

static int md400_send_custom_raw(const u8 *raw, u8 len)
{
    u8 pos = 0;
    u8 running_status = 0;
    u8 midi[3];
    int display_msg = MD400_PLAY_MSG_INVALID;

    while (pos < len) {
        u8 byte = raw[pos];
        u8 msg_len;

        if (byte == 0xF0) {
            u8 end = pos + 1;
            while ((end < len) && (raw[end] != 0xF7)) {
                end++;
            }
            if (end < len) {
                md400_send_all_sysex(&raw[pos], end - pos + 1);
                display_msg = md400_play_display_msg_sysex();
                pos = end + 1;
            } else {
                break;
            }
            running_status = 0;
            continue;
        }

        if (byte >= 0x80) {
            msg_len = md400_midi_message_len(byte);
            if (!msg_len) {
                pos++;
                running_status = 0;
                continue;
            }
            if (msg_len == 1) {
                midi[0] = byte;
                md400_send_all_short(midi, 1);
                display_msg = md400_play_display_msg_from_short(midi, 1);
                pos++;
                if (byte < 0xF0) {
                    running_status = byte;
                }
                continue;
            }
            if (md400_custom_data_available(raw, len, pos + 1, msg_len - 1)) {
                midi[0] = byte;
                midi[1] = raw[pos + 1] & 0x7f;
                midi[2] = (msg_len == 3) ? (raw[pos + 2] & 0x7f) : 0;
                md400_send_all_short(midi, msg_len);
                display_msg = md400_play_display_msg_from_short(midi, msg_len);
                pos += msg_len;
                running_status = (byte < 0xF0) ? byte : 0;
                continue;
            }
            pos++;
            running_status = 0;
            continue;
        }

        if (running_status) {
            msg_len = md400_midi_message_len(running_status);
            if ((msg_len > 1) &&
                md400_custom_data_available(raw, len, pos, msg_len - 1)) {
                midi[0] = running_status;
                midi[1] = raw[pos] & 0x7f;
                midi[2] = (msg_len == 3) ? (raw[pos + 1] & 0x7f) : 0;
                md400_send_all_short(midi, msg_len);
                display_msg = md400_play_display_msg_from_short(midi, msg_len);
                pos += msg_len - 1;
                continue;
            }
        }

        pos++;
    }
    return display_msg;
}

static int md400_send_assignment_custom_midi_id(const md400_assignment_t *assignment,
                                                u8 template_id,
                                                u8 out_value)
{
    const md400_custom_template_t *template;
    u8 len;

    if (!assignment) {
        return MD400_PLAY_MSG_INVALID;
    }

    template = md400_custom_template_from_id(template_id);
    if (!template) {
        md400_template_record_buf_release();
        return MD400_PLAY_MSG_INVALID;
    }

    len = template->len;
    if (!len || (len > MD400_CUSTOM_TEMPLATE_MAX_BODY)) {
        md400_template_record_buf_release();
        return MD400_PLAY_MSG_INVALID;
    }

    memcpy(md400_custom_send_buf, template->body, len);
    if (template->value_index < len) {
        md400_custom_send_buf[template->value_index] = out_value & 0x7f;
    }
    md400_template_record_buf_release();
    return md400_send_custom_raw(md400_custom_send_buf, len);
}

static int md400_send_assignment_custom_midi(const md400_assignment_t *assignment,
                                             u8 out_value)
{
    if (!assignment) {
        return MD400_PLAY_MSG_INVALID;
    }
    return md400_send_assignment_custom_midi_id(
        assignment,
        md400_assignment_custom_template_id(assignment),
        out_value);
}

static int md400_assignment_send_message(const md400_assignment_t *assignment, u8 control, u8 value)
{
    u8 midi[3];
    u8 out_value;
    int display_msg;

    if (!assignment) {
        return MD400_PLAY_MSG_INVALID;
    }

    value = md400_clip7(value);
    out_value = md400_scale_value(assignment, value);

    switch (assignment->type) {
    case MD400_MIDI_TYPE_CC:
        midi[0] = 0xB0 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PC:
        midi[0] = 0xC0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        md400_send_all_short(midi, 2);
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_NOTE:
        midi[0] = 0x90 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_NOTE_OFF:
        midi[0] = 0x80 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PITCH: {
        u16 bend = ((u16)out_value) << 7;
        midi[0] = 0xE0 | (assignment->channel & 0x0f);
        midi[1] = bend & 0x7f;
        midi[2] = (bend >> 7) & 0x7f;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    }
    case MD400_MIDI_TYPE_PRESSURE:
        midi[0] = 0xD0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        md400_send_all_short(midi, 2);
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_SYSEX: {
        u8 sysex[12];
        if (md400_send_assignment_custom_sysex(assignment, out_value)) {
            return md400_play_display_msg_sysex();
        }
        sysex[0] = 0xF0;
        sysex[1] = MD400_SYSEX_DEV_ID;
        sysex[2] = 'M';
        sysex[3] = 'D';
        sysex[4] = '4';
        sysex[5] = '0';
        sysex[6] = MD400_SYSEX_VERSION;
        sysex[7] = md400_assignment_sysex_template_id(assignment);
        sysex[8] = control & 0x7f;
        sysex[9] = assignment->data1 & 0x7f;
        sysex[10] = out_value & 0x7f;
        sysex[11] = 0xF7;
        md400_send_all_sysex(sysex, sizeof(sysex));
        return md400_play_display_msg_sysex();
    }
    case MD400_MIDI_TYPE_CUSTOM:
        display_msg = md400_send_assignment_custom_midi(assignment, out_value);
        if (display_msg != MD400_PLAY_MSG_INVALID) {
            return display_msg;
        }
        return md400_play_display_msg_from_output(assignment, out_value, 0);
    default:
        break;
    }
    return MD400_PLAY_MSG_INVALID;
}

static int md400_assignment_send_output_value(const md400_assignment_t *assignment,
                                              u8 control, u8 out_value)
{
    u8 midi[3];
    int display_msg;

    if (!assignment) {
        return MD400_PLAY_MSG_INVALID;
    }

    out_value = md400_clip7(out_value);

    switch (assignment->type) {
    case MD400_MIDI_TYPE_CC:
        midi[0] = 0xB0 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PC:
        midi[0] = 0xC0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        md400_send_all_short(midi, 2);
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_NOTE:
        midi[0] = 0x90 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_NOTE_OFF:
        midi[0] = 0x80 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    case MD400_MIDI_TYPE_PITCH: {
        u16 bend = ((u16)out_value) << 7;
        midi[0] = 0xE0 | (assignment->channel & 0x0f);
        midi[1] = bend & 0x7f;
        midi[2] = (bend >> 7) & 0x7f;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    }
    case MD400_MIDI_TYPE_PRESSURE:
        midi[0] = 0xD0 | (assignment->channel & 0x0f);
        midi[1] = out_value;
        md400_send_all_short(midi, 2);
        return md400_play_display_msg_from_short(midi, 2);
    case MD400_MIDI_TYPE_SYSEX: {
        u8 sysex[12];
        if (md400_send_assignment_custom_sysex(assignment, out_value)) {
            return md400_play_display_msg_sysex();
        }
        sysex[0] = 0xF0;
        sysex[1] = MD400_SYSEX_DEV_ID;
        sysex[2] = 'M';
        sysex[3] = 'D';
        sysex[4] = '4';
        sysex[5] = '0';
        sysex[6] = MD400_SYSEX_VERSION;
        sysex[7] = md400_assignment_sysex_template_id(assignment);
        sysex[8] = control & 0x7f;
        sysex[9] = assignment->data1 & 0x7f;
        sysex[10] = out_value & 0x7f;
        sysex[11] = 0xF7;
        md400_send_all_sysex(sysex, sizeof(sysex));
        return md400_play_display_msg_sysex();
    }
    case MD400_MIDI_TYPE_CUSTOM:
        display_msg = md400_send_assignment_custom_midi(assignment, out_value);
        if (display_msg != MD400_PLAY_MSG_INVALID) {
            return display_msg;
        }
        return md400_play_display_msg_from_output(assignment, out_value, 0);
    default:
        break;
    }
    return MD400_PLAY_MSG_INVALID;
}

static u8 md400_relative_dir_value(const md400_assignment_t *assignment, u8 direction)
{
    if (!assignment) {
        return 0;
    }
    if (direction == MD400_EVENT_ENCODER_RIGHT) {
        return assignment->value_max;
    }
    return assignment->value_min;
}

static u8 md400_assignment_range_direction(const md400_assignment_t *assignment, u8 direction)
{
    if (!assignment || (assignment->value_min <= assignment->value_max)) {
        return direction;
    }
    if (direction == MD400_EVENT_ENCODER_LEFT) {
        return MD400_EVENT_ENCODER_RIGHT;
    }
    if (direction == MD400_EVENT_ENCODER_RIGHT) {
        return MD400_EVENT_ENCODER_LEFT;
    }
    return direction;
}

static int md400_assignment_send_gate(const md400_assignment_t *assignment, u8 control, u8 active)
{
    u8 midi[3];
    u8 out_value;
    int display_msg;

    if (!assignment || (assignment->type == MD400_MIDI_TYPE_OFF)) {
        return MD400_PLAY_MSG_INVALID;
    }

    out_value = active ? assignment->value_max : assignment->value_min;

    if (assignment->type == MD400_MIDI_TYPE_SYSEX) {
        if (md400_send_assignment_custom_sysex_id(
                assignment,
                md400_assignment_sysex_gate_template_id(assignment, active),
                out_value)) {
            return md400_play_display_msg_sysex();
        }
        return md400_assignment_send_output_value(assignment, control, out_value);
    }

    if (assignment->type == MD400_MIDI_TYPE_CUSTOM) {
        display_msg = md400_send_assignment_custom_midi_id(
            assignment,
            md400_assignment_custom_gate_template_id(assignment, active),
            out_value);
        if (display_msg != MD400_PLAY_MSG_INVALID) {
            return display_msg;
        }
        return md400_assignment_send_output_value(assignment, control, out_value);
    }

    if (active) {
        return md400_assignment_send_output_value(assignment, control, out_value);
    }

    if (assignment->type == MD400_MIDI_TYPE_NOTE) {
        midi[0] = 0x80 | (assignment->channel & 0x0f);
        midi[1] = assignment->data1 & 0x7f;
        midi[2] = out_value & 0x7f;
        md400_send_all_short(midi, 3);
        return md400_play_display_msg_from_short(midi, 3);
    }

    return md400_assignment_send_output_value(assignment, control, out_value);
}

static u8 md400_encoder_direction_event(u8 event, u8 old_value, u8 value)
{
    if ((event == MD400_EVENT_ENCODER_LEFT) ||
        (event == MD400_EVENT_ENCODER_RIGHT)) {
        return event;
    }
    return (value >= old_value) ? MD400_EVENT_ENCODER_RIGHT : MD400_EVENT_ENCODER_LEFT;
}

static void md400_assignment_send(u8 encoder, u8 value, u8 event)
{
    md400_assignment_t *assignment = md400_get_assignment(encoder);
    u8 old_value;
    u8 direction;
    u8 out_value;
    int display_msg;

    if (!assignment) {
        return;
    }

    value = md400_clip7(value);
    old_value = md400_encoder_runtime_value[encoder - 1];
    md400_encoder_runtime_value[encoder - 1] = value;
    direction = md400_encoder_direction_event(event, old_value, value);
    direction = md400_assignment_range_direction(assignment, direction);

    if (assignment->behavior == MD400_BEHAVIOR_RELATIVE) {
        out_value = md400_relative_dir_value(assignment, direction);
        display_msg = md400_assignment_send_output_value(assignment, encoder,
                                                         out_value);
    } else {
        out_value = (assignment->type == MD400_MIDI_TYPE_OFF) ?
                    0 : md400_scale_value(assignment, value);
        display_msg = md400_assignment_send_message(assignment, encoder, value);
    }

    md400_play_show_assignment_sent(assignment, MD400_PLAY_LABEL_ROTATE_ACTION,
                                    encoder, display_msg, out_value);
    md400_log_event(0x01, direction, encoder, value);
}

static void md400_pedal_rotate_send(u8 encoder, u8 value, u8 event)
{
    md400_pedal_action_t *action;
    md400_assignment_t assignment;
    u8 *runtime_value;
    u8 old_value;
    u8 direction;
    u8 out_value;
    int display_msg;

    if ((encoder < 1) || (encoder > MD400_PEDAL_SLOT_COUNT)) {
        return;
    }

    value = md400_clip7(value);
    runtime_value = &md400_pedal_rotate_value[md400_pedal_active_preset]
                                                [md400_pedal_page]
                                                [encoder - 1];
    old_value = *runtime_value;
    *runtime_value = value;
    direction = md400_encoder_direction_event(event, old_value, value);

    action = md400_pedal_action_at(md400_pedal_active_preset,
                                   md400_pedal_page, encoder);
    if (!action) {
        return;
    }
    md400_pedal_action_assignment_get(action, 1, &assignment);
    direction = md400_assignment_range_direction(&assignment, direction);
    if (assignment.behavior == MD400_BEHAVIOR_RELATIVE) {
        out_value = md400_relative_dir_value(&assignment, direction);
        display_msg = md400_assignment_send_output_value(&assignment, encoder,
                                                         out_value);
    } else {
        out_value = (assignment.type == MD400_MIDI_TYPE_OFF) ?
                    0 : md400_scale_value(&assignment, value);
        display_msg = md400_assignment_send_message(&assignment, encoder,
                                                     value);
    }
    md400_play_show_assignment_sent(&assignment,
                                    MD400_PLAY_LABEL_ROTATE_ACTION,
                                    encoder, display_msg, out_value);
    md400_log_event(0x01, direction, encoder, value);
}

static u8 md400_delta_u8(u8 a, u8 b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static u8 md400_pedal_ui_value_changed(u8 old_value, u8 value)
{
    old_value = md400_clip7(old_value);
    value = md400_clip7(value);
    if (old_value == value) {
        return 0;
    }
    if ((value == 0) || (value == 0x7f)) {
        return 1;
    }
    return md400_delta_u8(old_value, value) >= MD400_PEDAL_UI_VALUE_DEADBAND;
}

static u8 md400_pedal_display_should_update(u8 out_value)
{
    out_value = md400_clip7(out_value);
    if (!md400_pedal_display_seen) {
        md400_pedal_display_seen = 1;
        md400_pedal_display_value = out_value;
        return 1;
    }
    if (!md400_pedal_ui_value_changed(md400_pedal_display_value, out_value)) {
        return 0;
    }
    md400_pedal_display_value = out_value;
    return 1;
}

static u8 md400_pedal_settings_log_changed(u8 value)
{
    if (!md400_pedal_settings_seen) {
        return 1;
    }
    if (md400_pedal_ui_value_changed(md400_pedal_settings_last_value, value)) {
        return 1;
    }
    if (md400_delta_u8(md400_pedal_settings_last_raw,
                       md400_pedal_adc_value) >= MD400_PEDAL_UI_RAW_DEADBAND) {
        return 1;
    }
    return md400_pedal_settings_last_cal != md400_pedal_cal_state;
}

static u8 md400_pedal_edit_auto_allowed(void)
{
    return (md400_ui_mode == MD400_UI_MODE_EDIT) &&
           !md400_pedal_cal_active &&
           !md400_ms32_mode_sync_active &&
           (md400_edit_subject != MD400_EDIT_SUBJECT_PEDAL);
}

static void md400_pedal_settings_enter_cancel(void)
{
    if (md400_pedal_settings_enter_timer) {
        sys_timeout_del(md400_pedal_settings_enter_timer);
        md400_pedal_settings_enter_timer = 0;
    }
}

static void md400_pedal_settings_enter_reset(void)
{
    md400_pedal_settings_enter_cancel();
    md400_pedal_settings_enter_channel = 0;
    md400_pedal_settings_enter_value = 0;
    md400_pedal_settings_enter_ready = 0;
    md400_pedal_settings_enter_mode = md400_pedal_mode_load();
    md400_pedal_settings_max_latched = 0;
}

static void md400_pedal_settings_enter_page(u8 channel, u8 value)
{
    if (!md400_pedal_edit_auto_allowed() ||
        md400_pedal_settings_max_latched) {
        return;
    }

    md400_pedal_settings_enter_cancel();
    md400_pedal_settings_max_latched = 1;
    md400_preset_load_timer_cancel();
    md400_pedal_cal_hide();
    md400_edit_subject_before_pedal = md400_edit_subject;
    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        !md400_pedal_exp_dirty) {
        md400_pedal_exp_snapshot = *md400_get_pedal_edit_assignment();
        md400_pedal_exp_snapshot_preset = md400_pedal_active_preset;
    }
    md400_edit_subject = MD400_EDIT_SUBJECT_PEDAL;
    md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
    md400_ui_display_sync();
    md400_log_event(0x03, MD400_EVENT_PEDAL_VALUE,
                    channel & 0x7f, value);
    printf("[MD400_UI] edit pedal from max hold channel=%u value=%u raw=%u mode=%u\r\n",
           (unsigned)channel, (unsigned)value,
           (unsigned)md400_pedal_adc_value,
           (unsigned)md400_pedal_mode_load());
}

static void md400_pedal_settings_enter_timeout(void *priv)
{
    (void)priv;
    md400_pedal_settings_enter_timer = 0;
    if (md400_pedal_settings_enter_ready &&
        (md400_pedal_settings_enter_mode == md400_pedal_mode_load()) &&
        (md400_pedal_settings_enter_value >= MD400_PEDAL_EDIT_RELEASE_TH)) {
        md400_pedal_settings_enter_page(md400_pedal_settings_enter_channel,
                                        md400_pedal_settings_enter_value);
    }
}

static void md400_pedal_settings_enter_arm(u8 channel, u8 value)
{
    u32 hold_ms;

    if (!md400_pedal_edit_auto_allowed() ||
        !md400_pedal_settings_enter_ready ||
        md400_pedal_settings_max_latched) {
        return;
    }

    md400_pedal_settings_enter_channel = channel & 0x7f;
    md400_pedal_settings_enter_value = md400_clip7(value);
    md400_pedal_settings_enter_mode = md400_pedal_mode_load();
    if (!md400_pedal_settings_enter_timer) {
        /* Both modes use a short deliberate hold. SWT entry is based on the
         * physical ADC maximum, not its inverted press/release MIDI value. */
        hold_ms = (md400_pedal_mode_load() == MD400_PEDAL_MODE_SWT) ?
                  MD400_PEDAL_EDIT_SWT_ENTER_HOLD_MS :
                  MD400_PEDAL_EDIT_EXP_ENTER_HOLD_MS;
        md400_pedal_settings_enter_timer =
            sys_timeout_add(NULL, md400_pedal_settings_enter_timeout,
                            hold_ms);
    }
}

static u8 md400_ui_handle_pedal_edit(u8 channel, u8 value, u8 trigger_value)
{
    md400_assignment_t *assignment;

    trigger_value = md400_clip7(trigger_value);
    if (md400_ui_mode != MD400_UI_MODE_EDIT) {
        return 0;
    }

    if (md400_edit_page_is_preset_select()) {
        if (md400_pedal_edit_auto_allowed()) {
            if (trigger_value < MD400_PEDAL_EDIT_RELEASE_TH) {
                md400_pedal_settings_max_latched = 0;
                md400_pedal_settings_enter_cancel();
                md400_pedal_settings_enter_ready = 1;
            } else if (trigger_value >= MD400_PEDAL_EDIT_ENTER_TH) {
                md400_pedal_settings_enter_arm(channel, trigger_value);
            }
        }
        if (md400_pedal_settings_log_changed(value)) {
            md400_pedal_settings_seen = 1;
            md400_pedal_settings_last_value = value;
            md400_pedal_settings_last_raw = md400_pedal_adc_value;
            md400_pedal_settings_last_cal = md400_pedal_cal_state;
            md400_log_event(0x03, MD400_EVENT_PEDAL_VALUE,
                            channel & 0x7f, value);
            printf("[MD400_UI] pedal settings input channel=%u value=%u raw=%u cal=%u mode=%u\r\n",
                   (unsigned)channel, (unsigned)value,
                   (unsigned)md400_pedal_adc_value,
                   (unsigned)md400_pedal_cal_active,
                   (unsigned)md400_pedal_mode_load());
        }
        return 1;
    }

    if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
        md400_pedal_settings_enter_cancel();
        if (!md400_edit_page_is_message()) {
            assignment = md400_get_pedal_edit_assignment();
            if (assignment &&
                md400_pedal_ui_value_changed(assignment->value_max, value)) {
                assignment->value_max = value;
                assignment->value_default = value;
                md400_store.checksum = md400_store_checksum(&md400_store);
                if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
                    md400_pedal_exp_dirty = 1;
                } else {
                    md400_edit_session_mark_dirty();
                }
                md400_ui_display_sync();
                md400_log_event(0x03, MD400_EVENT_PEDAL_VALUE,
                                channel & 0x7f, value);
                printf("[MD400_UI] edit pedal value=%u mode=%u\r\n",
                       (unsigned)value, (unsigned)md400_pedal_mode_load());
            }
        }
        return 1;
    }

    if (md400_pedal_edit_auto_allowed()) {
        if (trigger_value < MD400_PEDAL_EDIT_RELEASE_TH) {
            md400_pedal_settings_max_latched = 0;
            md400_pedal_settings_enter_cancel();
            md400_pedal_settings_enter_ready = 1;
        } else if (trigger_value >= MD400_PEDAL_EDIT_ENTER_TH) {
            md400_pedal_settings_enter_arm(channel, trigger_value);
        }
    }
    return 1;
}

static void md400_pedal_send_value(u8 channel, u8 value, u8 trigger_value)
{
    md400_assignment_t *assignment;
    static u32 log_count;
    int display_msg;
    u8 out_value;

    if (md400_daw_active) {
        (void)trigger_value;
        md400_daw_pedal_value(value);
        return;
    }

    value = md400_clip7(value);
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_pedal_seen = 1;
        md400_pedal_runtime_value = value;
        md400_log_event(0x05, MD400_EVENT_PEDAL_VALUE, channel & 0x7f, value);
        if (md400_ui_handle_pedal_edit(channel, value, trigger_value)) {
            return;
        }
    }
    if (md400_pedal_seen && (value == md400_pedal_runtime_value)) {
        return;
    }

    md400_pedal_seen = 1;
    md400_pedal_runtime_value = value;
    md400_log_event(0x05, MD400_EVENT_PEDAL_VALUE, channel & 0x7f, value);

    if (!md400_ui_performance_active()) {
        return;
    }

    /* The EXP jack is available in both foot modes. MIDI mode uses the active
     * MIDI preset's EXP assignment; PEDAL mode uses the active PEDAL preset's
     * shared EXP assignment. Do not confuse this with any of the four FTs. */
    assignment = md400_get_pedal_edit_assignment();
    if (!assignment || (assignment->type == MD400_MIDI_TYPE_OFF)) {
        return;
    }

    out_value = md400_scale_value(assignment, value);
    display_msg = md400_assignment_send_message(assignment, 5, value);
    if (md400_pedal_display_should_update(out_value)) {
        md400_play_show_assignment_sent(assignment, MD400_EVENT_PEDAL_VALUE, 5,
                                        display_msg, out_value);
    }
    if (log_count < 32) {
        printf("[MD400_PEDAL] adc_ch=%u value=%u type=%u ch=%u data1=%u outputs=%02x\r\n",
               (unsigned)channel, (unsigned)value,
               (unsigned)assignment->type, (unsigned)(assignment->channel + 1),
               (unsigned)assignment->data1,
               (unsigned)md400_available_target_mask());
    } else if (log_count == 32) {
        printf("[MD400_PEDAL] log muted\r\n");
    }
    log_count++;
}

static u8 md400_pedal_switch_from_value(u8 value)
{
    value = md400_clip7(value);

    /* Passive switch pedals on the expression jack pull ADC4 low when pressed. */
    if (!md400_pedal_switch_seen) {
        return (value <= 64) ? 1 : 0;
    }
    if (md400_pedal_switch_state) {
        return (value <= MD400_PEDAL_SWITCH_ON_TH) ? 1 : 0;
    }
    return (value <= MD400_PEDAL_SWITCH_OFF_TH) ? 1 : 0;
}

static void md400_pedal_input_value(u8 channel, u8 value)
{
    u8 active;

    value = md400_clip7(value);
    if (md400_pedal_mode_load() == MD400_PEDAL_MODE_SWT) {
        active = md400_pedal_switch_from_value(value);
        md400_pedal_switch_seen = 1;
        md400_pedal_switch_state = active;
        /* Keep switch MIDI semantics (pressed low -> 127), but use the
         * non-inverted physical value for maximum-position EXP Edit entry. */
        md400_pedal_send_value(channel, active ? 0x7f : 0x00, value);
        return;
    }

    md400_pedal_switch_seen = 0;
    md400_pedal_switch_state = 0;

    /*
     * An open expression jack rests near the ADC top rail. A two-count raw
     * wobble there becomes alternating MIDI values 126/127. Snap both endpoint
     * margins to the endpoint, then compare against the last accepted value so
     * slow real movement accumulates instead of producing idle MIDI jitter.
     */
    if (value <= MD400_PEDAL_MIDI_ENDPOINT_MARGIN) {
        value = 0;
    } else if (value >= (0x7f - MD400_PEDAL_MIDI_ENDPOINT_MARGIN)) {
        value = 0x7f;
    }
    if (md400_pedal_seen &&
        (md400_delta_u8(md400_pedal_runtime_value, value) <
         MD400_PEDAL_MIDI_VALUE_DEADBAND)) {
        return;
    }
    md400_pedal_send_value(channel, value, value);
}

static void md400_pedal_adc_event(u8 channel, u8 value)
{
    md400_pedal_adc_value = value;
    md400_pedal_cal_update(value);
    if (md400_pedal_mode_load() == MD400_PEDAL_MODE_SWT) {
        /*
         * MS32 switch mode reports the physical jack state as raw 0/255.
         * Classify that raw state directly. Expression calibration (especially
         * a reversed range) must never turn an open/unplugged switch into an
         * active press.
         */
        md400_pedal_input_value(channel, (value >> 1) & 0x7f);
    } else {
        md400_pedal_input_value(channel, md400_pedal_cal_scale(value));
    }
}

void md400_midi_send_encoder_cc(u8 encoder, u8 value, u8 event)
{
    static u32 log_count;
    md400_pedal_action_t *pedal_action;
    md400_assignment_t pedal_assignment;
    md400_assignment_t *assignment;
    u8 pedal_rotate;

    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return;
    }

    if (md400_daw_active) {
        md400_daw_rotate(encoder, value, event);
        return;
    }

    if (md400_ui_handle_encoder_rotate(encoder, value, event)) {
        return;
    }

    pedal_rotate = (md400_foot_mode == MD400_FOOT_MODE_PEDAL);
    if (pedal_rotate) {
        pedal_action = md400_pedal_action_at(md400_pedal_active_preset,
                                             md400_pedal_page, encoder);
        if (pedal_action) {
            md400_pedal_action_assignment_get(pedal_action, 1,
                                              &pedal_assignment);
            assignment = &pedal_assignment;
        } else {
            assignment = NULL;
        }
        md400_pedal_rotate_send(encoder, value, event);
    } else {
        assignment = md400_get_assignment(encoder);
        md400_assignment_send(encoder, value, event);
    }

    if (log_count < 32) {
        printf("[MD400_MIDI] enc=%u type=%u ch=%u data1=%u value=%u outputs=%02x\r\n",
               (unsigned)encoder,
               (unsigned)(assignment ? assignment->type : 0xff),
               (unsigned)(assignment ? assignment->channel + 1 : 0),
               (unsigned)(assignment ? assignment->data1 : 0),
               (unsigned)md400_clip7(value),
               (unsigned)md400_available_target_mask());
    } else if (log_count == 32) {
        printf("[MD400_MIDI] assignment log muted\r\n");
    }
    log_count++;
}

void md400_midi_send_encoder_press(u8 encoder, u8 pressed, u8 value)
{
    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return;
    }

    pressed = !!pressed;
    md400_encoder_button_gesture(encoder, pressed, value);
    md400_led_sync();
}

static void md400_pedal_page_step(int delta)
{
    int page = md400_pedal_page + delta;

    while (page < 0) {
        page += MD400_PEDAL_PAGE_COUNT;
    }
    while (page >= MD400_PEDAL_PAGE_COUNT) {
        page -= MD400_PEDAL_PAGE_COUNT;
    }
    if ((u8)page == md400_pedal_page) {
        return;
    }
    if (!md400_pedal_bank_load(md400_pedal_active_preset, (u8)page)) {
        printf("[NEXFOOT_PEDAL] page step blocked preset=%u page=%u dirty=%u\r\n",
               (unsigned)(md400_pedal_active_preset + 1),
               (unsigned)(page + 1),
               (unsigned)md400_pedal_record_dirty);
        return;
    }
    md400_pedal_page = page;
    if (md400_ui_mode != MD400_UI_MODE_EDIT) {
        md400_sync_ms32_encoder_runtime_values();
    }
    md400_ui_display_sync();
    md400_log_event(0x06, MD400_EVENT_COMBO_SHORT, 0,
                    md400_pedal_page + 1);
    printf("[NEXFOOT_PEDAL] page=%u preset=%u\r\n",
           (unsigned)(md400_pedal_page + 1),
           (unsigned)(md400_pedal_active_preset + 1));
}

static void md400_foot_mode_set_now(u8 mode)
{
    mode = mode ? MD400_FOOT_MODE_PEDAL : MD400_FOOT_MODE_MIDI;
    if (mode == md400_foot_mode) {
        return;
    }

    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_pedal_exp_cancel();
        md400_edit_session_end(0);
    }
    /* Changing foot mode is navigation, never an implicit preset save. */
    if (md400_pedal_names_dirty) md400_pedal_names_load();
    if (md400_pedal_record_dirty) {
        md400_pedal_record_dirty = 0;
        md400_pedal_record_loaded = 0;
    }
    md400_preset_load_timer_cancel();
    md400_pedal_cal_hide();
    md400_pedal_settings_enter_reset();
    md400_button_down_mask = 0;
    md400_combo_mask = 0;
    memset(md400_button_gesture, 0, sizeof(md400_button_gesture));
    md400_runtime_action_state_clear();
    md400_foot_mode = mode;
    md400_ui_mode = MD400_UI_MODE_PERFORM;
    md400_edit_page = MD400_EDIT_PAGE_PRESET;
    md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
    md400_pedal_edit_slot = 1;
    md400_pedal_label_position = 0;
    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        md400_pedal_selected_preset = md400_pedal_active_preset;
        md400_pedal_bank_load(md400_pedal_active_preset,
                              md400_pedal_page);
    }
    md400_sync_ms32_encoder_runtime_values();
    md400_pedal_header_save();
    md400_ui_display_sync();
    printf("[NEXFOOT_UI] mode=%s midi_preset=%u pedal_preset=%u page=%u\r\n",
           md400_foot_mode == MD400_FOOT_MODE_PEDAL ? "PEDAL" : "MIDI",
           (unsigned)(md400_active_preset + 1),
           (unsigned)(md400_pedal_active_preset + 1),
           (unsigned)(md400_pedal_page + 1));
}

static void md400_foot_mode_set(u8 mode)
{
    mode = mode ? MD400_FOOT_MODE_PEDAL : MD400_FOOT_MODE_MIDI;
    if (md400_persist_in_task) {
        md400_foot_mode_set_now(mode);
        return;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_FOOT_MODE, mode, 0)) {
        printf("[MD400_PERSIST] foot mode queue failed value=%u\r\n",
               (unsigned)mode);
    }
}

static void md400_pedal_release_action(u8 slot)
{
    md400_pedal_action_t *action;
    md400_assignment_t assignment;
    int display_msg;

    if ((slot < 1) || (slot > MD400_PEDAL_SLOT_COUNT)) {
        return;
    }

    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_pedal_edit_slot = slot;
        if (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL) {
            md400_pedal_cal_hide();
            md400_edit_subject = md400_edit_subject_before_pedal;
            if ((md400_edit_subject >= MD400_EDIT_SUBJECT_COMBO_PRESS) ||
                (md400_edit_subject == MD400_EDIT_SUBJECT_PEDAL)) {
                md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
            }
            md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
            md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
        }
        md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
        if (md400_edit_page_is_preset_select()) {
            md400_pedal_active_preset = md400_pedal_selected_preset;
            md400_pedal_bank_load(md400_pedal_active_preset,
                                  md400_pedal_page);
            md400_edit_page = MD400_EDIT_PAGE_MESSAGE;
            md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
        }
        md400_ui_display_sync();
        printf("[NEXFOOT_PEDAL] edit select page=%u ft=%u\r\n",
               (unsigned)(md400_pedal_page + 1), (unsigned)slot);
        return;
    }

    if (!md400_ui_performance_active()) {
        return;
    }
    action = md400_pedal_action_at(md400_pedal_active_preset,
                                   md400_pedal_page, slot);
    if (!action) {
        return;
    }
    md400_pedal_action_assignment_get(action, 0, &assignment);
    if (assignment.type == MD400_MIDI_TYPE_OFF) {
        return;
    }

    display_msg = md400_assignment_send_output_value(
        &assignment, slot, assignment.value_default);
    md400_play_show_assignment_sent(
        &assignment, MD400_EVENT_ENCODER_RELEASE, slot,
        display_msg, assignment.value_default);
    md400_log_event(0x06, MD400_EVENT_ENCODER_RELEASE, slot,
                    assignment.value_default);
    printf("[NEXFOOT_PEDAL] release preset=%u page=%u ft=%u type=%u ch=%u num=%u value=%u\r\n",
           (unsigned)(md400_pedal_active_preset + 1),
           (unsigned)(md400_pedal_page + 1),
           (unsigned)slot,
           (unsigned)assignment.type,
           (unsigned)(assignment.channel + 1),
           (unsigned)assignment.data1,
           (unsigned)assignment.value_default);
}

static void md400_ui_event(u8 event)
{
    if (event == MD400_EVENT_EDIT_LONG) {
        u8 mode_before = md400_ui_mode;

        if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
            if (md400_ui_mode == MD400_UI_MODE_EDIT) {
                if (md400_pedal_names_dirty) md400_pedal_names_load();
                /* Exit discards edits since the last explicit Save. Reload
                 * the pedal cache so unsaved actions cannot be saved later
                 * by a mode change or leave page-group navigation blocked. */
                md400_pedal_record_dirty = 0;
                md400_pedal_record_loaded = 0;
                md400_pedal_bank_load(md400_pedal_active_preset,
                                      md400_pedal_page);
                md400_pedal_exp_cancel();
                md400_pedal_cal_hide();
                md400_pedal_settings_enter_cancel();
                md400_ui_mode = MD400_UI_MODE_PERFORM;
                md400_sync_ms32_encoder_runtime_values();
            } else {
                md400_pedal_cal_hide();
                md400_pedal_settings_enter_reset();
                md400_pedal_settings_seen = 0;
                md400_pedal_exp_dirty = 0;
                md400_pedal_exp_snapshot = *md400_get_pedal_edit_assignment();
                md400_pedal_exp_snapshot_preset = md400_pedal_active_preset;
                md400_ui_mode = MD400_UI_MODE_EDIT;
                md400_edit_page = MD400_EDIT_PAGE_PRESET;
                md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
                md400_edit_subject_before_pedal = MD400_EDIT_DEFAULT_SUBJECT;
                md400_pedal_selected_preset =
                    md400_pedal_active_preset;
                md400_pedal_edit_slot = 1;
                md400_pedal_edit_view =
                    MD400_PEDAL_EDIT_VIEW_ACTION;
                md400_pedal_label_position = 0;
            }
            md400_ui_display_sync();
            md400_log_event(0x02, event, 0, md400_ui_mode);
            printf("[NEXFOOT_UI] pedal edit mode=%u from=%u\r\n",
                   (unsigned)md400_ui_mode, (unsigned)mode_before);
            return;
        }

        if (md400_ui_mode == MD400_UI_MODE_EDIT) {
            md400_preset_load_timer_cancel();
            md400_pedal_cal_hide();
            md400_pedal_settings_enter_cancel();
            md400_edit_session_end(0);
            md400_ui_mode = MD400_UI_MODE_PERFORM;
            md400_sync_ms32_encoder_runtime_values();
        } else {
            md400_preset_load_timer_cancel();
            md400_pedal_cal_hide();
            md400_pedal_settings_enter_reset();
            md400_pedal_settings_seen = 0;
            md400_ui_mode = MD400_UI_MODE_EDIT;
            md400_edit_session_begin();
            md400_edit_page = MD400_EDIT_PAGE_PRESET;
            md400_edit_subject = MD400_EDIT_DEFAULT_SUBJECT;
            md400_selected_preset = md400_active_preset;
        }
        md400_ui_display_sync();
        md400_log_event(0x02, event, 0, md400_ui_mode);
        printf("[MD400_UI] mode=%u from=%u\r\n",
               (unsigned)md400_ui_mode, (unsigned)mode_before);
    } else if (event == MD400_EVENT_EDIT_SHORT) {
        u8 mode_before = md400_ui_mode;

        if (md400_ui_mode == MD400_UI_MODE_EDIT) {
            /* Pedal FT edit stays on its MIDI parameters. The old FT/POS/CHAR/
             * COLOR label-style page is not part of the device edit flow. */
            md400_pedal_edit_view = MD400_PEDAL_EDIT_VIEW_ACTION;
            md400_log_event(0x02, event, 0, md400_ui_mode);
            printf("[MD400_UI] short ignored in edit mode=%u\r\n",
                   (unsigned)md400_ui_mode);
            return;
        }
        md400_ui_mode = (md400_ui_mode == MD400_UI_MODE_PLAY) ?
                         MD400_UI_MODE_PERFORM : MD400_UI_MODE_PLAY;
        md400_ui_display_sync();
        md400_log_event(0x02, event, mode_before, md400_ui_mode);
        printf("[MD400_UI] mode=%u from=%u short\r\n",
               (unsigned)md400_ui_mode, (unsigned)mode_before);
    } else if (event == MD400_EVENT_EDIT_DOUBLE) {
        u8 saved = 0;

        if (md400_ui_mode == MD400_UI_MODE_PERFORM) {
            md400_log_event(0x02, event, md400_foot_mode,
                            md400_foot_mode ^ 1);
            md400_foot_mode_set(md400_foot_mode ^ 1);
            return;
        }
        if (md400_ui_mode == MD400_UI_MODE_PLAY) {
            md400_log_event(0x02, event, md400_foot_mode,
                            md400_foot_mode);
            printf("[MD400_UI] double ignored in play mode\r\n");
            return;
        }
        if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
            if (!md400_persist_submit(MD400_PERSIST_MSG_PEDAL_SAVE, 1, 0)) {
                saved = 1;
            }
            if (md400_pedal_exp_dirty) {
                md400_pedal_exp_snapshot = *md400_get_pedal_edit_assignment();
                md400_store_save_schedule(MD400_STORE_SAVE_REASON_EDIT);
                md400_pedal_exp_dirty = 0;
                saved = 1;
            }
            if (saved) {
                md400_display_show_save_check();
            }
            md400_log_event(0x02, event,
                            md400_pedal_edit_slot,
                            md400_pedal_page + 1);
            md400_ui_display_sync();
            return;
        }
        md400_log_event(0x02, event, 0, md400_active_preset + 1);
        md400_edit_session_save();
        md400_ui_display_sync();
    }
}

static u8 md400_edit_key_raw_pressed(void)
{
    return gpio_read(MD400_EDIT_KEY_PIN) == 0;
}

static void md400_edit_key_short_timeout(void *priv)
{
    (void)priv;
    md400_edit_key_short_timer = 0;
    if (md400_edit_key_click_count == 1) {
        md400_edit_key_short_count++;
        printf("[MD400_UI] key short physical mode=%u count=%u\r\n",
               (unsigned)md400_ui_mode, (unsigned)md400_edit_key_short_count);
        md400_edit_key_event(MD400_EVENT_EDIT_SHORT);
    }
    md400_edit_key_click_count = 0;
}

static void md400_edit_key_release_short(void)
{
    if (md400_edit_key_click_count == 0) {
        md400_edit_key_click_count = 1;
        if (md400_edit_key_short_timer) {
            sys_timeout_del(md400_edit_key_short_timer);
        }
        md400_edit_key_short_timer =
            sys_timeout_add(NULL, md400_edit_key_short_timeout, MD400_EDIT_KEY_DOUBLE_MS);
    } else {
        if (md400_edit_key_short_timer) {
            sys_timeout_del(md400_edit_key_short_timer);
            md400_edit_key_short_timer = 0;
        }
        md400_edit_key_click_count = 0;
        md400_edit_key_double_count++;
        printf("[MD400_UI] key double physical mode=%u count=%u\r\n",
               (unsigned)md400_ui_mode, (unsigned)md400_edit_key_double_count);
        md400_edit_key_event(MD400_EVENT_EDIT_DOUBLE);
    }
}

static void md400_edit_key_scan(void *priv)
{
    u8 raw;

    (void)priv;
    raw = md400_edit_key_raw_pressed();

    if (raw != md400_edit_key_last_raw) {
        md400_edit_key_last_raw = raw;
        md400_edit_key_debounce_ticks = MD400_EDIT_KEY_DEBOUNCE_MS / MD400_EDIT_KEY_SCAN_MS;
        return;
    }

    if (md400_edit_key_debounce_ticks) {
        md400_edit_key_debounce_ticks--;
        return;
    }

    if (raw != md400_edit_key_stable) {
        md400_edit_key_stable = raw;
        md400_edit_key_edge_count++;
        if (raw) {
            md400_edit_key_press_ticks = 0;
            md400_edit_key_long_sent = 0;
        } else {
            if (!md400_edit_key_long_sent) {
                md400_edit_key_release_short();
            }
        }
    }

    if (md400_edit_key_stable) {
        md400_edit_key_press_ticks++;
        if (!md400_edit_key_long_sent &&
            (md400_edit_key_press_ticks >= (MD400_EDIT_KEY_LONG_MS / MD400_EDIT_KEY_SCAN_MS))) {
            md400_edit_key_long_sent = 1;
            if (md400_edit_key_short_timer) {
                sys_timeout_del(md400_edit_key_short_timer);
                md400_edit_key_short_timer = 0;
            }
            md400_edit_key_click_count = 0;
            md400_edit_key_long_count++;
            printf("[MD400_UI] key long physical mode=%u count=%u\r\n",
                   (unsigned)md400_ui_mode, (unsigned)md400_edit_key_long_count);
            md400_edit_key_event(MD400_EVENT_EDIT_LONG);
        }
    }
}

static void md400_edit_key_init(void)
{
    if (md400_edit_key_timer) {
        return;
    }

    gpio_set_die(MD400_EDIT_KEY_PIN, 1);
    gpio_set_pull_up(MD400_EDIT_KEY_PIN, 1);
    gpio_set_pull_down(MD400_EDIT_KEY_PIN, 0);
    gpio_direction_input(MD400_EDIT_KEY_PIN);
    md400_edit_key_last_raw = md400_edit_key_raw_pressed();
    md400_edit_key_stable = md400_edit_key_last_raw;
    md400_edit_key_timer = sys_timer_add(NULL, md400_edit_key_scan, MD400_EDIT_KEY_SCAN_MS);
    printf("[MD400_UI] edit key PA2 scan=%ums\r\n", (unsigned)MD400_EDIT_KEY_SCAN_MS);
}

void md400_preset_storage_boot_prepare(void)
{
    int result = md400_preset_storage_init(&md400_pedal_bank_storage,
                                         sizeof(md400_pedal_bank_storage));
    if (result > 0) {
        printf("[MD400_STORE] SDK settings restored; restart\r\n");
        cpu_reset();
    } else if (result < 0) {
        printf("[MD400_STORE] journal initialization failed\r\n");
    }
}

static void md400_control_init(void)
{
    if (md400_control_ready) {
        return;
    }

    md400_store_load();
    md400_pedal_header_load();
    md400_pedal_mode_load();
    md400_pedal_cal_load();
    md400_ms32_set_adc_event_cb(md400_pedal_adc_event);
    md400_edit_key_init();
    if (!md400_led_pattern_timer) {
        md400_led_pattern_timer = sys_timer_add(NULL, md400_led_pattern_tick, 120);
    }
    if (!md400_daw_timer) {
        md400_daw_timer = sys_timer_add(NULL, md400_daw_timer_tick, MD400_DAW_TICK_MS);
    }
    md400_control_ready = 1;
    md400_ui_display_sync();
}

void md400_display_boot_state_prepare(void)
{
    /*
     * The panel starts only 100 ms after app startup, which can precede the
     * normal BLE-side control initialization. Load the saved foot mode and its
     * active PEDAL action colors before the first boot-animation frame.
     */
    md400_control_init();
}

#define MD400_DETACH_SYSEX_MAX     192

static const u8 md400_detach_magic[] = {0x7D, 'J', 'L', 'O', 'T', 'A'};
static u16 md400_usb_mask_update_timer;
static u8 md400_usb_mask_update_pending;

static u8 md400_usb_midi_mask_update_schedule(void);

static void md400_put_u14(u8 *p, u16 v)
{
    p[0] = v & 0x7F;
    p[1] = (v >> 7) & 0x7F;
}

static void md400_put_s14(u8 *p, int value)
{
    u16 enc;

    if (value < 0) {
        enc = 0x2000 | ((u16)(-value) & 0x1fff);
    } else {
        enc = (u16)value & 0x1fff;
    }
    md400_put_u14(p, enc);
}

static u16 md400_get_u14(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 7);
}

static int md400_usb_midi_control_send_sysex(const u8 *body, u16 body_len)
{
    u8 packet[((MD400_NB_MAX_WIRE_REPLY + 12) / 3) * 4];
    u16 pos = 0, used = 0, raw_len = body_len + 2;
    if (!body || !body_len || body_len > MD400_NB_MAX_WIRE_REPLY + 8) return -1;
    while (pos < raw_len) {
        u8 take = raw_len - pos >= 3 ? 3 : raw_len - pos;
        u8 k;
        packet[used++] = USB_MIDI_EVENT(USB_MIDI_CABLE_CONTROL,
                raw_len - pos <= 3 ? 4 + take : 4);
        for (k = 0; k < 3; ++k) {
            u16 at = pos + k;
            packet[used++] = k >= take ? 0 : at == 0 ? 0xf0 :
                            at == raw_len - 1 ? 0xf7 : body[at - 1];
        }
        pos += take;
    }
    /* A control reply is accepted completely or reported as backpressure. */
    return usb_midi_device_send(0, packet, used) == used ? 0 : -1;
}

static u8 md400_nb_checksum(const u8 *data, u16 len)
{
    u16 i;
    u16 sum = 0;

    for (i = 0; i < len; i++) {
        sum += data[i] & 0x7f;
    }

    return (0x80 - (sum & 0x7f)) & 0x7f;
}

#ifdef NEXFOOT_WEB_BOOT
static void md400_nb_web_update_timeout(void *priv)
{
    (void)priv;
    local_irq_disable();
    // JieLi user boot's retained nvram_list field is at 0x800 on BD19.
    *(volatile u32 *)0x800u = 0x4857464eu;
    __asm__ volatile("ssync" ::: "memory");
    cpu_reset();
}
#endif

static void md400_nb_reset_timeout(void *priv)
{
    (void)priv;
    printf("[MD400_NB] reset\r\n");
    cpu_reset();
}

#if CONFIG_MD400_UART0_RX_TEST_ENABLE
static void md400_nb_wdt_test_timeout(void *priv)
{
    (void)priv;
    local_irq_disable();
    wdt_init(WDT_16MS);
    while (1) {
    }
}
#endif

static u8 md400_nb_reset_payload_valid(const u8 *payload, u8 payload_len)
{
    static const u8 magic[] = { 'R', 'E', 'S', 'E', 'T' };

    return payload && (payload_len == sizeof(magic)) &&
           !memcmp(payload, magic, sizeof(magic));
}

static u8 md400_nb_factory_reset_payload_valid(const u8 *payload, u8 payload_len)
{
    static const u8 magic[] = { 'F', 'A', 'C', 'T', 'O', 'R', 'Y' };

    return payload && (payload_len == sizeof(magic)) &&
           !memcmp(payload, magic, sizeof(magic));
}

static u8 md400_nb_pedal_flush_status(void)
{
    if (!md400_pedal_record_dirty) {
        return MD400_NB_STATUS_OK;
    }
    return (md400_pedal_bank_save() > 0) ?
           MD400_NB_STATUS_OK : MD400_NB_STATUS_FLASH_ERROR;
}

static u8 md400_nb_store_save_status(void)
{
    if (!md400_pedal_names_save()) return MD400_NB_STATUS_FLASH_ERROR;
    if (md400_nb_pedal_flush_status() != MD400_NB_STATUS_OK) {
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    return (md400_store_save() == sizeof(md400_store)) ?
           MD400_NB_STATUS_OK : MD400_NB_STATUS_FLASH_ERROR;
}

static u8 md400_nb_header_save_status(void)
{
    md400_store.active_preset = md400_active_preset;
    md400_store.checksum = md400_store_checksum(&md400_store);
    return (md400_header_save() > 0) ?
           MD400_NB_STATUS_OK : MD400_NB_STATUS_FLASH_ERROR;
}

static void md400_header_save_timeout(void *priv)
{
    (void)priv;
    if (md400_persist_submit(MD400_PERSIST_MSG_HEADER, 0, 0)) {
        sys_timeout_add(NULL, md400_header_save_timeout, MD400_NB_STORE_JOB_DELAY_MS);
    }
}

static void md400_header_save_schedule(void)
{
    if (md400_header_save_pending) {
        return;
    }

    md400_header_save_pending = 1;
    sys_timeout_add(NULL, md400_header_save_timeout, MD400_NB_STORE_JOB_DELAY_MS);
}

static void md400_store_save_timeout(void *priv)
{
    (void)priv;
    if (md400_persist_submit(MD400_PERSIST_MSG_STORE, 0, 0)) {
        sys_timeout_add(NULL, md400_store_save_timeout, MD400_NB_STORE_JOB_DELAY_MS);
    }
}

static void md400_store_save_schedule(u8 reason)
{
    md400_store_save_reason = reason;
    if (md400_store_save_pending) {
        return;
    }

    md400_store_save_pending = 1;
    sys_timeout_add(NULL, md400_store_save_timeout, MD400_NB_STORE_JOB_DELAY_MS);
}

static void md400_nb_count_inc(u16 *count)
{
    *count = (*count + 1) & 0x3fff;
}

static void md400_nb_status_account(u8 status)
{
    md400_nb_count_inc(&md400_nb_reply_count);

    switch (status & 0x7f) {
    case MD400_NB_STATUS_OK:
        md400_nb_count_inc(&md400_nb_status_ok_count);
        break;
    case MD400_NB_STATUS_BAD_CHECKSUM:
        md400_nb_count_inc(&md400_nb_status_bad_checksum_count);
        break;
    case MD400_NB_STATUS_BAD_LENGTH:
        md400_nb_count_inc(&md400_nb_status_bad_length_count);
        break;
    case MD400_NB_STATUS_BAD_VALUE:
        md400_nb_count_inc(&md400_nb_status_bad_value_count);
        break;
    case MD400_NB_STATUS_UNSUPPORTED:
        md400_nb_count_inc(&md400_nb_status_unsupported_count);
        break;
    case MD400_NB_STATUS_BUSY:
        md400_nb_count_inc(&md400_nb_status_busy_count);
        break;
    case MD400_NB_STATUS_FLASH_ERROR:
        md400_nb_count_inc(&md400_nb_status_flash_error_count);
        break;
    default:
        break;
    }
}

static int md400_nb_ble_enqueue(md400_nb_origin_t origin, const u8 *body, u16 length)
{
    md400_nb_ble_reply_t *reply;
    if (length + 2 > MD400_NB_BLE_FRAME_SIZE) return -1;
    local_irq_disable();
    if (!origin.handle || origin.handle != trans_con_handle ||
            origin.epoch != md400_nb_ble_epoch || md400_nb_ble_tx_count == 2) {
        local_irq_enable();
        return -1;
    }
    reply = &md400_nb_ble_replies[(md400_nb_ble_tx_head + md400_nb_ble_tx_count) & 1];
    reply->origin = origin;
    reply->data[0] = 0xf0;
    memcpy(reply->data + 1, body, length);
    reply->data[length + 1] = 0xf7;
    reply->len = length + 2;
    reply->pos = 0;
    ++md400_nb_ble_tx_count;
    local_irq_enable();
    return 0;
}

static void md400_nb_ble_link_reset(void)
{
    ++md400_nb_ble_epoch;
    md400_nb_ble_mtu = 23;
    md400_nb_ble_editor = 0;
    md400_nb_ble_tx_count = md400_nb_ble_tx_head = 0;
    memset(&md400_ble_decoder, 0, sizeof(md400_ble_decoder));
    memset(&md400_nb_ble_stream, 0, sizeof(md400_nb_ble_stream));
}

static u8 md400_nb_ble_tx_pump(void)
{
    static u8 packet[128]; /* owned by the existing single BLE TX pump */
    md400_nb_ble_reply_t *reply;
    md400_nb_origin_t origin;
    u16 count, size, limit;
    u8 head;
    if ((md400_nb_ble_editor || md400_nb_usb_subscribed) && !md400_nb_notify_pending &&
        timer_get_ms() - md400_nb_notify_last_post >= 100) {
        md400_nb_notify_last_post = timer_get_ms();
        md400_nb_notify_pending = 1;
        if (md400_persist_submit(MD400_PERSIST_MSG_NB_NOTIFY, 0, 0)) md400_nb_notify_pending = 0;
    }
    local_irq_disable();
    if (!md400_nb_ble_tx_count) { local_irq_enable(); return 0; }
    head = md400_nb_ble_tx_head;
    reply = &md400_nb_ble_replies[head];
    origin = reply->origin;
    if (origin.handle != trans_con_handle || origin.epoch != md400_nb_ble_epoch) {
        md400_nb_ble_tx_head = (head + 1) & 1;
        --md400_nb_ble_tx_count;
        local_irq_enable();
        return 1;
    }
    limit = md400_nb_ble_mtu > 6 ? md400_nb_ble_mtu - 6 : 17;
    if (limit > sizeof(packet) - 3) limit = sizeof(packet) - 3;
    count = reply->len - reply->pos;
    if (count > limit) count = limit;
    size = ble_midi_encode_packet(reply->data + reply->pos, count, packet, sizeof(packet));
    local_irq_enable();
    if (ble_comm_att_check_send(origin.handle, size) &&
        !ble_comm_att_send_data(origin.handle,
            ATT_CHARACTERISTIC_BLE_MIDI_IO_01_VALUE_HANDLE, packet, size, ATT_OP_AUTO_READ_CCC)) {
        local_irq_disable();
        if (md400_nb_ble_tx_count && head == md400_nb_ble_tx_head && origin.epoch == md400_nb_ble_epoch) {
            reply->pos += count;
            if (reply->pos == reply->len) {
                md400_nb_ble_tx_head = (head + 1) & 1;
                --md400_nb_ble_tx_count;
            }
        }
        local_irq_enable();
    }
    return 1;
}

static int md400_nb_send_reply(u8 cmd, u8 seq, u8 status, const u8 *payload, u8 payload_len)
{
    u8 body[MD400_NB_MAX_WIRE_REPLY + 8];
    u8 body_len;

    if (payload_len > MD400_NB_MAX_WIRE_REPLY) return -1;

    md400_nb_status_account(status);

    body[0] = 0x7D;
    body[1] = 'N';
    body[2] = 'B';
    body[3] = 0x01;
    body[4] = (cmd | 0x80) & 0x7f;
    body[5] = seq & 0x7f;
    body[6] = status & 0x7f;
    if (payload && payload_len) {
        memcpy(&body[7], payload, payload_len);
    }
    body_len = 7 + payload_len;
    body[body_len] = md400_nb_checksum(&body[3], body_len - 3);
    body_len++;
    if (md400_nb_origin.source == 1) return md400_nb_ble_enqueue(md400_nb_origin, body, body_len);
    return md400_usb_midi_control_send_sysex(body, body_len);
}

/* DAW_DRAW frames skip the two request slots: a screen is many frames in a
 * row. Valid frames go straight to the display's drawing queue. Returns 1 if
 * the frame was a DAW_DRAW frame (handled or dropped). */
static u8 md400_daw_draw_frame(const u8 *frame, u16 length)
{
    if ((frame[0] != 0xF0) || (frame[length - 1] != 0xF7) || (frame[1] != 0x7D) ||
        (frame[2] != 'N') || (frame[3] != 'B') || (frame[4] != 0x01) ||
        ((frame[5] & 0x7f) != MD400_NB_CMD_DAW_DRAW)) {
        return 0;
    }
    if (md400_nb_checksum(&frame[4], length - 6) != (frame[length - 2] & 0x7f)) {
        return 1;
    }
    if (!md400_daw_active || md400_daw_waiting) {
        return 1;
    }
    md400_daw_host_ms = sys_timer_get_ms();
    md400_daw_host_seen = 1;
    if (md400_display_daw_draw_push(&frame[7], (u8)(length - 9))) {
        md400_daw_draw_rx_count++;
    } else {
        md400_daw_draw_drop_count++;
        md400_daw_draw_overflow = 1;
    }
    return 1;
}

static void md400_nb_submit(md400_nb_origin_t origin, const u8 *frame, u16 length)
{
    u8 slot;
    if (length < 9 || length > MD400_NB_STREAM_LIMIT) return;
    if (!origin.source && md400_daw_draw_frame(frame, length)) return;
    local_irq_disable();
    for (slot = 0; slot < 2; ++slot) if (!md400_nb_requests[slot].active) break;
    if (slot == 2) { local_irq_enable(); return; }
    md400_nb_requests[slot].origin = origin;
    md400_nb_requests[slot].len = length;
    memcpy(md400_nb_requests[slot].data, frame, length);
    md400_nb_requests[slot].active = 1;
    local_irq_enable();
    if (md400_persist_submit(MD400_PERSIST_MSG_NB_FRAME, slot, 0)) {
        local_irq_disable();
        md400_nb_requests[slot].active = 0;
        local_irq_enable();
    }
}

static void md400_nb_process_request(u8 slot)
{
    md400_nb_origin_t saved = md400_nb_origin, origin;
    u8 data[MD400_NB_STREAM_LIMIT], length;
    if (slot >= 2 || !md400_nb_requests[slot].active) return;
    local_irq_disable();
    origin = md400_nb_requests[slot].origin;
    length = md400_nb_requests[slot].len;
    memcpy(data, md400_nb_requests[slot].data, length);
    md400_nb_requests[slot].active = 0;
    local_irq_enable();
    if (origin.source == 1 && (origin.handle != trans_con_handle || origin.epoch != md400_nb_ble_epoch)) return;
    md400_nb_origin = origin;
    md400_nb_dispatch_active = 1;
    md400_nb_sysex_handle(data, length);
    md400_nb_dispatch_active = 0;
    md400_nb_origin = saved;
}

static void md400_nb_note_change(u8 source)
{
    md400_nb_revision = (md400_nb_revision + 1) & 0x3fff;
    md400_nb_change_source = source;
}

static u8 md400_nb_changes_state(u8 command)
{
    switch (command) {
    case 0x11: case 0x12: case 0x13: case 0x15: case 0x16:
    case 0x21: case 0x23: case 0x25: case 0x35: case 0x37: case 0x39:
    case 0x40: case 0x41: case 0x43: case 0x45: case 0x47:
    case 0x4A: case 0x4C: return 1;
    default: return 0;
    }
}

static void md400_nb_store_job_run(void)
{
    md400_nb_origin_t saved_origin = md400_nb_origin;
    u8 cmd;
    u8 seq;
    u8 payload_len;
    u8 payload[MD400_NB_STORE_JOB_MAX_PAYLOAD];
    u8 reply[MD400_NB_MAX_REPLY_PAYLOAD];
    u8 reply_len = 0;
    u8 status = MD400_NB_STATUS_OK;

    local_irq_disable();
    if (!md400_nb_store_job.active) {
        local_irq_enable();
        return;
    }
    md400_nb_origin = md400_nb_store_job.origin;
    cmd = md400_nb_store_job.cmd;
    seq = md400_nb_store_job.seq;
    payload_len = md400_nb_store_job.payload_len;
    memcpy(payload, md400_nb_store_job.payload, sizeof(payload));
    local_irq_enable();

    printf("[MD400_NB] store job run cmd=0x%x seq=%u len=%u\r\n",
           (unsigned)cmd, (unsigned)seq, (unsigned)payload_len);

    switch (cmd) {
    case MD400_NB_CMD_SELECT_VIEW: {
        u8 previous_mode = md400_ui_mode;
        if (md400_pedal_record_dirty || md400_pedal_names_dirty ||
            md400_pedal_exp_dirty || md400_edit_session_dirty) {
            status = MD400_NB_STATUS_BUSY;
            break;
        }
        if (payload[0] && !md400_pedal_bank_load(payload[1] - 1, payload[2] - 1)) {
            status = MD400_NB_STATUS_FLASH_ERROR;
            break;
        }
        md400_preset_load_timer_cancel();
        md400_edit_session_end(0);
        md400_pedal_cal_hide();
        md400_pedal_settings_enter_reset();
        md400_button_down_mask = md400_combo_mask = 0;
        memset(md400_button_gesture, 0, sizeof(md400_button_gesture));
        md400_foot_mode = payload[0];
        if (payload[0]) {
            md400_pedal_active_preset = md400_pedal_selected_preset = payload[1] - 1;
            md400_pedal_page = payload[2] - 1;
        } else {
            md400_active_preset = md400_selected_preset = payload[1] - 1;
            md400_store.active_preset = md400_active_preset;
            md400_sync_runtime_defaults();
        }
        md400_ui_mode = previous_mode == MD400_UI_MODE_PLAY ? MD400_UI_MODE_PLAY : MD400_UI_MODE_PERFORM;
        md400_sync_ms32_encoder_runtime_values();
        md400_ui_display_sync();
        md400_nb_state_payload(reply);
        reply_len = 12;
        break;
    }

    case MD400_NB_CMD_SAVE_PRESET:
        if (payload_len) {
            md400_active_preset = payload[0] - 1;
        }
        status = md400_nb_store_save_status();
        break;

    case MD400_NB_CMD_LOAD_PRESET:
        {
            u8 previous_active = md400_active_preset;
            md400_active_preset = payload[0] - 1;
            md400_store.active_preset = md400_active_preset;
            md400_sync_runtime_defaults();
            status = md400_nb_header_save_status();
            if (status != MD400_NB_STATUS_OK) {
                md400_active_preset = previous_active;
                md400_store.active_preset = previous_active;
                md400_sync_runtime_defaults();
            }
            md400_ui_display_sync();
        }
        break;

    case MD400_NB_CMD_FACTORY_RESET:
        if (md400_factory_reset_save() == sizeof(md400_store)) {
            reply[0] = MD400_PRESET_COUNT;
            reply_len = 1;
        } else {
            status = MD400_NB_STATUS_FLASH_ERROR;
        }
        md400_template_record_buf_release();
        break;

    case MD400_NB_CMD_SET_PLAY_LABEL:
        status = md400_nb_apply_play_label_payload(payload, payload_len);
        if (!status) status = md400_nb_play_label_to_payload(payload, 3, reply, &reply_len);
        break;

    case MD400_NB_CMD_GET_PLAY_LABEL:
        status = md400_nb_play_label_to_payload(payload, payload_len,
                                                reply, &reply_len);
        break;

    case MD400_NB_CMD_SET_SYSEX_TEMPLATE:
        status = md400_nb_apply_sysex_template_payload(payload, payload_len);
        md400_template_record_buf_release();
        break;

    case MD400_NB_CMD_GET_SYSEX_TEMPLATE:
        status = md400_nb_sysex_template_to_payload(payload, payload_len,
                                                    reply, &reply_len);
        md400_template_record_buf_release();
        break;

    case MD400_NB_CMD_SET_CUSTOM_TEMPLATE:
        status = md400_nb_apply_custom_template_payload(payload, payload_len);
        md400_template_record_buf_release();
        break;

    case MD400_NB_CMD_GET_CUSTOM_TEMPLATE:
        status = md400_nb_custom_template_to_payload(payload, payload_len,
                                                     reply, &reply_len);
        md400_template_record_buf_release();
        break;

    case MD400_NB_CMD_SET_FOOT_MODE:
        md400_foot_mode_set_now(payload[0]);
        break;

    default:
        status = MD400_NB_STATUS_UNSUPPORTED;
        break;
    }

    printf("[MD400_NB] store job done cmd=0x%x status=%u\r\n",
           (unsigned)cmd, (unsigned)status);
    /* Release the one-request slot before publishing the ACK. USB IN can wake
     * the host immediately, before md400_nb_send_reply() has unwound. */
    local_irq_disable();
    md400_nb_store_job.active = 0;
    local_irq_enable();
    if (!status && md400_nb_changes_state(cmd)) md400_nb_note_change(md400_nb_origin.source);
    md400_nb_send_reply(cmd, seq, status, reply, reply_len);
    md400_nb_origin = saved_origin;
}

static u8 md400_nb_store_job_start(u8 cmd, u8 seq, const u8 *payload, u8 payload_len)
{
    if (payload_len > MD400_NB_STORE_JOB_MAX_PAYLOAD) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    local_irq_disable();
    if (md400_nb_store_job.active) {
        local_irq_enable();
        return MD400_NB_STATUS_BUSY;
    }

    memset(&md400_nb_store_job, 0, sizeof(md400_nb_store_job));
    md400_nb_store_job.origin = md400_nb_origin;
    md400_nb_store_job.active = 1;
    md400_nb_store_job.cmd = cmd;
    md400_nb_store_job.seq = seq;
    md400_nb_store_job.payload_len = payload_len;
    if (payload && payload_len) {
        memcpy(md400_nb_store_job.payload, payload, payload_len);
    }
    local_irq_enable();

    if (md400_persist_in_task) {
        md400_nb_store_job_run();
        return MD400_NB_STATUS_OK;
    }
    if (md400_persist_submit(MD400_PERSIST_MSG_NANOBRIDGE, 0, 0)) {
        local_irq_disable();
        md400_nb_store_job.active = 0;
        local_irq_enable();
        return MD400_NB_STATUS_BUSY;
    }
    return MD400_NB_STATUS_OK;
}

static int md400_persist_post_callback(void)
{
    int argv[3];
    int ret;

    argv[0] = (int)md400_persist_app_callback;
    argv[1] = 1;
    argv[2] = 0;
    ret = os_taskq_post_type("app_core", Q_CALLBACK, ARRAY_SIZE(argv), argv);
    if (ret) {
        printf("[MD400_PERSIST] app_core callback post failed ret=%d\r\n", ret);
    }
    return ret;
}

static void md400_persist_post_retry_timeout(void *priv)
{
    u8 needed;

    (void)priv;
    md400_persist_post_retry_timer = 0;
    local_irq_disable();
    needed = md400_persist_callback_pending && md400_persist_queue_count;
    local_irq_enable();
    if (needed && md400_persist_post_callback()) {
        md400_persist_post_retry_timer =
            sys_timeout_add(NULL, md400_persist_post_retry_timeout,
                            MD400_NB_STORE_JOB_DELAY_MS);
    }
}

static int md400_persist_submit(u8 message, u8 arg0, u8 arg1)
{
    u8 post_callback = 0;
    u8 tail;

    if (!message) {
        return 0;
    }

    local_irq_disable();
    if (md400_persist_queue_count >= MD400_PERSIST_QUEUE_DEPTH) {
        local_irq_enable();
        printf("[MD400_PERSIST] queue full msg=%u\r\n", (unsigned)message);
        return -1;
    }
    tail = md400_persist_queue_tail;
    md400_persist_queue[tail].message = message;
    md400_persist_queue[tail].arg0 = arg0;
    md400_persist_queue[tail].arg1 = arg1;
    md400_persist_queue_tail = (tail + 1) % MD400_PERSIST_QUEUE_DEPTH;
    md400_persist_queue_count++;
    if (!md400_persist_callback_pending) {
        md400_persist_callback_pending = 1;
        post_callback = 1;
    }
    local_irq_enable();

    if (post_callback && md400_persist_post_callback()) {
        /* The mutation is already in the ring. Returning BUSY here would be
         * misleading because a later callback could still execute it. Retry
         * only the app_core post from systimer; flash remains on app_core. */
        if (!md400_persist_post_retry_timer) {
            md400_persist_post_retry_timer =
                sys_timeout_add(NULL, md400_persist_post_retry_timeout,
                                MD400_NB_STORE_JOB_DELAY_MS);
        }
    }
    return 0;
}

static u8 md400_nb_validate_assignment_fields(const u8 *payload, u8 len)
{
    if (!payload || (len < 9)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if (payload[0] > MD400_MIDI_TYPE_MAX) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if ((payload[1] < 1) || (payload[1] > 16)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload[2] > 0x7f || payload[3] > 0x7f || payload[4] > 0x7f || payload[5] > 0x7f) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload[6] > MD400_BEHAVIOR_PULSE) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload[7] > 0x7f) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload[8] > 0x7f) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_validate_assignment_payload(const u8 *payload, u8 len)
{
    if (!payload || (len < 10)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_ENCODER_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    return md400_nb_validate_assignment_fields(&payload[1], len - 1);
}

static u8 md400_nb_validate_gesture_assignment_payload(const u8 *payload, u8 len)
{
    if (!payload || (len < 11)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if (!md400_get_gesture_assignment(payload[0], payload[1])) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    return md400_nb_validate_assignment_fields(&payload[2], len - 2);
}

static void md400_nb_assignment_fields_to_payload(const md400_assignment_t *assignment, u8 *payload)
{
    payload[0] = assignment->type & 0x7f;
    payload[1] = (assignment->channel & 0x0f) + 1;
    payload[2] = assignment->data1 & 0x7f;
    payload[3] = assignment->value_min & 0x7f;
    payload[4] = assignment->value_max & 0x7f;
    payload[5] = assignment->value_default & 0x7f;
    payload[6] = assignment->behavior & 0x7f;
    payload[7] = md400_available_target_mask() & 0x7f;
    payload[8] = assignment->sysex_id & 0x7f;
}

static void md400_nb_assignment_apply_fields(md400_assignment_t *assignment, const u8 *payload)
{
    assignment->type = payload[0] & 0x7f;
    assignment->channel = (payload[1] - 1) & 0x0f;
    assignment->data1 = payload[2] & 0x7f;
    assignment->value_min = payload[3] & 0x7f;
    assignment->value_max = payload[4] & 0x7f;
    assignment->value_default = payload[5] & 0x7f;
    assignment->behavior = payload[6] & 0x7f;
    assignment->target_mask = md400_available_target_mask();
    assignment->sysex_id = payload[8] & 0x7f;
}

static u8 md400_preset_label_len(const md400_preset_t *preset)
{
    u8 i;

    if (!preset) {
        return 0;
    }
    for (i = 0; i < MD400_PRESET_LABEL_LEN; i++) {
        if (!preset->label[i]) {
            return i;
        }
    }
    return MD400_PRESET_LABEL_LEN;
}

static u8 md400_nb_preset_label_to_payload(u8 preset_number, u8 *payload, u8 *payload_len)
{
    md400_preset_t *preset;
    u8 label_len;

    if (!payload || !payload_len ||
        (preset_number < 1) || (preset_number > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset = &md400_store.preset[preset_number - 1];
    label_len = md400_preset_label_len(preset);
    payload[0] = preset_number;
    payload[1] = label_len;
    if (label_len) {
        memcpy(&payload[2], preset->label, label_len);
    }
    *payload_len = 2 + label_len;
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_preset_label_payload(const u8 *payload, u8 payload_len)
{
    md400_preset_t *preset;
    u8 preset_index;
    u8 label_len;
    u8 i;

    if (!payload || (payload_len < 2)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    label_len = payload[1] & 0x7f;
    if ((label_len > MD400_PRESET_LABEL_LEN) || (payload_len < (2 + label_len))) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    for (i = 0; i < label_len; i++) {
        if ((payload[2 + i] < 0x20) || (payload[2 + i] > 0x7e)) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }

    preset_index = payload[0] - 1;
    preset = &md400_store.preset[preset_index];
    memset(preset->label, 0, sizeof(preset->label));
    if (label_len) {
        memcpy(preset->label, &payload[2], label_len);
    }
    md400_store.checksum = md400_store_checksum(&md400_store);
    md400_log_event(0x06, MD400_NB_CMD_SET_PRESET_LABEL, payload[0], label_len);
    printf("[MD400_CFG] set label preset=%u len=%u\r\n",
           (unsigned)payload[0], (unsigned)label_len);
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_play_label_to_payload(const u8 *payload, u8 payload_len,
                                         u8 *reply, u8 *reply_len)
{
    u8 preset_index;
    u8 action;
    u8 control;
    u8 slot;
    u8 label_len;

    if (!payload || !reply || !reply_len || (payload_len < 3)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    action = payload[1] & 0x7f;
    control = payload[2] & 0x7f;
    slot = md400_play_label_slot(action, control);
    if (slot == 0xff) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_play_label_source(preset_index, md400_play_label_work);
    label_len = md400_play_label_len(md400_play_label_work[slot]);
    reply[0] = payload[0];
    reply[1] = action;
    reply[2] = control;
    reply[3] = label_len;
    if (label_len) {
        memcpy(&reply[4], md400_play_label_work[slot], label_len);
    }
    *reply_len = 4 + label_len;
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_play_label_payload(const u8 *payload, u8 payload_len)
{
    u8 preset_index;
    u8 action;
    u8 control;
    u8 slot;
    u8 label_len;
    u8 i;
    u8 status;

    if (!payload || (payload_len < 4)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    action = payload[1] & 0x7f;
    control = payload[2] & 0x7f;
    label_len = payload[3] & 0x7f;
    slot = md400_play_label_slot(action, control);
    if (slot == 0xff) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if ((label_len > MD400_PLAY_LABEL_LEN) || (payload_len < (4 + label_len))) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    for (i = 0; i < label_len; i++) {
        if ((payload[4 + i] < 0x20) || (payload[4 + i] > 0x7e)) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }

    md400_play_label_source(preset_index, md400_play_label_work);
    memset(md400_play_label_work[slot], 0, MD400_PLAY_LABEL_LEN);
    if (label_len) {
        memcpy(md400_play_label_work[slot], &payload[4], label_len);
    }

    status = md400_play_label_record_write(preset_index, md400_play_label_work);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }

    if (preset_index == md400_active_preset) {
        memcpy(md400_play_label, md400_play_label_work,
               MD400_PLAY_LABEL_SLOT_COUNT * MD400_PLAY_LABEL_LEN);
        md400_play_refresh_display();
        md400_ui_display_sync();
    }

    md400_log_event(0x06, MD400_NB_CMD_SET_PLAY_LABEL, payload[0], label_len);
    printf("[MD400_CFG] set play label preset=%u action=%u control=%u len=%u\r\n",
           (unsigned)payload[0], (unsigned)action, (unsigned)control,
           (unsigned)label_len);
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_sysex_template_to_payload(const u8 *payload, u8 payload_len,
                                             u8 *reply, u8 *reply_len)
{
    u8 preset_index;
    u8 slot;
    u16 offset;
    u16 remain_len;
    u8 chunk_len;
    md400_sysex_template_t *template;

    if (!payload || !reply || !reply_len || (payload_len < 4)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_SYSEX_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    slot = payload[1] - 1;
    offset = md400_get_u14(&payload[2]);
    if (!md400_sysex_template_load_to_buf(preset_index)) {
        return MD400_NB_STATUS_BUSY;
    }
    template = &md400_sysex_template_buf->slot[slot];
    if (offset > template->len) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    remain_len = template->len - offset;
    chunk_len = (remain_len > MD400_SYSEX_TEMPLATE_CHUNK) ?
                MD400_SYSEX_TEMPLATE_CHUNK : remain_len;
    reply[0] = payload[0];
    reply[1] = payload[1];
    reply[2] = template->value_index & 0x7f;
    md400_put_u14(&reply[3], template->len);
    md400_put_u14(&reply[5], offset);
    reply[7] = chunk_len & 0x7f;
    if (chunk_len) {
        memcpy(&reply[8], &template->body[offset], chunk_len);
    }
    *reply_len = 8 + chunk_len;
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_sysex_template_payload(const u8 *payload, u8 payload_len)
{
    u8 preset_index;
    u8 slot;
    u8 value_index;
    u16 total_len;
    u16 offset;
    u8 chunk_len;
    u8 i;
    u8 status;
    md400_sysex_template_t *template;

    if (!payload || (payload_len < 8)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_SYSEX_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    slot = payload[1] - 1;
    value_index = payload[2] & 0x7f;
    total_len = md400_get_u14(&payload[3]);
    offset = md400_get_u14(&payload[5]);
    chunk_len = payload[7] & 0x7f;

    if ((total_len > MD400_SYSEX_TEMPLATE_MAX_BODY) ||
        ((value_index != MD400_SYSEX_TEMPLATE_VALUE_NONE) &&
         (value_index >= total_len)) ||
        ((u32)offset + chunk_len > total_len) ||
        (payload_len < (8 + chunk_len))) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    for (i = 0; i < chunk_len; i++) {
        if (payload[8 + i] > 0x7f) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }
    if (!total_len && (offset || chunk_len ||
                       (value_index != MD400_SYSEX_TEMPLATE_VALUE_NONE))) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    if (!md400_sysex_template_load_to_buf(preset_index)) {
        return MD400_NB_STATUS_BUSY;
    }
    template = &md400_sysex_template_buf->slot[slot];
    if (!offset) {
        memset(template, 0, sizeof(*template));
        template->len = total_len & 0xff;
        template->value_index = value_index;
    } else if ((template->len != total_len) ||
               (template->value_index != value_index)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (chunk_len) {
        memcpy(&template->body[offset], &payload[8], chunk_len);
    }
    if (total_len < MD400_SYSEX_TEMPLATE_MAX_BODY) {
        memset(&template->body[total_len], 0,
               MD400_SYSEX_TEMPLATE_MAX_BODY - total_len);
    }

    status = md400_sysex_template_record_write(preset_index,
                                               md400_sysex_template_buf->slot);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }

    md400_log_event(0x06, MD400_NB_CMD_SET_SYSEX_TEMPLATE, payload[0],
                    total_len & 0x7f);
    printf("[MD400_CFG] set sysex template preset=%u slot=%u len=%u offset=%u chunk=%u\r\n",
           (unsigned)payload[0], (unsigned)payload[1],
           (unsigned)total_len, (unsigned)offset, (unsigned)chunk_len);
    return MD400_NB_STATUS_OK;
}

static void md400_nb_put_byte_nibbles(u8 *payload, u8 value)
{
    payload[0] = (value >> 4) & 0x0f;
    payload[1] = value & 0x0f;
}

static u8 md400_nb_get_byte_nibbles(const u8 *payload)
{
    return ((payload[0] & 0x0f) << 4) | (payload[1] & 0x0f);
}

static u8 md400_nb_custom_template_to_payload(const u8 *payload, u8 payload_len,
                                              u8 *reply, u8 *reply_len)
{
    u8 preset_index;
    u8 slot;
    u16 offset;
    u16 remain_len;
    u8 chunk_len;
    u8 i;
    md400_custom_template_t *template;

    if (!payload || !reply || !reply_len || (payload_len < 4)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_CUSTOM_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    slot = payload[1] - 1;
    offset = md400_get_u14(&payload[2]);
    if (!md400_custom_template_load_to_buf(preset_index)) {
        return MD400_NB_STATUS_BUSY;
    }
    template = &md400_custom_template_buf->slot[slot];
    if (offset > template->len) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    remain_len = template->len - offset;
    chunk_len = (remain_len > MD400_CUSTOM_TEMPLATE_CHUNK) ?
                MD400_CUSTOM_TEMPLATE_CHUNK : remain_len;
    reply[0] = payload[0];
    reply[1] = payload[1];
    reply[2] = template->value_index & 0x7f;
    md400_put_u14(&reply[3], template->len);
    md400_put_u14(&reply[5], offset);
    reply[7] = chunk_len & 0x7f;
    for (i = 0; i < chunk_len; i++) {
        md400_nb_put_byte_nibbles(&reply[8 + (i * 2)],
                                  template->body[offset + i]);
    }
    *reply_len = 8 + (chunk_len * 2);
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_custom_template_payload(const u8 *payload, u8 payload_len)
{
    u8 preset_index;
    u8 slot;
    u8 value_index;
    u16 total_len;
    u16 offset;
    u8 chunk_len;
    u8 i;
    u8 status;
    md400_custom_template_t *template;

    if (!payload || (payload_len < 8)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_CUSTOM_TEMPLATE_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset_index = payload[0] - 1;
    slot = payload[1] - 1;
    value_index = payload[2] & 0x7f;
    total_len = md400_get_u14(&payload[3]);
    offset = md400_get_u14(&payload[5]);
    chunk_len = payload[7] & 0x7f;

    if ((total_len > MD400_CUSTOM_TEMPLATE_MAX_BODY) ||
        ((value_index != MD400_CUSTOM_TEMPLATE_VALUE_NONE) &&
         (value_index >= total_len)) ||
        (chunk_len > MD400_CUSTOM_TEMPLATE_CHUNK) ||
        ((u32)offset + chunk_len > total_len) ||
        (payload_len < (8 + (chunk_len * 2)))) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    for (i = 0; i < (chunk_len * 2); i++) {
        if (payload[8 + i] > 0x0f) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }
    if (!total_len && (offset || chunk_len ||
                       (value_index != MD400_CUSTOM_TEMPLATE_VALUE_NONE))) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    if (!md400_custom_template_load_to_buf(preset_index)) {
        return MD400_NB_STATUS_BUSY;
    }
    template = &md400_custom_template_buf->slot[slot];
    if (!offset) {
        memset(template, 0, sizeof(*template));
        template->len = total_len & 0xff;
        template->value_index = value_index;
    } else if ((template->len != total_len) ||
               (template->value_index != value_index)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    for (i = 0; i < chunk_len; i++) {
        template->body[offset + i] =
            md400_nb_get_byte_nibbles(&payload[8 + (i * 2)]);
    }
    if (total_len < MD400_CUSTOM_TEMPLATE_MAX_BODY) {
        memset(&template->body[total_len], 0,
               MD400_CUSTOM_TEMPLATE_MAX_BODY - total_len);
    }

    status = md400_custom_template_record_write(preset_index,
                                                md400_custom_template_buf->slot);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }

    md400_log_event(0x06, MD400_NB_CMD_SET_CUSTOM_TEMPLATE, payload[0],
                    total_len & 0x7f);
    printf("[MD400_CFG] set custom template preset=%u slot=%u len=%u offset=%u chunk=%u\r\n",
           (unsigned)payload[0], (unsigned)payload[1],
           (unsigned)total_len, (unsigned)offset, (unsigned)chunk_len);
    return MD400_NB_STATUS_OK;
}

static void md400_nb_assignment_to_payload(u8 encoder, u8 *payload, u8 *payload_len)
{
    md400_assignment_t *assignment = md400_get_assignment(encoder);

    if (!assignment || !payload || !payload_len) {
        return;
    }

    payload[0] = encoder;
    md400_nb_assignment_fields_to_payload(assignment, &payload[1]);
    payload[10] = md400_encoder_runtime_value[encoder - 1] & 0x7f;
    *payload_len = 11;
}

static void md400_nb_gesture_assignment_to_payload(u8 event, u8 control, u8 *payload, u8 *payload_len)
{
    md400_assignment_t *assignment = md400_get_gesture_assignment(event, control);

    if (!assignment || !payload || !payload_len) {
        return;
    }

    payload[0] = event & 0x7f;
    payload[1] = control & 0x7f;
    md400_nb_assignment_fields_to_payload(assignment, &payload[2]);
    payload[11] = assignment->value_default & 0x7f;
    *payload_len = 12;
}

static void md400_nb_pedal_assignment_to_payload(u8 *payload, u8 *payload_len)
{
    md400_assignment_t *assignment = md400_get_pedal_assignment();

    if (!assignment || !payload || !payload_len) {
        return;
    }

    payload[0] = 1;
    md400_nb_assignment_fields_to_payload(assignment, &payload[1]);
    payload[10] = md400_pedal_runtime_value & 0x7f;
    *payload_len = 11;
}

static u8 md400_nb_apply_assignment_payload(const u8 *payload, u8 len, u8 preset_index)
{
    md400_assignment_t *assignment;
    u8 status;
    u8 encoder;

    status = md400_nb_validate_assignment_payload(payload, len);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }
    if (preset_index >= MD400_PRESET_COUNT) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    encoder = payload[0];
    assignment = &md400_store.preset[preset_index].encoder[encoder - 1];
    md400_nb_assignment_apply_fields(assignment, &payload[1]);

    if (preset_index == md400_active_preset) {
        md400_encoder_runtime_value[encoder - 1] = assignment->value_default;
        md400_ms32_set_encoder_value(encoder, assignment->value_default);
        md400_encoder_note_toggle_state[encoder - 1] = 0;
        if ((md400_play_last_action == MD400_PLAY_LABEL_ROTATE_ACTION) &&
            (md400_play_last_control == encoder)) {
            md400_play_show_assignment(assignment, MD400_PLAY_LABEL_ROTATE_ACTION,
                                       encoder, md400_encoder_runtime_value[encoder - 1]);
        }
        md400_ui_display_sync();
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
    printf("[MD400_CFG] set enc%u type=%u ch=%u data1=%u range=%u-%u outputs=%02x\r\n",
           (unsigned)encoder, (unsigned)assignment->type,
           (unsigned)(assignment->channel + 1), (unsigned)assignment->data1,
           (unsigned)assignment->value_min, (unsigned)assignment->value_max,
           (unsigned)md400_available_target_mask());
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_pedal_assignment_payload(const u8 *payload, u8 len, u8 preset_index)
{
    md400_assignment_t *assignment;
    u8 status;

    status = md400_nb_validate_assignment_fields(payload, len);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }
    if (preset_index >= MD400_PRESET_COUNT) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    assignment = &md400_store.preset[preset_index].pedal;
    md400_nb_assignment_apply_fields(assignment, payload);
    if (preset_index == md400_active_preset) {
        md400_pedal_runtime_value = assignment->value_default;
        md400_pedal_seen = 0;
        md400_ui_display_sync();
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
    printf("[MD400_CFG] set pedal type=%u ch=%u data1=%u range=%u-%u outputs=%02x\r\n",
           (unsigned)assignment->type, (unsigned)(assignment->channel + 1),
           (unsigned)assignment->data1, (unsigned)assignment->value_min,
           (unsigned)assignment->value_max,
           (unsigned)md400_available_target_mask());
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_gesture_assignment_payload(const u8 *payload, u8 len)
{
    md400_assignment_t *assignment;
    u8 status;
    u8 *state;

    status = md400_nb_validate_gesture_assignment_payload(payload, len);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }

    assignment = md400_get_gesture_assignment(payload[0], payload[1]);
    if (!assignment) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_nb_assignment_apply_fields(assignment, &payload[2]);
    state = md400_gesture_toggle_state_ptr(payload[0], payload[1]);
    if (state) {
        *state = 0;
    }
    state = md400_gesture_momentary_active_ptr(payload[0], payload[1]);
    if (state) {
        *state = 0;
    }
    md400_led_sync();

    if ((md400_play_last_action == payload[0]) &&
        (md400_play_last_control == payload[1])) {
        md400_play_show_assignment_output(assignment, payload[0], payload[1],
                                          assignment->value_default);
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
    printf("[MD400_CFG] set gesture event=%u control=%u type=%u ch=%u data1=%u value=%u outputs=%02x\r\n",
           (unsigned)payload[0], (unsigned)payload[1], (unsigned)assignment->type,
           (unsigned)(assignment->channel + 1), (unsigned)assignment->data1,
           (unsigned)assignment->value_default, (unsigned)md400_available_target_mask());
    return MD400_NB_STATUS_OK;
}

static md400_assignment_t *md400_nb_preset_assignment_ptr(u8 preset_index, u8 section, u8 key0, u8 key1)
{
    u8 index;
    u8 combo;

    if (preset_index >= MD400_PRESET_COUNT) {
        return NULL;
    }

    if (section == MD400_PRESET_SECTION_ENCODER) {
        if ((key0 < 1) || (key0 > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_store.preset[preset_index].encoder[key0 - 1];
    }

    if (section == MD400_PRESET_SECTION_PEDAL) {
        return &md400_store.preset[preset_index].pedal;
    }

    if (section != MD400_PRESET_SECTION_GESTURE) {
        return NULL;
    }

    index = md400_encoder_gesture_index(key0);
    if (index != 0xff) {
        if ((key1 < 1) || (key1 > MD400_ENCODER_COUNT)) {
            return NULL;
        }
        return &md400_store.preset[preset_index].encoder_gesture[key1 - 1][index];
    }

    index = md400_combo_gesture_index(key0);
    if (index != 0xff) {
        combo = md400_combo_index_from_mask(key1);
        if (combo >= MD400_COMBO_COUNT) {
            return NULL;
        }
        return &md400_store.preset[preset_index].combo_gesture[combo][index];
    }

    return NULL;
}

static u8 md400_nb_preset_item_to_payload(const u8 *payload, u8 payload_len,
                                          u8 *reply, u8 *reply_len)
{
    md400_assignment_t *assignment;
    u8 preset;
    u8 section;
    u8 key0;
    u8 key1 = 0;

    if (!payload || !reply || !reply_len || (payload_len < 3)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset = payload[0] - 1;
    section = payload[1];
    key0 = payload[2];
    if (section == MD400_PRESET_SECTION_GESTURE) {
        if (payload_len < 4) {
            return MD400_NB_STATUS_BAD_LENGTH;
        }
        key1 = payload[3];
    }
    if ((section == MD400_PRESET_SECTION_PEDAL) && (payload_len < 3)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    if (section == MD400_PRESET_SECTION_LED) {
        if (key0 != 1) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
        reply[0] = preset + 1;
        reply[1] = section & 0x7f;
        reply[2] = key0;
        reply[3] = md400_store.preset[preset].led_mode & 0x7f;
        reply[4] = md400_store.preset[preset].led_mask & 0x0f;
        *reply_len = 5;
        return MD400_NB_STATUS_OK;
    }

    assignment = md400_nb_preset_assignment_ptr(preset, section, key0, key1);
    if (!assignment) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    reply[0] = preset + 1;
    reply[1] = section & 0x7f;
    reply[2] = key0 & 0x7f;
    if (section == MD400_PRESET_SECTION_GESTURE) {
        reply[3] = key1 & 0x7f;
        md400_nb_assignment_fields_to_payload(assignment, &reply[4]);
        *reply_len = 13;
    } else {
        md400_nb_assignment_fields_to_payload(assignment, &reply[3]);
        *reply_len = 12;
    }
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_apply_preset_item_payload(const u8 *payload, u8 payload_len)
{
    md400_assignment_t *assignment;
    u8 status;
    u8 preset;
    u8 section;
    u8 key0;
    u8 key1 = 0;
    const u8 *fields;
    u8 *state;

    if (!payload || (payload_len < 3)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    preset = payload[0] - 1;
    section = payload[1];
    key0 = payload[2];
    if (section == MD400_PRESET_SECTION_GESTURE) {
        if (payload_len < 13) {
            return MD400_NB_STATUS_BAD_LENGTH;
        }
        key1 = payload[3];
        fields = &payload[4];
        status = md400_nb_validate_assignment_fields(fields, payload_len - 4);
    } else if (section == MD400_PRESET_SECTION_ENCODER) {
        if (payload_len < 12) {
            return MD400_NB_STATUS_BAD_LENGTH;
        }
        fields = &payload[3];
        status = md400_nb_validate_assignment_fields(fields, payload_len - 3);
    } else if (section == MD400_PRESET_SECTION_PEDAL) {
        if (payload_len < 12) {
            return MD400_NB_STATUS_BAD_LENGTH;
        }
        fields = &payload[3];
        status = md400_nb_validate_assignment_fields(fields, payload_len - 3);
    } else if (section == MD400_PRESET_SECTION_LED) {
        if (payload_len < 5) {
            return MD400_NB_STATUS_BAD_LENGTH;
        }
        if ((key0 != 1) || (payload[3] > MD400_LED_MODE_MANUAL) || (payload[4] > 0x0f)) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
        md400_store.preset[preset].led_mode = payload[3] & 0x7f;
        md400_store.preset[preset].led_mask = payload[4] & 0x0f;
        if (preset == md400_active_preset) {
            md400_ui_display_sync();
        }
        md400_store.checksum = md400_store_checksum(&md400_store);
        printf("[MD400_CFG] set preset=%u led mode=%u mask=%02x\r\n",
               (unsigned)(preset + 1), (unsigned)md400_store.preset[preset].led_mode,
               (unsigned)md400_store.preset[preset].led_mask);
        return MD400_NB_STATUS_OK;
    } else {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    if (status != MD400_NB_STATUS_OK) {
        return status;
    }

    assignment = md400_nb_preset_assignment_ptr(preset, section, key0, key1);
    if (!assignment) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_nb_assignment_apply_fields(assignment, fields);
    if (preset == md400_active_preset) {
        if (section == MD400_PRESET_SECTION_ENCODER) {
            md400_encoder_runtime_value[key0 - 1] = assignment->value_default;
            md400_ms32_set_encoder_value(key0, assignment->value_default);
            md400_encoder_note_toggle_state[key0 - 1] = 0;
            if ((md400_play_last_action == MD400_PLAY_LABEL_ROTATE_ACTION) &&
                (md400_play_last_control == key0)) {
                md400_play_show_assignment(assignment, MD400_PLAY_LABEL_ROTATE_ACTION,
                                           key0, md400_encoder_runtime_value[key0 - 1]);
            }
        } else if (section == MD400_PRESET_SECTION_PEDAL) {
            md400_pedal_runtime_value = assignment->value_default;
            md400_pedal_seen = 0;
        } else {
            state = md400_gesture_toggle_state_ptr(key0, key1);
            if (state) {
                *state = 0;
            }
            state = md400_gesture_momentary_active_ptr(key0, key1);
            if (state) {
                *state = 0;
            }
            if ((md400_play_last_action == key0) &&
                (md400_play_last_control == key1)) {
                md400_play_show_assignment_output(assignment, key0, key1,
                                                  assignment->value_default);
            }
        }
        md400_ui_display_sync();
    }

    md400_store.checksum = md400_store_checksum(&md400_store);
    printf("[MD400_CFG] set preset=%u section=%u key=%u/%u type=%u ch=%u data1=%u outputs=%02x\r\n",
           (unsigned)(preset + 1), (unsigned)section, (unsigned)key0, (unsigned)key1,
           (unsigned)assignment->type, (unsigned)(assignment->channel + 1),
           (unsigned)assignment->data1, (unsigned)md400_available_target_mask());
    return MD400_NB_STATUS_OK;
}

static void md400_nb_event_inject(const u8 *payload, u8 len)
{
    u8 event;
    u8 control;
    u8 repeat;
    u8 i;

    if (!payload || !len) {
        return;
    }

    event = payload[0];
    control = (len > 1) ? payload[1] : 0;
    repeat = (len > 2) ? payload[2] : 1;
    if (event == MD400_EVENT_PEDAL_VALUE) {
        md400_pedal_adc_event(control ? control : 4,
                              (repeat & 0x7f) << 1);
        return;
    }
    if (!repeat) {
        repeat = 1;
    }
    if (repeat > 16) {
        repeat = 16;
    }

    if ((event == MD400_EVENT_ENCODER_LEFT) ||
        (event == MD400_EVENT_ENCODER_RIGHT) ||
        (event == MD400_EVENT_ENCODER_PRESS) ||
        (event == MD400_EVENT_ENCODER_RELEASE)) {
        for (i = 0; i < repeat; i++) {
            md400_ms32_inject_encoder_event(control, event);
        }
    } else if ((event == MD400_EVENT_ENCODER_SHORT) ||
               (event == MD400_EVENT_ENCODER_LONG) ||
               (event == MD400_EVENT_ENCODER_DOUBLE) ||
               (event == MD400_EVENT_COMBO_PRESS) ||
               (event == MD400_EVENT_COMBO_SHORT) ||
               (event == MD400_EVENT_COMBO_LONG)) {
        for (i = 0; i < repeat; i++) {
            md400_emit_gesture(event, control, 0);
        }
    } else if ((event == MD400_EVENT_EDIT_SHORT) ||
               (event == MD400_EVENT_EDIT_LONG) ||
               (event == MD400_EVENT_EDIT_DOUBLE)) {
        for (i = 0; i < repeat; i++) {
            md400_edit_key_event(event);
        }
    }
}

void md400_control_inject_event(u8 event, u8 control, u8 repeat)
{
    u8 payload[3];

    payload[0] = event & 0x7f;
    payload[1] = control & 0x7f;
    payload[2] = repeat & 0x7f;

    md400_control_init();
    md400_nb_event_inject(payload, sizeof(payload));
}

static void md400_nb_get_event_log_payload(u8 *payload, u8 *payload_len)
{
    u8 available = (md400_event_log_total > 4) ? 4 : (u8)md400_event_log_total;
    u8 start;
    u8 i;
    u8 out = 0;

    payload[out++] = available;
    start = (md400_event_log_wp + 16 - available) & 0x0f;
    for (i = 0; i < available; i++) {
        md400_event_log_t *entry = &md400_event_log[(start + i) & 0x0f];
        payload[out++] = entry->source & 0x7f;
        payload[out++] = entry->event & 0x7f;
        payload[out++] = entry->control & 0x7f;
        payload[out++] = entry->value & 0x7f;
    }
    *payload_len = out;
}

u8 md400_runtime_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 6)) {
        return 0;
    }

    md400_control_init();
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_edit_normalize_subject_for_page();
    }
    payload[0] = md400_ui_mode;
    payload[1] = md400_active_preset + 1;
    payload[2] = md400_selected_preset + 1;
    payload[3] = md400_edit_page;
    payload[4] = md400_edit_target_encoder;
    payload[5] = md400_edit_subject;
    if (max_len >= 12) {
        payload[6] = md400_foot_mode;
        payload[7] = md400_pedal_active_preset + 1;
        payload[8] = md400_pedal_selected_preset + 1;
        payload[9] = md400_pedal_page + 1;
        payload[10] = md400_pedal_edit_slot;
        payload[11] = md400_pedal_edit_view;
        return 12;
    }
    return 6;
}

static u8 md400_nb_pedal_action_get(const u8 *payload, u8 payload_len,
                                    u8 *reply, u8 *reply_len)
{
    md400_pedal_action_t *action;
    md400_assignment_t assignment;
    u8 rotate = 0;

    if (!payload || !reply || !reply_len ||
        ((payload_len != 3) && (payload_len != 4))) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_PEDAL_PAGE_COUNT) ||
        (payload[2] < 1) || (payload[2] > MD400_PEDAL_SLOT_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    if (payload_len == 4) {
        if (payload[3] > 1) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
        rotate = payload[3];
    }
    action = md400_pedal_action_at(payload[0] - 1, payload[1] - 1,
                                   payload[2]);
    if (!action) {
        printf("[NEXFOOT_PEDAL] get missing preset=%u page=%u ft=%u loaded=%u dirty=%u\r\n",
               (unsigned)payload[0], (unsigned)payload[1], (unsigned)payload[2],
               (unsigned)md400_pedal_record_loaded,
               (unsigned)md400_pedal_record_dirty);
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    reply[0] = payload[0];
    reply[1] = payload[1];
    reply[2] = payload[2];
    md400_pedal_action_assignment_get(action, rotate, &assignment);
    md400_nb_assignment_fields_to_payload(&assignment, &reply[3]);
    reply[12] = payload[2] - 1; /* Fixed A/B/C/D palette index. */
    reply[13] = rotate;
    *reply_len = 14;
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_pedal_action_set(const u8 *payload, u8 payload_len)
{
    md400_pedal_action_t *action;
    md400_assignment_t assignment;
    u8 status;
    u8 rotate = 0;
    const u8 *fields;

    if (!payload || ((payload_len != 13) && (payload_len != 14))) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if (payload_len == 14) {
        if (payload[3] > 1) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
        rotate = payload[3];
        fields = &payload[4];
    } else {
        fields = &payload[3];
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_PEDAL_PAGE_COUNT) ||
        (payload[2] < 1) || (payload[2] > MD400_PEDAL_SLOT_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    status = md400_nb_validate_assignment_fields(fields, 9);
    if (status != MD400_NB_STATUS_OK) {
        return status;
    }
    action = md400_pedal_action_at(payload[0] - 1, payload[1] - 1,
                                   payload[2]);
    if (!action) {
        printf("[NEXFOOT_PEDAL] set missing preset=%u page=%u ft=%u loaded=%u dirty=%u\r\n",
               (unsigned)payload[0], (unsigned)payload[1], (unsigned)payload[2],
               (unsigned)md400_pedal_record_loaded,
               (unsigned)md400_pedal_record_dirty);
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    md400_nb_assignment_apply_fields(&assignment, fields);
    if (!rotate) {
        assignment.behavior = MD400_BEHAVIOR_ABSOLUTE;
    }
    assignment.target_mask = md400_available_target_mask();
    md400_pedal_action_assignment_set(action, rotate, &assignment);
    /* The trailing legacy color byte is reserved and ignored. Grid colors
     * belong to physical A/B/C/D, not to stored assignments. */
    md400_pedal_record_dirty = 1;
    md400_ui_display_sync();
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_pedal_label_get(const u8 *payload, u8 payload_len,
                                   u8 *reply, u8 *reply_len)
{
    md400_pedal_action_t *action;
    u8 len = MD400_PEDAL_LABEL_LEN;

    if (!payload || (payload_len != 3) || !reply || !reply_len) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_PEDAL_PAGE_COUNT) ||
        (payload[2] < 1) || (payload[2] > MD400_PEDAL_SLOT_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    action = md400_pedal_action_at(payload[0] - 1, payload[1] - 1,
                                   payload[2]);
    if (!action) {
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    while (len && !action->label[len - 1]) {
        len--;
    }
    reply[0] = payload[0];
    reply[1] = payload[1];
    reply[2] = payload[2];
    reply[3] = len;
    memcpy(&reply[4], action->label, len);
    *reply_len = 4 + len;
    return MD400_NB_STATUS_OK;
}

static u8 md400_nb_pedal_label_set(const u8 *payload, u8 payload_len)
{
    md400_pedal_action_t *action;
    u8 len;
    u8 i;

    if (!payload || (payload_len < 4)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    len = payload[3];
    if ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT) ||
        (payload[1] < 1) || (payload[1] > MD400_PEDAL_PAGE_COUNT) ||
        (payload[2] < 1) || (payload[2] > MD400_PEDAL_SLOT_COUNT) ||
        (len > MD400_PEDAL_LABEL_LEN) ||
        (payload_len != (u8)(4 + len))) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    for (i = 0; i < len; i++) {
        if ((payload[4 + i] < 0x20) || (payload[4 + i] > 0x7e)) {
            return MD400_NB_STATUS_BAD_VALUE;
        }
    }
    action = md400_pedal_action_at(payload[0] - 1, payload[1] - 1,
                                   payload[2]);
    if (!action) {
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    memset(action->label, 0, sizeof(action->label));
    memcpy(action->label, &payload[4], len);
    md400_pedal_record_dirty = 1;
    md400_ui_display_sync();
    return MD400_NB_STATUS_OK;
}

u8 md400_runtime_assignment_snapshot(u8 *payload, u8 max_len)
{
    md400_assignment_t *assignment;
    u8 offset = 1;
    u8 i;

    if (!payload || (max_len < 56)) {
        return 0;
    }

    md400_control_init();
    payload[0] = md400_active_preset + 1;
    for (i = 0; i < MD400_ENCODER_COUNT; i++) {
        assignment = &md400_store.preset[md400_active_preset].encoder[i];
        payload[offset++] = i + 1;
        md400_nb_assignment_fields_to_payload(assignment, &payload[offset]);
        offset += 9;
        payload[offset++] = md400_encoder_runtime_value[i] & 0x7f;
    }

    assignment = md400_get_pedal_assignment();
    payload[offset++] = 4;
    md400_nb_assignment_fields_to_payload(assignment, &payload[offset]);
    offset += 9;
    payload[offset++] = md400_pedal_runtime_value & 0x7f;

    return offset;
}

u8 md400_runtime_pedal_state_snapshot(u8 *payload, u8 max_len)
{
    u8 ms32_channel = 0;
    u8 raw8 = 0;
    u8 min8 = 0;
    u8 max8 = 0;
    u8 ack_mode = 0x7f;
    u8 ack_seq = 0;
    u32 count = 0;
    u32 ack_count = 0;

    if (!payload || (max_len < 20)) {
        return 0;
    }

    md400_ms32_get_adc_snapshot(&ms32_channel, &raw8, &count, &min8, &max8);
    md400_ms32_get_pedal_mode_ack_snapshot(&ack_mode, &ack_count, &ack_seq);
    payload[0] = 4;
    payload[1] = (md400_pedal_adc_value >> 1) & 0x7f;
    payload[2] = md400_pedal_runtime_value & 0x7f;
    payload[3] = md400_pedal_seen ? 1 : 0;
    payload[4] = md400_pedal_mode_load() ? 1 : 0;
    payload[5] = md400_pedal_switch_seen ? (md400_pedal_switch_state ? 1 : 0) : 0x7f;
    payload[6] = ms32_channel & 0x7f;
    payload[7] = (raw8 >> 1) & 0x7f;
    md400_put_u14(&payload[8], raw8);
    md400_put_u14(&payload[10], count & 0x3fff);
    md400_put_u14(&payload[12], min8);
    md400_put_u14(&payload[14], max8);
    payload[16] = ack_mode & 0x7f;
    payload[17] = ack_seq & 0x7f;
    md400_put_u14(&payload[18], ack_count & 0x3fff);
    return 20;
}

u8 md400_runtime_action_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 12)) {
        return 0;
    }

    md400_control_init();
    payload[0] = md400_active_preset + 1;
    payload[1] = md400_action_mask4(md400_encoder_note_toggle_state, MD400_ENCODER_COUNT);
    payload[2] = md400_encoder_gesture_toggle_mask(md400_encoder_gesture_index(MD400_EVENT_ENCODER_PRESS));
    payload[3] = md400_encoder_gesture_toggle_mask(md400_encoder_gesture_index(MD400_EVENT_ENCODER_SHORT));
    payload[4] = md400_encoder_gesture_toggle_mask(md400_encoder_gesture_index(MD400_EVENT_ENCODER_LONG));
    payload[5] = md400_encoder_gesture_toggle_mask(md400_encoder_gesture_index(MD400_EVENT_ENCODER_DOUBLE));
    payload[6] = md400_combo_gesture_toggle_mask(md400_combo_gesture_index(MD400_EVENT_COMBO_PRESS));
    payload[7] = md400_combo_gesture_toggle_mask(md400_combo_gesture_index(MD400_EVENT_COMBO_SHORT));
    payload[8] = md400_combo_gesture_toggle_mask(md400_combo_gesture_index(MD400_EVENT_COMBO_LONG));
    payload[9] = md400_action_mask4(md400_encoder_press_momentary_active, MD400_ENCODER_COUNT);
    payload[10] = md400_action_mask4(md400_encoder_long_momentary_active, MD400_ENCODER_COUNT);
    payload[11] = md400_action_mask4(md400_combo_long_momentary_active, MD400_COMBO_COUNT);
    return 12;
}

u8 md400_runtime_protocol_state_snapshot(u8 *payload, u8 max_len)
{
    u16 rx_valid;
    u16 reply;
    u16 ok;
    u16 bad_checksum;
    u16 bad_length;
    u16 bad_value;
    u16 unsupported;
    u16 busy;
    u16 flash_error;

    if (!payload || (max_len < 28)) {
        return 0;
    }

    local_irq_disable();
    rx_valid = md400_nb_rx_valid_count;
    reply = md400_nb_reply_count;
    ok = md400_nb_status_ok_count;
    bad_checksum = md400_nb_status_bad_checksum_count;
    bad_length = md400_nb_status_bad_length_count;
    bad_value = md400_nb_status_bad_value_count;
    unsupported = md400_nb_status_unsupported_count;
    busy = md400_nb_status_busy_count;
    flash_error = md400_nb_status_flash_error_count;
    local_irq_enable();

    payload[0] = 1;
    md400_put_u14(&payload[1], rx_valid & 0x3fff);
    md400_put_u14(&payload[3], reply & 0x3fff);
    md400_put_u14(&payload[5], ok & 0x3fff);
    md400_put_u14(&payload[7], bad_checksum & 0x3fff);
    md400_put_u14(&payload[9], bad_length & 0x3fff);
    md400_put_u14(&payload[11], bad_value & 0x3fff);
    md400_put_u14(&payload[13], unsupported & 0x3fff);
    md400_put_u14(&payload[15], busy & 0x3fff);
    md400_put_u14(&payload[17], flash_error & 0x3fff);
    payload[19] = md400_dbg_vm_which;
    md400_put_u14(&payload[20], md400_dbg_vm_id);
    md400_put_s14(&payload[22], md400_dbg_vm_write_ret);
    md400_put_s14(&payload[24], md400_dbg_vm_verify_ret);
    payload[26] = md400_dbg_vm_valid;
    payload[27] = md400_dbg_vm_match;
    return 28;
}

u8 md400_runtime_set_encoder_assignment(u8 encoder, const u8 *fields, u8 len)
{
    u8 payload[10];

    if (!fields || (len != 9)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if ((encoder < 1) || (encoder > MD400_ENCODER_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_control_init();
    payload[0] = encoder;
    memcpy(&payload[1], fields, 9);
    return md400_nb_apply_assignment_payload(payload, sizeof(payload), md400_active_preset);
}

u8 md400_runtime_set_pedal_assignment(const u8 *fields, u8 len)
{
    if (!fields || (len != 9)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    md400_control_init();
    return md400_nb_apply_pedal_assignment_payload(fields, len, md400_active_preset);
}

static u8 md400_runtime_gesture_slot_key(u8 index, u8 *event, u8 *control)
{
    static const u8 encoder_events[] = {
        MD400_EVENT_ENCODER_PRESS,
        MD400_EVENT_ENCODER_SHORT,
        MD400_EVENT_ENCODER_LONG,
        MD400_EVENT_ENCODER_DOUBLE,
    };
    static const u8 combo_events[] = {
        MD400_EVENT_COMBO_PRESS,
        MD400_EVENT_COMBO_SHORT,
        MD400_EVENT_COMBO_LONG,
    };
    static const u8 combo_controls[] = {0x03, 0x06, 0x0c};
    u8 encoder_total = MD400_ENCODER_COUNT * sizeof(encoder_events);
    u8 combo_index;

    if (!event || !control) {
        return 0;
    }

    if (index < encoder_total) {
        *event = encoder_events[index / MD400_ENCODER_COUNT];
        *control = (index % MD400_ENCODER_COUNT) + 1;
        return 1;
    }

    combo_index = index - encoder_total;
    if (combo_index < (MD400_COMBO_COUNT * sizeof(combo_events))) {
        *event = combo_events[combo_index / MD400_COMBO_COUNT];
        *control = combo_controls[combo_index % MD400_COMBO_COUNT];
        return 1;
    }

    return 0;
}

u8 md400_runtime_gesture_assignment_count(void)
{
    return (MD400_ENCODER_COUNT * 4) + (MD400_COMBO_COUNT * 3);
}

u8 md400_runtime_midi_out_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 10)) {
        return 0;
    }

#if CONFIG_MD400_MIDI_OUT_ENABLE
    payload[0] = 1;
    payload[1] = md400_midi_out_uart_bus ? 1 : 0;
    md400_put_u14(&payload[2], md400_midi_out_queued_bytes & 0x3fff);
    md400_put_u14(&payload[4], md400_midi_out_written_bytes & 0x3fff);
    md400_put_u14(&payload[6], md400_midi_out_dropped_bytes & 0x3fff);
    md400_put_u14(&payload[8], md400_midi_out_open_failures & 0x3fff);
#else
    memset(payload, 0, 10);
#endif
    return 10;
}

u8 md400_runtime_midi_out_trace_snapshot(u8 *payload, u8 max_len)
{
#if CONFIG_MD400_MIDI_OUT_ENABLE
    u8 len;

    if (!payload) {
        return 0;
    }
    local_irq_disable();
    len = md400_midi_out_trace_len;
    if (len > max_len) {
        len = max_len;
    }
    if (len) {
        memcpy(payload, md400_midi_out_trace, len);
    }
    local_irq_enable();
    return len;
#else
    (void)payload;
    (void)max_len;
    return 0;
#endif
}

u8 md400_runtime_gesture_assignment_snapshot(u8 index, u8 *payload, u8 max_len)
{
    u8 event;
    u8 control;
    u8 payload_len = 0;

    if (!payload || (max_len < 12) ||
        !md400_runtime_gesture_slot_key(index, &event, &control)) {
        return 0;
    }

    md400_control_init();
    md400_nb_gesture_assignment_to_payload(event, control, payload, &payload_len);
    return payload_len;
}

u8 md400_runtime_set_gesture_assignment(u8 event, u8 control, const u8 *fields, u8 len)
{
    u8 payload[11];

    if (!fields || (len != 9)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    md400_control_init();
    payload[0] = event & 0x7f;
    payload[1] = control & 0x7f;
    memcpy(&payload[2], fields, 9);
    return md400_nb_apply_gesture_assignment_payload(payload, sizeof(payload));
}

static u8 md400_runtime_save_active_preset_now(void)
{
    md400_control_init();
    if (md400_foot_mode == MD400_FOOT_MODE_PEDAL) {
        if (md400_pedal_bank_save() > 0) {
            md400_pedal_header_save();
            return MD400_NB_STATUS_OK;
        }
        return MD400_NB_STATUS_FLASH_ERROR;
    }
    return md400_nb_store_save_status();
}

u8 md400_runtime_save_active_preset(void)
{
    md400_control_init();
    return md400_persist_submit(MD400_PERSIST_MSG_RUNTIME_SAVE, 0, 0) ?
           MD400_NB_STATUS_BUSY : MD400_NB_STATUS_OK;
}

static u8 md400_runtime_load_preset_from_vm_now(u8 preset_number)
{
    u8 status;

    if ((preset_number < 1) || (preset_number > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }

    md400_control_init();
    md400_store_load();
    md400_preset_load_timer_cancel();
    md400_active_preset = preset_number - 1;
    md400_selected_preset = md400_active_preset;
    md400_store.active_preset = md400_active_preset;
    md400_sync_runtime_defaults();
    status = md400_nb_header_save_status();
    if (status != MD400_NB_STATUS_OK) {
        md400_store_load();
    }
    md400_ui_display_sync();
    return status;
}

u8 md400_runtime_load_preset_from_vm(u8 preset_number)
{
    if ((preset_number < 1) || (preset_number > MD400_PRESET_COUNT)) {
        return MD400_NB_STATUS_BAD_VALUE;
    }
    md400_control_init();
    return md400_persist_submit(MD400_PERSIST_MSG_RUNTIME_LOAD,
                                preset_number, 0) ?
           MD400_NB_STATUS_BUSY : MD400_NB_STATUS_OK;
}

static u8 md400_runtime_factory_reset_now(void)
{
    md400_control_init();
    if (md400_factory_reset_save() == sizeof(md400_store)) {
        return MD400_NB_STATUS_OK;
    }
    return MD400_NB_STATUS_FLASH_ERROR;
}

u8 md400_runtime_factory_reset(void)
{
    md400_control_init();
    return md400_persist_submit(MD400_PERSIST_MSG_RUNTIME_FACTORY, 0, 0) ?
           MD400_NB_STATUS_BUSY : MD400_NB_STATUS_OK;
}

int md400_persist_wdt_guard_clear_async(void)
{
    return md400_persist_submit(MD400_PERSIST_MSG_WDT_GUARD_CLEAR, 0, 0);
}

static void md400_persist_app_callback(void *priv)
{
    md400_persist_message_t msg;
    int save_ret;
    u8 status;
    u8 reason;

    (void)priv;
    while (1) {
        local_irq_disable();
        if (!md400_persist_queue_count) {
            md400_persist_callback_pending = 0;
            local_irq_enable();
            break;
        }
        msg = md400_persist_queue[md400_persist_queue_head];
        md400_persist_queue_head =
            (md400_persist_queue_head + 1) % MD400_PERSIST_QUEUE_DEPTH;
        md400_persist_queue_count--;
        local_irq_enable();

        md400_persist_in_task = 1;
        switch (msg.message) {
        case MD400_PERSIST_MSG_BLE_PAIR:
            trans_conn_pair_vm_do(&trans_pair_info, 1);
            break;

        case MD400_PERSIST_MSG_NB_NOTIFY:
            md400_nb_notify_pending = 0;
            md400_nb_notify_state();
            break;

        case MD400_PERSIST_MSG_NB_FRAME:
            md400_nb_process_request(msg.arg0);
            break;

        case MD400_PERSIST_MSG_NANOBRIDGE:
            md400_nb_store_job_run();
            break;

        case MD400_PERSIST_MSG_HEADER:
            md400_header_save_pending = 0;
            status = md400_nb_header_save_status();
            printf("[MD400_PERSIST] header done status=%u active=%u\r\n",
                   (unsigned)status, (unsigned)(md400_active_preset + 1));
            break;

        case MD400_PERSIST_MSG_STORE:
            reason = md400_store_save_reason;
            md400_store_save_pending = 0;
            md400_store_save_reason = 0;
            status = md400_nb_store_save_status();
            md400_ui_display_sync();
            printf("[MD400_PERSIST] store done reason=%u status=%u active=%u\r\n",
                   (unsigned)reason, (unsigned)status,
                   (unsigned)(md400_active_preset + 1));
            break;

        case MD400_PERSIST_MSG_PEDAL_SAVE:
            if (msg.arg0 && !md400_pedal_names_save()) break;
            save_ret = md400_pedal_bank_save();
            if (save_ret > 0) {
                md400_pedal_header_save_now();
            }
            printf("[MD400_PERSIST] pedal done ret=%d dirty=%u\r\n",
                   save_ret, (unsigned)md400_pedal_record_dirty);
            break;

        case MD400_PERSIST_MSG_FOOT_MODE:
            md400_foot_mode_set_now(msg.arg0);
            break;

        case MD400_PERSIST_MSG_BT_ENABLE:
            md400_bt_set_user_enable_now(msg.arg0);
            break;

        case MD400_PERSIST_MSG_PEDAL_MODE:
            md400_pedal_mode_set_now(msg.arg0);
            break;

        case MD400_PERSIST_MSG_PEDAL_CAL:
            md400_pedal_cal_store_range_now(msg.arg0, msg.arg1);
            break;

        case MD400_PERSIST_MSG_RUNTIME_SAVE:
            status = md400_runtime_save_active_preset_now();
            printf("[MD400_PERSIST] UART save done status=%u\r\n",
                   (unsigned)status);
            break;

        case MD400_PERSIST_MSG_RUNTIME_LOAD:
            status = md400_runtime_load_preset_from_vm_now(msg.arg0);
            printf("[MD400_PERSIST] UART load done preset=%u status=%u\r\n",
                   (unsigned)msg.arg0, (unsigned)status);
            break;

        case MD400_PERSIST_MSG_RUNTIME_FACTORY:
            status = md400_runtime_factory_reset_now();
            printf("[MD400_PERSIST] UART factory done status=%u\r\n",
                   (unsigned)status);
            break;

        case MD400_PERSIST_MSG_WDT_GUARD_CLEAR:
            {
                extern void md400_board_wdt_boot_guard_clear_now(void);
                md400_board_wdt_boot_guard_clear_now();
            }
            break;

        case MD400_PERSIST_MSG_OTA_BLE_ARM:
            md400_ota_ble_arm_persist();
            break;

        default:
            printf("[MD400_PERSIST] unknown msg=%u\r\n",
                   (unsigned)msg.message);
            break;
        }
        md400_persist_in_task = 0;
    }
}

u8 md400_runtime_get_preset_label(u8 preset_number, u8 *label, u8 *label_len, u8 max_len)
{
    u8 payload[2 + MD400_PRESET_LABEL_LEN];
    u8 payload_len = 0;
    u8 len;
    u8 status;

    if (!label || !label_len || (max_len < MD400_PRESET_LABEL_LEN)) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    md400_control_init();
    status = md400_nb_preset_label_to_payload(preset_number, payload, &payload_len);
    if (status != MD400_NB_STATUS_OK) {
        *label_len = 0;
        return status;
    }
    if (payload_len < 2) {
        *label_len = 0;
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    len = payload[1] & 0x7f;
    if (len > MD400_PRESET_LABEL_LEN) {
        len = MD400_PRESET_LABEL_LEN;
    }
    memset(label, 0, max_len);
    if (len) {
        memcpy(label, &payload[2], len);
    }
    *label_len = len;
    return MD400_NB_STATUS_OK;
}

u8 md400_runtime_set_preset_label(u8 preset_number, const u8 *label, u8 label_len)
{
    u8 payload[2 + MD400_PRESET_LABEL_LEN];

    if (!label && label_len) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }
    if (label_len > MD400_PRESET_LABEL_LEN) {
        return MD400_NB_STATUS_BAD_LENGTH;
    }

    md400_control_init();
    payload[0] = preset_number;
    payload[1] = label_len & 0x7f;
    if (label_len) {
        memcpy(&payload[2], label, label_len);
    }
    return md400_nb_apply_preset_label_payload(payload, 2 + label_len);
}

u8 md400_runtime_exit_edit(void)
{
    md400_control_init();
    if ((md400_foot_mode == MD400_FOOT_MODE_PEDAL) &&
        (md400_ui_mode == MD400_UI_MODE_EDIT)) {
        md400_ui_event(MD400_EVENT_EDIT_LONG);
        return MD400_NB_STATUS_OK;
    }
    md400_preset_load_timer_cancel();
    md400_pedal_cal_hide();
    md400_pedal_settings_enter_cancel();
    if (md400_ui_mode == MD400_UI_MODE_EDIT) {
        md400_edit_session_end(0);
    }
    md400_ui_mode = MD400_UI_MODE_PERFORM;
    md400_sync_ms32_encoder_runtime_values();
    md400_ui_display_sync();
    md400_log_event(0x02, MD400_EVENT_EDIT_LONG, 0, md400_ui_mode);
    return MD400_NB_STATUS_OK;
}

u8 md400_runtime_event_log_snapshot(u8 *payload, u8 max_len)
{
    u8 payload_len = 0;

    if (!payload || (max_len < 21)) {
        return 0;
    }

    md400_control_init();
    md400_nb_get_event_log_payload(payload, &payload_len);
    return payload_len;
}

static void md400_nb_get_edit_key_state_payload(u8 *payload, u8 *payload_len)
{
    payload[0] = md400_edit_key_timer ? 1 : 0;
    payload[1] = md400_edit_key_raw_pressed() ? 1 : 0;
    payload[2] = md400_edit_key_stable ? 1 : 0;
    payload[3] = md400_edit_key_last_raw ? 1 : 0;
    payload[4] = md400_edit_key_debounce_ticks & 0x7f;
    payload[5] = md400_edit_key_click_count & 0x7f;
    payload[6] = md400_edit_key_long_sent ? 1 : 0;
    md400_put_u14(&payload[7], md400_edit_key_press_ticks & 0x3fff);
    payload[9] = md400_edit_key_short_timer ? 1 : 0;
    md400_put_u14(&payload[10], md400_edit_key_short_count & 0x3fff);
    md400_put_u14(&payload[12], md400_edit_key_long_count & 0x3fff);
    md400_put_u14(&payload[14], md400_edit_key_double_count & 0x3fff);
    md400_put_u14(&payload[16], md400_edit_key_edge_count & 0x3fff);
    *payload_len = 18;
}

u8 md400_runtime_edit_key_state_snapshot(u8 *payload, u8 max_len)
{
    u8 payload_len = 0;

    if (!payload || (max_len < 18)) {
        return 0;
    }

    md400_control_init();
    md400_nb_get_edit_key_state_payload(payload, &payload_len);
    return payload_len;
}

static void md400_nb_get_ble_midi_state_payload(u8 *payload, u8 *payload_len,
                                                u8 max_len)
{
    u16 tx_used;
    u16 rx_used;
    u16 conn_handle;
    u8 ccc_enabled = 0;
    u8 server_ccc_enabled = 0;
    u8 client_ready = 0;
    u8 active_role = 0;
    u16 client_conn_handle = 0;
    u16 client_io_handle = 0;
    u16 client_write_handle = 0;
    u16 client_notify_handle = 0;
    u32 rx_packet;
    u32 rx_queued;
    u32 rx_dropped;
    u32 tx_queued;
    u32 tx_written;
    u32 tx_packet;
    u32 tx_dropped;
    u32 tx_failures;
    u32 tx_ccc_drops;
    u16 connect_count;
    u16 disconnect_count;

    if (trans_con_handle) {
        server_ccc_enabled = ble_gatt_server_characteristic_ccc_get(
            trans_con_handle,
            ATT_CHARACTERISTIC_BLE_MIDI_IO_01_CLIENT_CONFIGURATION_HANDLE) ? 1 : 0;
        ccc_enabled = server_ccc_enabled;
    }
#if CONFIG_BT_GATT_CLIENT_NUM
    trans_ble_midi_client_diag(&client_conn_handle, &client_io_handle,
                               &client_write_handle, &client_notify_handle);
    client_ready = trans_ble_midi_client_is_ready();
    if (!ccc_enabled && client_ready) {
        ccc_enabled = 1;
    }
#endif
    if (client_conn_handle) {
        active_role = 2;
    } else if (trans_con_handle) {
        active_role = 1;
    }

    local_irq_disable();
    tx_used = midi_ring_used(ble_midi_tx_buf_rp, ble_midi_tx_buf_wp,
                             BLE_MIDI_TX_BUFF_SIZE, ble_midi_tx_full);
    rx_used = midi_ring_used(ble_midi_rx_buf_rp, ble_midi_rx_buf_wp,
                             BLE_MIDI_RX_BUFF_SIZE, ble_midi_rx_full);
    conn_handle = trans_comm_connected_handle ? trans_comm_connected_handle : trans_con_handle;
    rx_packet = ble_midi_rx_packet_bytes;
    rx_queued = ble_midi_rx_queued_bytes;
    rx_dropped = ble_midi_rx_dropped_bytes;
    tx_queued = ble_midi_tx_queued_bytes;
    tx_written = ble_midi_tx_written_bytes;
    tx_packet = ble_midi_tx_packet_bytes;
    tx_dropped = ble_midi_tx_dropped_bytes;
    tx_failures = ble_midi_tx_send_failures;
    tx_ccc_drops = ble_midi_tx_ccc_disabled_drops;
    connect_count = ble_midi_connect_count;
    disconnect_count = ble_midi_disconnect_count;
    local_irq_enable();

    payload[0] = 1;
    payload[1] = conn_handle ? 1 : 0;
    payload[2] = ccc_enabled;
    md400_put_u14(&payload[3], conn_handle & 0x3fff);
    md400_put_u14(&payload[5], tx_used & 0x3fff);
    md400_put_u14(&payload[7], rx_used & 0x3fff);
    md400_put_u14(&payload[9], tx_queued & 0x3fff);
    md400_put_u14(&payload[11], tx_written & 0x3fff);
    md400_put_u14(&payload[13], tx_packet & 0x3fff);
    md400_put_u14(&payload[15], tx_dropped & 0x3fff);
    md400_put_u14(&payload[17], tx_failures & 0x3fff);
    md400_put_u14(&payload[19], tx_ccc_drops & 0x3fff);
    md400_put_u14(&payload[21], rx_packet & 0x3fff);
    md400_put_u14(&payload[23], rx_queued & 0x3fff);
    md400_put_u14(&payload[25], rx_dropped & 0x3fff);
    payload[27] = ((connect_count + disconnect_count) & 0x7f);
    *payload_len = 28;

    if (max_len >= 40) {
        payload[28] = trans_con_handle ? 1 : 0;
        payload[29] = server_ccc_enabled;
        payload[30] = client_ready;
        md400_put_u14(&payload[31], client_conn_handle & 0x3fff);
        md400_put_u14(&payload[33], client_io_handle & 0x3fff);
        md400_put_u14(&payload[35], client_write_handle & 0x3fff);
        md400_put_u14(&payload[37], client_notify_handle & 0x3fff);
        payload[39] = active_role;
        *payload_len = 40;
    }
}

u8 md400_runtime_usb_host_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 18)) {
        return 0;
    }

#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    payload[0] = 1;
    payload[1] = usb1_host_mounted ? 1 : 0;
    md400_put_u14(&payload[2], usb1_midi_tx_queued_bytes & 0x3fff);
    md400_put_u14(&payload[4], usb1_midi_tx_written_bytes & 0x3fff);
    md400_put_u14(&payload[6], usb1_midi_tx_packet_bytes & 0x3fff);
    md400_put_u14(&payload[8], usb1_midi_tx_dropped_bytes & 0x3fff);
    md400_put_u14(&payload[10], usb1_midi_rx_packet_bytes & 0x3fff);
    md400_put_u14(&payload[12], usb1_midi_rx_queued_bytes & 0x3fff);
    md400_put_u14(&payload[14], usb1_midi_rx_dropped_bytes & 0x3fff);
    md400_put_u14(&payload[16], usb1_midi_tx_send_failures & 0x3fff);
#else
    memset(payload, 0, 18);
#endif
    return 18;
}

u8 md400_runtime_led_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 8)) {
        return 0;
    }

    md400_control_init();
    md400_led_sync();
    payload[0] = md400_led_policy_mode & 0x7f;
    payload[1] = md400_led_policy_mask & 0x0f;
    payload[2] = md400_led_get_mask() & 0x0f;
    payload[3] = md400_button_down_mask & 0x0f;
    payload[4] = md400_edit_target_encoder & 0x7f;
    payload[5] = (md400_selected_preset % MD400_ENCODER_COUNT) + 1;
    payload[6] = md400_store.preset[md400_active_preset].led_mode & 0x7f;
    payload[7] = md400_store.preset[md400_active_preset].led_mask & 0x0f;
    return 8;
}

u8 md400_runtime_ble_midi_state_snapshot(u8 *payload, u8 max_len)
{
    u8 payload_len = 0;

    if (!payload || (max_len < 28)) {
        return 0;
    }

    md400_nb_get_ble_midi_state_payload(payload, &payload_len, max_len);
    return payload_len;
}

u8 md400_runtime_display_state_snapshot(u8 *payload, u8 max_len)
{
    if (!payload || (max_len < 17)) {
        return 0;
    }

    return md400_display_snapshot(payload, max_len);
}

/* ---------- Pure-BLE JL RCSP OTA boot ------------------------------------
 * The current product has one OTA owner: JL's BLE RCSP loader-download path.
 * NanoBridge command 0x51 or an exact GATT magic arms a one-shot VM flag on
 * app_core, then reboots into the trimmed BLE-OTA boot. The superseded USB
 * SysEx pull/dual-bank protocol was removed; wired recovery uses UBOOT detach. */
extern u32 pmalloc_get_unused_pages(void);

#define MD400_OTA_MODE_SWITCH_DELAY_MS  300   /* let the reply flush before reset */
#define MD400_OTA_WATCHDOG_SEC          60    /* auto-reboot to normal if OTA stalls */

static u8 md400_ota_mode;        /* 1 => trimmed boot with BLE RCSP enabled */
static volatile u16 md400_ota_watchdog_left;  /* seconds until watchdog reboot */

static const u8 md400_ota_flag_magic[4] = {'M', 'D', '4', 'O'};
#define MD400_OTA_BOOT_BLE   1
typedef struct {
    u8 magic[4];
    u8 pending;   /* 1 => next boot enters trimmed BLE OTA mode */
    u8 mode;      /* retained in the record schema; only BLE is accepted */
    u8 rsvd[2];
} md400_ota_boot_flag_t;

int md400_ota_boot_is_ble(void)
{
    return md400_ota_mode;
}

static void md400_ota_enter_mode_reset(void *priv)
{
    (void)priv;
    printf("[MD400_OTA] rebooting into minimal OTA mode\r\n");
    local_irq_disable();
    cpu_reset();
    while (1) {
    }
}

/* Called only by the app_core persistence owner. */
static int md400_ota_set_ble_boot_flag(void)
{
    md400_ota_boot_flag_t f;
    int ret;
    int verify;
    md400_ota_boot_flag_t rb;

    memcpy(f.magic, md400_ota_flag_magic, sizeof(f.magic));
    f.pending = 1;
    f.mode = MD400_OTA_BOOT_BLE;
    f.rsvd[0] = f.rsvd[1] = 0;
    ret = syscfg_write(MD400_VM_OTA_FLAG_ID, &f, sizeof(f));
    verify = syscfg_read(MD400_VM_OTA_FLAG_ID, &rb, sizeof(rb));
    if ((ret == (int)sizeof(f)) && (verify == (int)sizeof(rb)) && rb.pending &&
        rb.mode == MD400_OTA_BOOT_BLE &&
        !memcmp(rb.magic, md400_ota_flag_magic, sizeof(rb.magic))) {
        return 1;
    }
    printf("[MD400_OTA] set boot flag fail ret=%d verify=%d\r\n", ret, verify);
    return 0;
}

/* Watchdog: while stuck in the trimmed BLE-OTA mode, reboot back to the
 * normal firmware if no OTA traffic arrives for MD400_OTA_WATCHDOG_SEC. The flag
 * is already cleared at boot, so the reboot lands in the safe original bank. */
static void md400_ota_watchdog_tick(void *priv)
{
    (void)priv;
    if (md400_ota_watchdog_left && (--md400_ota_watchdog_left == 0)) {
        printf("[MD400_OTA] watchdog timeout in OTA mode -> reboot to normal\r\n");
        local_irq_disable();
        cpu_reset();
        while (1) {
        }
    }
}

static void md400_ota_watchdog_kick(void)
{
    md400_ota_watchdog_left = MD400_OTA_WATCHDOG_SEC;
}

/* Called once from spple_app_start(). Returns 1 for the trimmed BLE-OTA boot.
 * Clears the flag immediately so a
 * failed/aborted OTA falls back to a normal boot on the next reset. */
int md400_ota_boot_mode_check(void)
{
    md400_ota_boot_flag_t f;
    int ret = syscfg_read(MD400_VM_OTA_FLAG_ID, &f, sizeof(f));

    if ((ret == (int)sizeof(f)) &&
        !memcmp(f.magic, md400_ota_flag_magic, sizeof(f.magic)) && f.pending) {
        f.pending = 0;
        syscfg_write(MD400_VM_OTA_FLAG_ID, &f, sizeof(f));
        if (f.mode != MD400_OTA_BOOT_BLE) {
            printf("[MD400_OTA] obsolete boot mode=%u ignored\r\n",
                   (unsigned)f.mode);
            md400_ota_mode = 0;
            return 0;
        }
        md400_ota_mode = 1;
        md400_ota_watchdog_kick();
        sys_timer_add(NULL, md400_ota_watchdog_tick, 1000);
        printf("[MD400_OTA] boot in BLE OTA mode; watchdog=%us free_pages=%u\r\n",
               MD400_OTA_WATCHDOG_SEC, (unsigned)pmalloc_get_unused_pages());
        /* The RCSP loader and its RAM updater inherit this transient unlock. */
        {
            extern void norflash_set_write_protect(u8 cmd);
            norflash_set_write_protect(0);
        }
        printf("[MD400_OTA] norflash write-protect lifted for BLE OTA\r\n");
        return 1;
    }
    md400_ota_mode = 0;
    return 0;
}

/* Strong override of the weak le_gatt_server.c hook: only let the SDK run
 * rcsp_init() on connect in the BLE-OTA boot. In the normal boot, RCSP on top
 * of btstack + the full ATT buffers exhausts the heap and any BLE connection
 * dies in a "No enough physics memory" assert loop (hardware-verified). */
u8 app_rcsp_connect_allow(void)
{
    return md400_ota_mode;
}

static const u8 md400_ble_ota_arm_magic[9] = {'M', 'D', '4', 'O', 'T', 'A', 'B', 'L', 'E'};
static u8 md400_ble_ota_arm_pending;
static u8 md400_ble_ota_arm_reply;
static u8 md400_ble_ota_arm_seq;

static int md400_ota_ble_arm_request(u8 reply_required, u8 seq)
{
    if (md400_ble_ota_arm_pending) {
        return -1;
    }
    md400_ble_ota_arm_pending = 1;
    md400_ble_ota_arm_reply = reply_required;
    md400_ble_ota_arm_seq = seq;
    if (md400_persist_submit(MD400_PERSIST_MSG_OTA_BLE_ARM, 0, 0)) {
        md400_ble_ota_arm_pending = 0;
        md400_ble_ota_arm_reply = 0;
        return -1;
    }
    return 0;
}

static void md400_ota_ble_arm_persist(void)
{
    u8 reply = MD400_OTA_REPLY_MODE_SWITCH;
    u8 ok = md400_ota_set_ble_boot_flag();

    if (md400_ble_ota_arm_reply) {
        md400_nb_send_reply(MD400_NB_CMD_OTA_BEGIN_BLE,
                            md400_ble_ota_arm_seq,
                            ok ? MD400_NB_STATUS_OK : MD400_NB_STATUS_FLASH_ERROR,
                            ok ? &reply : NULL, ok ? 1 : 0);
    }
    md400_ble_ota_arm_pending = 0;
    md400_ble_ota_arm_reply = 0;
    if (ok) {
        printf("[MD400_OTA] BLE arm persisted on app_core\r\n");
        sys_timeout_add(NULL, md400_ota_enter_mode_reset,
                        MD400_OTA_MODE_SWITCH_DELAY_MS);
    }
}

static int md400_ble_ota_arm_check(const u8 *buf, u16 len)
{
    if (len != sizeof(md400_ble_ota_arm_magic) ||
        memcmp(buf, md400_ble_ota_arm_magic, sizeof(md400_ble_ota_arm_magic))) {
        return 0;
    }
    if (md400_ota_ble_arm_request(0, 0)) {
        printf("[MD400_OTA] BLE arm queue busy\r\n");
    }
    return 1;
}

static void md400_nb_state_payload(u8 *reply)
{
    reply[0] = md400_ui_mode;
    reply[1] = md400_active_preset + 1;
    reply[2] = md400_selected_preset + 1;
    reply[3] = md400_edit_page;
    reply[4] = md400_edit_target_encoder;
    reply[5] = md400_edit_subject;
    reply[6] = md400_foot_mode;
    reply[7] = md400_pedal_active_preset + 1;
    reply[8] = md400_pedal_selected_preset + 1;
    reply[9] = md400_pedal_page + 1;
    reply[10] = md400_pedal_edit_slot;
    reply[11] = md400_pedal_edit_view;
}

static void md400_nb_notify_state(void)
{
    static u32 last_check;
    static u8 last[14];
    static u16 epoch;
    u8 payload[15];
    u32 now = timer_get_ms();
    md400_nb_origin_t saved;
    if (now - last_check < 100) return;
    last_check = now;
    payload[0] = md400_nb_change_source;
    md400_put_u14(payload + 1, md400_nb_revision);
    md400_nb_state_payload(payload + 3);
    saved = md400_nb_origin;
    if (md400_nb_usb_subscribed && now - md400_nb_usb_last_activity >= 15000) {
        md400_nb_usb_subscribed = md400_nb_usb_notify_valid = 0;
    }
    if (md400_nb_usb_subscribed && (!md400_nb_usb_notify_valid ||
        memcmp(md400_nb_usb_last_state, payload + 1, 14))) {
        memset(&md400_nb_origin, 0, sizeof(md400_nb_origin));
        if (!md400_nb_send_reply(MD400_NB_EVENT_STATE, 0, 0, payload, sizeof(payload))) {
            memcpy(md400_nb_usb_last_state, payload + 1, 14);
            md400_nb_usb_notify_valid = 1;
        }
    }
    if (md400_nb_ble_editor && trans_con_handle &&
        now - md400_nb_ble_last_activity < 15000 &&
        (epoch != md400_nb_ble_epoch || memcmp(last, payload + 1, sizeof(last)))) {
        md400_nb_origin.source = 1;
        md400_nb_origin.handle = trans_con_handle;
        md400_nb_origin.epoch = md400_nb_ble_epoch;
        if (!md400_nb_send_reply(MD400_NB_EVENT_STATE, 0, 0, payload, sizeof(payload))) {
            memcpy(last, payload + 1, sizeof(last));
            epoch = md400_nb_ble_epoch;
        }
    }
    md400_nb_origin = saved;
}

/* One visible view, 112 seven-bit bytes: schema, state[12], revision[2],
 * preset label[8], four {assignment[9], color, label[10]}, expression[9].
 * With NanoBridge/BLE framing it fits one 125-byte ATT notification. */
static void md400_nb_snapshot(u8 command, u8 seq, const u8 *payload, u8 length)
{
    static u8 reply[MD400_NB_MAX_WIRE_REPLY]; /* app_core is the only writer */
    md400_assignment_t *assignment;
    md400_assignment_t pedal_assignment;
    md400_pedal_action_t *action;
    u8 bank, preset, page, rotate, ft, slot, status = 0;
    memset(reply, 0, sizeof(reply));
    if (command == MD400_NB_CMD_PRESET_LABELS) {
        if ((length != 2 && length != 3) || (length == 3 && payload[2] > 1) || !payload[0] || !payload[1] || payload[1] > 12 ||
            payload[0] + payload[1] - 1 > MD400_PRESET_COUNT) {
            md400_nb_send_reply(command, seq, MD400_NB_STATUS_BAD_VALUE, NULL, 0);
            return;
        }
        reply[0] = payload[0]; reply[1] = payload[1];
        for (ft = 0; ft < payload[1]; ++ft)
            memcpy(reply + 2 + ft * 8, length == 3 && payload[2] ?
                   md400_pedal_names.name[payload[0] - 1 + ft] :
                   md400_store.preset[payload[0] - 1 + ft].label, 8);
        md400_nb_send_reply(command, seq, 0, reply, 2 + payload[1] * 8);
        return;
    }
    if (length != 4 || payload[0] > 2 || payload[1] > MD400_PRESET_COUNT ||
        payload[2] > MD400_PEDAL_PAGE_COUNT || payload[3] > 1) {
        md400_nb_send_reply(command, seq, MD400_NB_STATUS_BAD_VALUE, NULL, 0);
        return;
    }
    bank = payload[0] == 2 ? md400_foot_mode : payload[0];
    preset = payload[1] ? payload[1] - 1 : bank ? md400_pedal_active_preset : md400_active_preset;
    page = payload[2] ? payload[2] - 1 : md400_pedal_page;
    rotate = payload[3];
    reply[0] = 1;
    md400_nb_state_payload(reply + 1);
    memcpy(reply + 15, bank ? md400_pedal_names.name[preset] : md400_store.preset[preset].label, 8);
    if (!bank) md400_play_label_source(preset, md400_play_label_work);
    for (ft = 1; ft <= 4; ++ft) {
        u8 *record = reply + 23 + (ft - 1) * 20;
        if (bank) {
            action = md400_pedal_action_at(preset, page, ft);
            if (!action) { status = MD400_NB_STATUS_FLASH_ERROR; break; }
            md400_pedal_action_assignment_get(action, rotate, &pedal_assignment);
            md400_nb_assignment_fields_to_payload(&pedal_assignment, record);
            record[9] = ft - 1; /* Fixed A/B/C/D palette index. */
            memcpy(record + 10, action->label, 10);
        } else {
            assignment = md400_nb_preset_assignment_ptr(preset,
                rotate ? MD400_PRESET_SECTION_ENCODER : MD400_PRESET_SECTION_GESTURE,
                rotate ? ft : MD400_EVENT_ENCODER_PRESS, rotate ? 0 : ft);
            if (!assignment) { status = MD400_NB_STATUS_BAD_VALUE; break; }
            md400_nb_assignment_fields_to_payload(assignment, record);
            slot = md400_play_label_slot(rotate ? 0 : MD400_EVENT_ENCODER_PRESS, ft);
            if (slot != 0xff) memcpy(record + 10, md400_play_label_work[slot], 10);
        }
    }
    md400_nb_assignment_fields_to_payload(&md400_store.preset[preset].pedal, reply + 103);
    md400_put_u14(reply + 13, md400_nb_revision);
    md400_nb_send_reply(command, seq, status, status ? NULL : reply, status ? 0 : sizeof(reply));
}

static void md400_nb_sysex_handle(const u8 *sysex, u16 len)
{
    const u8 *body;
    const u8 *payload;
    u16 body_len;
    u8 cmd;
    u8 seq;
    u8 payload_len;
    u8 reply[MD400_NB_MAX_REPLY_PAYLOAD];
    u8 reply_len = 0;
    u8 status = MD400_NB_STATUS_OK;
    u8 defer_reply = 0;

    if (!sysex || (len < 9) || (sysex[0] != 0xF0) || (sysex[len - 1] != 0xF7)) {
        return;
    }

    body = &sysex[1];
    body_len = len - 2;
    if ((body_len < 7) || (body[0] != 0x7D) || (body[1] != 'N') || (body[2] != 'B')) {
        return;
    }

    cmd = body[4] & 0x7f;
    seq = body[5] & 0x7f;
    if ((body[3] != 0x01) ||
        (md400_nb_checksum(&body[3], body_len - 4) != (body[body_len - 1] & 0x7f))) {
        md400_nb_send_reply(cmd, seq, MD400_NB_STATUS_BAD_CHECKSUM, NULL, 0);
        return;
    }

    if (md400_nb_origin.source == 1) {
        md400_nb_ble_editor = 1;
        md400_nb_ble_last_activity = timer_get_ms();
        if (cmd >= 0x50 && cmd != MD400_NB_CMD_RESET) {
            md400_nb_send_reply(cmd, seq, MD400_NB_STATUS_UNSUPPORTED, NULL, 0);
            return;
        }
    }
    if (!md400_nb_origin.source) md400_nb_usb_last_activity = timer_get_ms();
    md400_nb_count_inc(&md400_nb_rx_valid_count);
    if (!md400_ota_mode) {
        md400_control_init();
    }
    payload = &body[6];
    payload_len = body_len - 7;

    switch (cmd) {
    case MD400_NB_CMD_VIEW_SNAPSHOT:
    case MD400_NB_CMD_PRESET_LABELS:
        md400_nb_snapshot(cmd, seq, payload, payload_len);
        return;
    case MD400_NB_CMD_PING:
        reply[0] = 'P';
        reply[1] = 'O';
        reply[2] = 'N';
        reply[3] = 'G';
        reply_len = 4;
        break;

    case MD400_NB_CMD_GET_INFO:
        {
        extern u8 md400_board_wdt_boot_guard_count(void);
        extern u32 md400_board_boot_reset_source_get(void);
        u32 reset_source;

        reply[0] = MD400_ENCODER_COUNT;
        reply[1] = MD400_PRESET_COUNT;
        reply[2] = 0x01;
        reply[3] = (1 << MD400_MIDI_TYPE_CC) |
                   (1 << MD400_MIDI_TYPE_PC) |
                   (1 << MD400_MIDI_TYPE_NOTE) |
                   (1 << MD400_MIDI_TYPE_PITCH) |
                   (1 << MD400_MIDI_TYPE_PRESSURE) |
                   (1 << MD400_MIDI_TYPE_SYSEX) |
                   (1 << MD400_MIDI_TYPE_CUSTOM) |
                   (1 << MD400_MIDI_TYPE_OFF);
        reply[4] = md400_runtime_target_mask();
        reply[5] = md400_runtime_preset_store_version();
        reply[6] = MD400_FW_INFO_MAJOR;
        reply[7] = MD400_FW_INFO_MINOR;
        reply[8] = MD400_FW_INFO_PATCH;
        reply[9] = md400_runtime_feature_flags();
        reply[10] = md400_board_wdt_boot_guard_count() & 0x7f;
        reset_source = md400_board_boot_reset_source_get();
        reply[11] = reset_source & 0x7f;
        reply[12] = (reset_source >> 7) & 0x7f;
        reply[13] = (reset_source >> 14) & 0x7f;
        reply[14] = (reset_source >> 21) & 0x7f;
        reply[15] = 7; /* BLE editor, compact snapshot, state notifications */
        reply[16] = MD400_NB_MAX_WIRE_REPLY;
        {
            u32 pages = pmalloc_get_unused_pages();
            reply[17] = pages & 0x7f;
            reply[18] = (pages >> 7) & 0x7f;
        }
        reply_len = 19;
        break;
        }

    case MD400_NB_CMD_GET_STATE:
        md400_nb_state_payload(reply);
        reply_len = 12;
        break;

    case MD400_NB_CMD_GET_PRESET:
        if ((payload_len < 1) || (payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else if (payload_len > 1) {
            status = md400_nb_preset_item_to_payload(payload, payload_len, reply, &reply_len);
        } else {
            reply[0] = payload[0];
            reply[1] = (payload[0] == (md400_active_preset + 1));
            reply[2] = md400_store.checksum & 0x7f;
            reply[3] = md400_preset_group_active_slot[(payload[0] - 1) / MD400_PRESET_GROUP_SIZE] + 1;
            md400_put_u14(&reply[4], md400_preset_group_active_seq[(payload[0] - 1) / MD400_PRESET_GROUP_SIZE]);
            reply_len = 6;
        }
        break;

    case MD400_NB_CMD_SET_PRESET:
        if ((payload_len < 2) || (payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else if (payload[1] == MD400_PRESET_SECTION_ENCODER) {
            status = md400_nb_apply_preset_item_payload(payload, payload_len);
        } else if ((payload_len >= 12) && (payload[1] == MD400_PRESET_SECTION_PEDAL)) {
            status = md400_nb_apply_preset_item_payload(payload, payload_len);
        } else if ((payload_len >= 13) && (payload[1] == MD400_PRESET_SECTION_GESTURE)) {
            status = md400_nb_apply_preset_item_payload(payload, payload_len);
        } else if (payload[1] == MD400_PRESET_SECTION_LED) {
            status = md400_nb_apply_preset_item_payload(payload, payload_len);
        } else {
            status = md400_nb_apply_assignment_payload(&payload[1], payload_len - 1, payload[0] - 1);
        }
        if (!status && payload[1] <= MD400_PRESET_SECTION_LED &&
            payload_len >= (payload[1] == MD400_PRESET_SECTION_GESTURE ? 13 :
                            payload[1] == MD400_PRESET_SECTION_LED ? 5 : 12))
            status = md400_nb_preset_item_to_payload(payload, payload_len, reply, &reply_len);
        break;

    case 0x52: /* DAW_APP: query by host ID or install the complete record */
        status = md400_daw_app_command(payload, payload_len, reply, &reply_len);
        break;
    case MD400_NB_CMD_DAW_MODE:
        status = md400_daw_nb_mode(payload, payload_len, reply, &reply_len);
        break;
    case MD400_NB_CMD_DAW_UPDATE:
        status = md400_daw_nb_update(payload, payload_len, reply, &reply_len);
        if (!seq) {
            /* SEQ 0: fire and forget, keeps the reply path free for the DAW. */
            return;
        }
        break;
    case MD400_NB_CMD_USB_STATE_SUBSCRIBE:
        if (payload_len != 1 || payload[0] > 1) status = MD400_NB_STATUS_BAD_VALUE;
        else if (md400_nb_origin.source) status = MD400_NB_STATUS_UNSUPPORTED;
        else {
            md400_nb_usb_subscribed = payload[0];
            md400_nb_usb_notify_valid = 0;
        }
        break;
    case MD400_NB_CMD_SELECT_VIEW:
        if (payload_len != 3 || payload[0] > 1 || !payload[1] ||
            payload[1] > MD400_PRESET_COUNT ||
            (payload[0] ? (!payload[2] || payload[2] > MD400_PEDAL_PAGE_COUNT) : payload[2] != 0)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
            defer_reply = status == MD400_NB_STATUS_OK;
        }
        break;
    case MD400_NB_CMD_GET_PEDAL_PRESET_NAME:
    case MD400_NB_CMD_SET_PEDAL_PRESET_NAME: {
        u8 n, i;
        if (!payload_len || !payload[0] || payload[0] > MD400_PRESET_COUNT) {
            status = MD400_NB_STATUS_BAD_VALUE; break;
        }
        if (cmd == MD400_NB_CMD_SET_PEDAL_PRESET_NAME) {
            if (payload_len < 2 || payload[1] > MD400_PRESET_LABEL_LEN || payload_len != payload[1] + 2) {
                status = MD400_NB_STATUS_BAD_LENGTH; break;
            }
            for (i = 0; i < payload[1]; i++) {
                if (payload[i + 2] < 0x20 || payload[i + 2] > 0x7e) break;
            }
            if (i != payload[1]) { status = MD400_NB_STATUS_BAD_VALUE; break; }
            memset(md400_pedal_names.name[payload[0] - 1], 0, MD400_PRESET_LABEL_LEN);
            memcpy(md400_pedal_names.name[payload[0] - 1], payload + 2, payload[1]);
            md400_pedal_names_dirty = 1;
        } else if (payload_len != 1) { status = MD400_NB_STATUS_BAD_LENGTH; break; }
        for (n = 0; n < MD400_PRESET_LABEL_LEN && md400_pedal_names.name[payload[0] - 1][n]; n++);
        reply[0] = payload[0]; reply[1] = n;
        memcpy(reply + 2, md400_pedal_names.name[payload[0] - 1], n);
        reply_len = n + 2;
        break;
    }

    case MD400_NB_CMD_SAVE_PRESET:
        if ((payload_len > 1) ||
            (payload_len && ((payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)))) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
            defer_reply = (status == MD400_NB_STATUS_OK);
        }
        break;

    case MD400_NB_CMD_LOAD_PRESET:
        if ((payload_len < 1) || (payload[0] < 1) || (payload[0] > MD400_PRESET_COUNT)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
            defer_reply = (status == MD400_NB_STATUS_OK);
        }
        break;

    case MD400_NB_CMD_GET_PRESET_LABEL:
        if (payload_len < 1) {
            status = MD400_NB_STATUS_BAD_LENGTH;
        } else {
            status = md400_nb_preset_label_to_payload(payload[0], reply, &reply_len);
        }
        break;

    case MD400_NB_CMD_SET_PRESET_LABEL:
        status = md400_nb_apply_preset_label_payload(payload, payload_len);
        break;

    case MD400_NB_CMD_FACTORY_RESET:
        if (!md400_nb_factory_reset_payload_valid(payload, payload_len)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
            defer_reply = (status == MD400_NB_STATUS_OK);
        }
        break;

    case MD400_NB_CMD_GET_ASSIGNMENT:
        if ((payload_len < 1) || (payload[0] < 1) || (payload[0] > MD400_ENCODER_COUNT)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            md400_nb_assignment_to_payload(payload[0], reply, &reply_len);
        }
        break;

    case MD400_NB_CMD_SET_ASSIGNMENT:
        status = md400_nb_apply_assignment_payload(payload, payload_len, md400_active_preset);
        break;

    case MD400_NB_CMD_GET_GESTURE_ASSIGNMENT:
        if ((payload_len < 2) || !md400_get_gesture_assignment(payload[0], payload[1])) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            md400_nb_gesture_assignment_to_payload(payload[0], payload[1], reply, &reply_len);
        }
        break;

    case MD400_NB_CMD_SET_GESTURE_ASSIGNMENT:
        status = md400_nb_apply_gesture_assignment_payload(payload, payload_len);
        break;

    case MD400_NB_CMD_GET_PEDAL_ASSIGNMENT:
        md400_nb_pedal_assignment_to_payload(reply, &reply_len);
        break;

    case MD400_NB_CMD_SET_PEDAL_ASSIGNMENT:
        status = md400_nb_apply_pedal_assignment_payload(payload, payload_len, md400_active_preset);
        break;

    case MD400_NB_CMD_GET_PEDAL_STATE:
        reply_len = md400_runtime_pedal_state_snapshot(reply, sizeof(reply));
        if (!reply_len) {
            status = MD400_NB_STATUS_BUSY;
        }
        break;

    case MD400_NB_CMD_GET_MIDI_OUT_STATE:
        reply_len = md400_runtime_midi_out_state_snapshot(reply, 10);
        break;

    case MD400_NB_CMD_GET_USB_HOST_STATE:
        reply_len = md400_runtime_usb_host_state_snapshot(reply, 18);
        break;

    case MD400_NB_CMD_GET_LED_STATE:
        reply_len = md400_runtime_led_state_snapshot(reply, 8);
        break;

    case MD400_NB_CMD_GET_EDIT_KEY_STATE:
        md400_nb_get_edit_key_state_payload(reply, &reply_len);
        break;

    case MD400_NB_CMD_GET_BLE_MIDI_STATE:
        reply_len = md400_runtime_ble_midi_state_snapshot(reply, sizeof(reply));
        break;

    case MD400_NB_CMD_GET_DISPLAY_STATE:
        reply_len = md400_runtime_display_state_snapshot(reply, sizeof(reply));
        if (!reply_len) {
            status = MD400_NB_STATUS_BUSY;
        }
        break;

    case MD400_NB_CMD_EVENT_INJECT:
        if (!payload_len) {
            status = MD400_NB_STATUS_BAD_LENGTH;
        } else {
            md400_nb_event_inject(payload, payload_len);
        }
        break;

    case MD400_NB_CMD_GET_EVENT_LOG:
        md400_nb_get_event_log_payload(reply, &reply_len);
        break;

    case MD400_NB_CMD_GET_ACTION_STATE:
        reply_len = md400_runtime_action_state_snapshot(reply, sizeof(reply));
        break;

    case MD400_NB_CMD_GET_PROTOCOL_STATE:
        reply_len = md400_runtime_protocol_state_snapshot(reply, sizeof(reply));
        break;

    case MD400_NB_CMD_GET_PLAY_LABEL:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_SET_PLAY_LABEL:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_GET_SYSEX_TEMPLATE:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_SET_SYSEX_TEMPLATE:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_GET_CUSTOM_TEMPLATE:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_SET_CUSTOM_TEMPLATE:
        status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
        defer_reply = (status == MD400_NB_STATUS_OK);
        break;

    case MD400_NB_CMD_GET_FOOT_MODE:
        reply[0] = md400_foot_mode;
        reply[1] = md400_active_preset + 1;
        reply[2] = md400_pedal_active_preset + 1;
        reply[3] = md400_pedal_page + 1;
        reply[4] = md400_pedal_edit_slot;
        reply[5] = md400_pedal_edit_view;
        reply[6] = md400_pedal_record_dirty || md400_pedal_names_dirty || md400_pedal_exp_dirty;
        reply_len = 7;
        break;

    case MD400_NB_CMD_SET_FOOT_MODE:
        if ((payload_len != 1) ||
            (payload[0] > MD400_FOOT_MODE_PEDAL)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            status = md400_nb_store_job_start(cmd, seq, payload, payload_len);
            defer_reply = (status == MD400_NB_STATUS_OK);
        }
        break;

    case MD400_NB_CMD_GET_PEDAL_ACTION:
        status = md400_nb_pedal_action_get(payload, payload_len,
                                           reply, &reply_len);
        break;

    case MD400_NB_CMD_SET_PEDAL_ACTION:
        status = md400_nb_pedal_action_set(payload, payload_len);
        if (!status) status = md400_nb_pedal_action_get(payload, payload_len == 14 ? 4 : 3, reply, &reply_len);
        break;

    case MD400_NB_CMD_GET_PEDAL_LABEL:
        status = md400_nb_pedal_label_get(payload, payload_len,
                                          reply, &reply_len);
        break;

    case MD400_NB_CMD_SET_PEDAL_LABEL:
        status = md400_nb_pedal_label_set(payload, payload_len);
        if (!status) status = md400_nb_pedal_label_get(payload, 3, reply, &reply_len);
        break;

    case MD400_NB_CMD_ENTER_EDIT:
        if (md400_ui_mode != MD400_UI_MODE_EDIT) {
            /* An editor takes over; DAW mode resumes after edit exits. */
            md400_daw_leave();
            md400_ui_event(MD400_EVENT_EDIT_LONG);
        }
        break;

    case MD400_NB_CMD_EXIT_EDIT:
        if (md400_ui_mode == MD400_UI_MODE_EDIT) {
            md400_ui_event(MD400_EVENT_EDIT_LONG);
        }
        break;

    case MD400_NB_CMD_OTA_BEGIN_BLE:
        printf("[MD400_OTA] begin BLE ota_mode=%u\r\n", md400_ota_mode);
        if (!md400_ota_mode) {
            /* The persistence owner sends the reply after write+readback, then
             * schedules reset so no VM erase/defrag runs in this USB callback. */
            if (md400_ota_ble_arm_request(1, seq)) {
                status = MD400_NB_STATUS_BUSY;
            } else {
                defer_reply = 1;
            }
            break;
        }
        md400_ota_watchdog_kick();
        reply[0] = MD400_OTA_REPLY_READY;
        reply_len = 1;
        break;

#ifdef NEXFOOT_WEB_BOOT
    case MD400_NB_CMD_DETACH_WEB:
        if (payload_len != 0) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else if (!sys_timeout_add(NULL, md400_nb_web_update_timeout, MD400_RESET_DELAY_MS)) {
            status = MD400_NB_STATUS_BUSY;
        }
        break;
#endif

    case MD400_NB_CMD_DETACH_UPDATE:
        if (!md400_usb_midi_mask_update_schedule()) {
            status = MD400_NB_STATUS_BUSY;
        }
        break;

    case MD400_NB_CMD_WDT_TEST:
#if CONFIG_MD400_UART0_RX_TEST_ENABLE
        if ((payload_len != 3) || memcmp(payload, "WDT", 3)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            sys_timeout_add(NULL, md400_nb_wdt_test_timeout,
                            MD400_RESET_DELAY_MS);
        }
#else
        status = MD400_NB_STATUS_UNSUPPORTED;
#endif
        break;

    case MD400_NB_CMD_RESET:
        if (!md400_nb_reset_payload_valid(payload, payload_len)) {
            status = MD400_NB_STATUS_BAD_VALUE;
        } else {
            sys_timeout_add(NULL, md400_nb_reset_timeout, MD400_RESET_DELAY_MS);
        }
        break;

    default:
        status = MD400_NB_STATUS_UNSUPPORTED;
        break;
    }

    if (!defer_reply) {
        if (!status && md400_nb_changes_state(cmd)) md400_nb_note_change(md400_nb_origin.source);
        md400_nb_send_reply(cmd, seq, status, reply, reply_len);
    }
}

static void md400_usb_mask_update_start(void *priv)
{
    extern void md400_board_prepare_mask_usb_update(void);
    extern void md400_board_wdt_uboot_handoff_commit(void);
    extern void nvram_set_boot_state(u32 state);

    (void)priv;
    md400_usb_mask_update_pending = 0;
    printf("[MD400_OTA] app_core direct UBOOT handoff\r\n");
    md400_board_wdt_uboot_handoff_commit();
    md400_board_prepare_mask_usb_update();
    local_irq_disable();
    nvram_set_boot_state(MD400_UPGRADE_USB_SOFT_KEY);
    printf("[MD400_OTA] reset with USB soft key=%u\r\n",
           (unsigned)MD400_UPGRADE_USB_SOFT_KEY);
    cpu_reset();
    while (1) {
    }
}

static void md400_usb_mask_update_post(void *priv)
{
    int argv[3];
    int ret;

    (void)priv;
    md400_usb_mask_update_timer = 0;
    argv[0] = (int)md400_usb_mask_update_start;
    argv[1] = 1;
    argv[2] = 0;
    ret = os_taskq_post_type("app_core", Q_CALLBACK, ARRAY_SIZE(argv), argv);
    if (ret) {
        printf("[MD400_OTA] app_core post failed ret=%d\r\n", ret);
        md400_usb_mask_update_timer =
            sys_timeout_add(NULL, md400_usb_mask_update_post, 20);
        if (!md400_usb_mask_update_timer) {
            md400_usb_mask_update_pending = 0;
            printf("[MD400_OTA] app_core retry schedule failed\r\n");
        }
    }
}

static u8 md400_usb_midi_mask_update_schedule_delay(u16 delay_ms)
{
    if (!md400_usb_mask_update_pending) {
        md400_usb_mask_update_pending = 1;
        md400_usb_mask_update_timer =
            sys_timeout_add(NULL, md400_usb_mask_update_post, delay_ms);
        if (!md400_usb_mask_update_timer) {
            md400_usb_mask_update_pending = 0;
            printf("[MD400_OTA] detach schedule failed\r\n");
            return 0;
        }
        return 1;
    }
    return 0;
}

static u8 md400_usb_midi_mask_update_schedule(void)
{
    return md400_usb_midi_mask_update_schedule_delay(200);
}

u8 md400_usb_mask_update_schedule(void)
{
    return md400_usb_midi_mask_update_schedule();
}

u8 md400_usb_mask_update_schedule_fast(void)
{
    return md400_usb_midi_mask_update_schedule_delay(1);
}

static void md400_detach_sysex_handle(const u8 *sysex, u16 len)
{
    const u8 *body;
    u16 body_len;

    if (!sysex || (len < (sizeof(md400_detach_magic) + 2)) ||
        (sysex[0] != 0xF0) || (sysex[len - 1] != 0xF7)) {
        return;
    }

    body = &sysex[1];
    body_len = len - 2;

    if ((body_len < sizeof(md400_detach_magic)) ||
        memcmp(body, md400_detach_magic, sizeof(md400_detach_magic))) {
        return;
    }

    if ((body_len == sizeof(md400_detach_magic)) ||
        ((body[6] == 'M') && (body_len == (sizeof(md400_detach_magic) + 1)))) {
        if (md400_usb_midi_mask_update_schedule()) {
            printf("[MD400_OTA] JLOTA mask detach\r\n");
        }
        return;
    }
}

static u8 md400_nb_is_control_sysex(const u8 *sysex, u16 len)
{
    const u8 *body;
    u16 body_len;

    if (!sysex || (len < 3) || (sysex[0] != 0xF0) || (sysex[len - 1] != 0xF7)) {
        return 0;
    }

    body = &sysex[1];
    body_len = len - 2;
    if ((body_len >= 3) && (body[0] == 0x7D) && (body[1] == 'N') && (body[2] == 'B')) {
        return 1;
    }
    if ((body_len >= sizeof(md400_detach_magic)) &&
        !memcmp(body, md400_detach_magic, sizeof(md400_detach_magic))) {
        return 1;
    }
    return 0;
}

static void md400_nb_usb_host_forward(const u8 *data, u16 len)
{
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
    if (data && len) {
        usb1_midi_tx_write_drop_on_full(data, len);
    }
#else
    (void)data;
    (void)len;
#endif
}

void md400_usb_midi_control_rx(u8 cin, const u8 *payload, u8 len)
{
    static u8 sysex_buf[MD400_DETACH_SYSEX_MAX];
    static u16 sysex_len;
    static u8 sysex_drop;
    static u8 in_sysex;
    u8 i;

    if (!payload || !len) {
        return;
    }

    /*
     * Cable 0 is NanoBridge. Our editor SysEx and JLOTA stay local. Any other
     * MIDI on this cable is a thru to USB-A host, not NanoControl or BLE.
     */
    if ((cin == 0x2) || (cin == 0x3) || (cin == 0xF) ||
        ((cin >= 0x8) && (cin <= 0xE))) {
        md400_nb_usb_host_forward(payload, len);
        return;
    }
    if ((cin == 0x5) && !in_sysex && (payload[0] != 0xF0) && (payload[0] != 0xF7)) {
        md400_nb_usb_host_forward(payload, len);
        return;
    }

    for (i = 0; i < len; i++) {
        u8 b = payload[i];

        if (b == 0xF0) {
            sysex_len = 0;
            sysex_drop = 0;
            in_sysex = 1;
        }

        if (!in_sysex) {
            md400_nb_usb_host_forward(&b, 1);
            continue;
        }

        if (!sysex_drop && (sysex_len < sizeof(sysex_buf))) {
            sysex_buf[sysex_len++] = b;
        } else {
            sysex_drop = 1;
        }

        if (b == 0xF7) {
            if (!sysex_drop && md400_nb_is_control_sysex(sysex_buf, sysex_len)) {
                md400_nb_origin_t usb_origin = {0};

                md400_nb_submit(usb_origin, sysex_buf, sysex_len);
                md400_detach_sysex_handle(sysex_buf, sysex_len);
            } else if (!sysex_drop) {
                md400_nb_usb_host_forward(sysex_buf, sysex_len);
            }
            sysex_len = 0;
            sysex_drop = 0;
            in_sysex = 0;
        }
    }
}

static u8 usb_midi_msg_total_len(u8 status)
{
    if ((status >= 0x80) && (status <= 0xEF)) {
        return (((status & 0xF0) == 0xC0) || ((status & 0xF0) == 0xD0)) ? 2 : 3;
    }
    switch (status) {
    case 0xF1:
    case 0xF3:
        return 2;
    case 0xF2:
        return 3;
    case 0xF6:
    case 0xF7:
    case 0xF8:
    case 0xFA:
    case 0xFB:
    case 0xFC:
    case 0xFE:
    case 0xFF:
        return 1;
    default:
        return 1;
    }
}

static u8 usb_midi_cin_from_status(u8 status)
{
    if ((status >= 0x80) && (status <= 0xEF)) {
        return (status >> 4) & 0x0F;
    }
    switch (status) {
    case 0xF1:
    case 0xF3:
        return 0x2;
    case 0xF2:
        return 0x3;
    case 0xF6:
    case 0xF7:
        return 0x5;
    default:
        if (status >= 0xF8) {
            return 0xF;
        }
        return 0xF;
    }
}

void ac632n_usb_midi_rx(const u8 *packet, u32 len)
{
    static const u8 cin_len[16] = {0, 1, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1};
    u8 raw[64];
    u16 raw_len = 0;
    u32 i;

    if (!packet || (len < 4)) {
        return;
    }

    for (i = 0; (i + 3) < len; i += 4) {
        u8 cin = packet[i] & 0x0F;
        u8 cable = (packet[i] >> 4) & 0x0F;
        u8 n = cin_len[cin];
        u8 k;

        if (!n) {
            continue;
        }

        if (cable == USB_MIDI_CABLE_CONTROL) {
            md400_usb_midi_control_rx(cin, &packet[i + 1], n);
            continue;
        }

        if (cable != USB_MIDI_CABLE_PORT) {
            continue;
        }

        if ((raw_len + n) > sizeof(raw)) {
            break;
        }
        for (k = 0; k < n; k++) {
            raw[raw_len++] = packet[i + 1 + k];
        }
    }

    if (raw_len) {
        usb_midi_rx_write_drop_on_full(raw, raw_len);
    }
}

u32 ac632n_usb_midi_tx(u8 *packet, u32 max_len)
{
    // 修改日期: 2026-02-16, 修改说明: 增加USB MIDI编码/发送前关键打印, 排查USB TX缓存有数据但不出包
    static u32 usb_midi_tx_dbg_cnt;
    u16 out_len = 0;
    u16 consumed = 0;
    u8 raw[96];
    u16 raw_avail;

    if (!packet || !max_len) {
        return 0;
    }

    local_irq_disable();
    raw_avail = midi_ring_peek_copy(raw, sizeof(raw), usb_midi_tx_buf, usb_midi_tx_buf_rp, usb_midi_tx_buf_wp, USB_MIDI_TX_BUFF_SIZE, usb_midi_tx_full);
    local_irq_enable();

    if (0) { // 修改日期: 2026-02-17, 修改说明: 屏蔽[USB_MIDI]编码打印
        if (raw_avail) {
            usb_midi_tx_dbg_cnt++;
            if (usb_midi_tx_dbg_cnt <= 20) {
                printf("[USB_MIDI] tx_encode#%lu raw_avail=%u max=%lu b0=%02x b1=%02x b2=%02x\r\n",
                       (unsigned long)usb_midi_tx_dbg_cnt, (unsigned)raw_avail, (unsigned long)max_len,
                       (unsigned)raw[0], (unsigned)(raw_avail > 1 ? raw[1] : 0), (unsigned)(raw_avail > 2 ? raw[2] : 0));
            } else if (usb_midi_tx_dbg_cnt == 21) {
                printf("[USB_MIDI] tx_encode log muted\r\n");
            }
        }
    }

    while ((out_len + 4) <= max_len) {
        u16 remain = raw_avail - consumed;
        u8 b;

        if (!remain) {
            break;
        }

        if (usb_midi_tx_sysex_active) {
            u16 take = (remain >= 3) ? 3 : remain;
            u16 k;

            for (k = 0; k < take; k++) {
                if (raw[consumed + k] == 0xF7) {
                    break;
                }
            }

            if (k < take) {
                u8 cin = (k == 0) ? 0x5 : (k == 1) ? 0x6 : 0x7;
                packet[out_len++] = USB_MIDI_EVENT(USB_MIDI_CABLE_PORT, cin);
                packet[out_len++] = raw[consumed];
                packet[out_len++] = (k >= 1) ? raw[consumed + 1] : 0;
                packet[out_len++] = (k >= 2) ? raw[consumed + 2] : 0;
                consumed += (k + 1);
                usb_midi_tx_sysex_active = 0;
                continue;
            }

            if (remain < 3) {
                break;
            }

            packet[out_len++] = USB_MIDI_EVENT(USB_MIDI_CABLE_PORT, 0x4);
            packet[out_len++] = raw[consumed];
            packet[out_len++] = raw[consumed + 1];
            packet[out_len++] = raw[consumed + 2];
            consumed += 3;
            continue;
        }

        b = raw[consumed];
        if (b >= 0x80) {
            u8 msg_len;
            u8 cin;
            u8 b2;
            u8 b3;

            if (b == 0xF0) {
                usb_midi_tx_sysex_active = 1;
                usb_midi_tx_running_status = 0;
                continue;
            }

            msg_len = usb_midi_msg_total_len(b);
            if (msg_len > remain) {
                break;
            }
            cin = usb_midi_cin_from_status(b);
            b2 = (msg_len >= 2) ? raw[consumed + 1] : 0;
            b3 = (msg_len >= 3) ? raw[consumed + 2] : 0;

            packet[out_len++] = USB_MIDI_EVENT(USB_MIDI_CABLE_PORT, cin);
            packet[out_len++] = b;
            packet[out_len++] = b2;
            packet[out_len++] = b3;
            consumed += msg_len;

            if ((b >= 0x80) && (b <= 0xEF)) {
                usb_midi_tx_running_status = b;
            } else if ((b >= 0xF0) && (b < 0xF8)) {
                usb_midi_tx_running_status = 0;
            }
            continue;
        }

        if (usb_midi_tx_running_status) {
            u8 status = usb_midi_tx_running_status;
            u8 msg_len = usb_midi_msg_total_len(status);
            u8 data_need = msg_len - 1;
            u8 cin = (status >> 4) & 0x0F;
            u8 b2 = 0;

            if (data_need > remain) {
                break;
            }
            if (data_need == 2) {
                b2 = raw[consumed + 1];
            }

            packet[out_len++] = USB_MIDI_EVENT(USB_MIDI_CABLE_PORT, cin);
            packet[out_len++] = status;
            packet[out_len++] = raw[consumed];
            packet[out_len++] = (data_need == 2) ? b2 : 0;
            consumed += data_need;
        } else {
            packet[out_len++] = USB_MIDI_EVENT(USB_MIDI_CABLE_PORT, 0xF);
            packet[out_len++] = b;
            packet[out_len++] = 0;
            packet[out_len++] = 0;
            consumed += 1;
        }
    }

    if (consumed) {
        local_irq_disable();
        midi_ring_consume(USB_MIDI_TX_BUFF_SIZE, &usb_midi_tx_buf_rp, &usb_midi_tx_full, consumed);
        local_irq_enable();
    }

    return out_len;
}

static void ble_midi_tx_send_pump(void)
{
    if (md400_nb_ble_tx_pump()) return;
    u16 used;
    u8 raw[32];
    u8 packet[64];
    u16 packet_len;
    u16 raw_len;

    local_irq_disable();
    used = midi_ring_used(ble_midi_tx_buf_rp, ble_midi_tx_buf_wp, BLE_MIDI_TX_BUFF_SIZE, ble_midi_tx_full);
    local_irq_enable();
    if (!used) {
        return;
    }

#if CONFIG_BT_GATT_CLIENT_NUM
    if (trans_ble_midi_client_is_ready()) {
        raw_len = (used > sizeof(raw)) ? sizeof(raw) : used;
        local_irq_disable();
        raw_len = midi_ring_peek_copy(raw, raw_len, ble_midi_tx_buf, ble_midi_tx_buf_rp, ble_midi_tx_buf_wp, BLE_MIDI_TX_BUFF_SIZE, ble_midi_tx_full);
        local_irq_enable();

        if (!raw_len) {
            return;
        }

        packet_len = ble_midi_encode_packet(raw, raw_len, packet, sizeof(packet));
        if (!packet_len) {
            return;
        }

        if (trans_ble_midi_client_send_now(packet, packet_len) == 0) {
            local_irq_disable();
            midi_ring_consume(BLE_MIDI_TX_BUFF_SIZE, &ble_midi_tx_buf_rp, &ble_midi_tx_full, raw_len);
            local_irq_enable();
            ble_midi_tx_written_bytes += raw_len;
            ble_midi_tx_packet_bytes += packet_len;
        } else {
            ble_midi_tx_send_failures++;
        }
        return;
    }
#endif

    if (!trans_con_handle ||
        !ble_gatt_server_characteristic_ccc_get(trans_con_handle, ATT_CHARACTERISTIC_BLE_MIDI_IO_01_CLIENT_CONFIGURATION_HANDLE)) {
        local_irq_disable();
        ble_midi_tx_buf_rp = ble_midi_tx_buf_wp;
        ble_midi_tx_full = 0;
        local_irq_enable();
        ble_midi_tx_ccc_disabled_drops += used;
        ble_midi_tx_dropped_bytes += used;
        return;
    }

    raw_len = (used > sizeof(raw)) ? sizeof(raw) : used;
    local_irq_disable();
    raw_len = midi_ring_peek_copy(raw, raw_len, ble_midi_tx_buf, ble_midi_tx_buf_rp, ble_midi_tx_buf_wp, BLE_MIDI_TX_BUFF_SIZE, ble_midi_tx_full);
    local_irq_enable();

    if (!raw_len) {
        return;
    }

    packet_len = ble_midi_encode_packet(raw, raw_len, packet, sizeof(packet));
    if (!packet_len) {
        return;
    }

    if (ble_comm_att_check_send(trans_con_handle, packet_len)) {
        if (ble_comm_att_send_data(trans_con_handle, ATT_CHARACTERISTIC_BLE_MIDI_IO_01_VALUE_HANDLE, packet, packet_len, ATT_OP_AUTO_READ_CCC) == 0) {
            local_irq_disable();
            midi_ring_consume(BLE_MIDI_TX_BUFF_SIZE, &ble_midi_tx_buf_rp, &ble_midi_tx_full, raw_len);
            local_irq_enable();
            ble_midi_tx_written_bytes += raw_len;
            ble_midi_tx_packet_bytes += packet_len;
        } else {
            ble_midi_tx_send_failures++;
        }
    } else {
        ble_midi_tx_send_failures++;
    }
}

void usb_midi_ac632n_kick_tx(void)
{
#if CONFIG_MD400_ENCODER_CC_ONLY_TEST
    static u8 raw_kick_logged;

    if (!raw_kick_logged) {
        printf("[MD400_DIAG] raw usb midi tx pump disabled\r\n");
        raw_kick_logged = 1;
    }
    md400_usb_midi_tx_stream_clear();
    return;
#endif
#if TCFG_USB_SLAVE_ENABLE
    u16 used;
    u8 packet[64];
    u32 out_len;
    struct usb_device_t *usb_dev;

    local_irq_disable();
    used = midi_ring_used(usb_midi_tx_buf_rp, usb_midi_tx_buf_wp, USB_MIDI_TX_BUFF_SIZE, usb_midi_tx_full);
    local_irq_enable();
    if (!used) {
        return;
    }

    usb_dev = usb_id2device(0);
    if (!usb_dev || (usb_dev->bDeviceStates != USB_CONFIGURED)) {
        local_irq_disable();
        usb_midi_tx_buf_rp = usb_midi_tx_buf_wp;
        usb_midi_tx_full = 0;
        local_irq_enable();
        return;
    }

    /* Keep the raw stream until there is room for every encoded event.
     * The reservation and enqueue share the reset/producer IRQ exclusion. */
    local_irq_disable();
    out_len = usb_midi_device_tx_space(0);
    if (out_len > sizeof(packet)) out_len = sizeof(packet);
    if (out_len) {
        out_len = ac632n_usb_midi_tx(packet, out_len);
        if (out_len) usb_midi_device_send(0, packet, out_len);
    }
    local_irq_enable();
#endif
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI统一接收入口(client notify与server write共用)，解码后写入ble_midi_rx_buf
 */
static void md400_nb_ble_control_rx(void *context, const uint8_t *data, uint16_t length)
{
    md400_nb_origin_t origin = {1, (u16)(u32)context, md400_nb_ble_epoch};
    if (origin.handle != trans_con_handle || !origin.handle) return;
    md400_nb_submit(origin, data, length);
}

static void md400_nb_ble_performance_rx(void *context, const uint8_t *data, uint16_t length)
{
    ble_midi_rx_write_drop_on_full(data, length);
    if (trans_ble_midi_rx_cb) trans_ble_midi_rx_cb((u16)(u32)context, data, length);
}

void trans_ble_midi_on_rx(u16 conn_handle, const u8 *data, u16 len)
{
    static u32 ble_midi_rx_dbg_cnt;
    static u8 decoded[BLE_MIDI_RX_BUFF_SIZE];
    u16 ble_rx_used_after;
    u16 decoded_len;

    if (!data || !len) {
        return;
    }

    decoded_len = ble_midi_decode_packet(data, len, decoded, sizeof(decoded));
    if (!decoded_len) {
        return;
    }

    ble_midi_rx_packet_bytes += len;
    ble_midi_rx_dbg_cnt++;

    if (conn_handle == trans_con_handle) {
        md400_nb_stream_feed(&md400_nb_ble_stream, decoded, decoded_len,
            (void *)(u32)conn_handle, md400_nb_ble_control_rx, md400_nb_ble_performance_rx);
    } else {
        md400_nb_ble_performance_rx((void *)(u32)conn_handle, decoded, decoded_len);
    }
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: 设置BLE MIDI统一接收回调
 */
int trans_ble_midi_set_rx_handler(void (*cb)(u16 conn_handle, const u8 *data, u16 len))
{
    trans_ble_midi_rx_cb = cb;
    return 0;
}

/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI统一发送接口(写入ble_midi_tx_buf，由发送泵异步发送)
 */
int trans_ble_midi_send(const u8 *data, u16 len)
{
    if (!data || !len) {
        return GATT_CMD_PARAM_ERROR;
    }
    if (len >= BLE_MIDI_TX_BUFF_SIZE) {
        return GATT_CMD_PARAM_ERROR;
    }

    if (ble_midi_tx_write_drop_on_full(data, len)) {
        return GATT_BUFFER_FULL;
    }
    return 0;
}

/*
 打开流控使能后,确定使能接口 att_server_flow_enable 被调用
 然后使用过程 通过接口 att_server_flow_hold 来控制流控开关
 注意:流控只能控制对方使用带响应READ/WRITE等命令方式
 例如:ATT_WRITE_REQUEST = 0x12
 */
#define ATT_DATA_RECIEVT_FLOW      0//流控功能使能


//测试NRF连接,工具不会主动发起交换流程,需要手动操作; 但设备可配置主动发起MTU长度交换请求
// 修改日期: 2026-02-15, 修改说明: 启用主动MTU交换以满足Windows BLE MIDI要求
#define ATT_MTU_REQUEST_ENALBE     1    /*配置1,就是设备端主动发起交换*/

//检测对方的系统类型，ios or 非ios
#define ATT_CHECK_REMOTE_REQUEST_ENALBE     0    /*配置1,就是设备端主动检查*/

//ATT发送的包长,    note: 23 <=need >= MTU
/* 128 (was 200): with RCSP compiled in for BLE OTA, the old 200/512 ATT pool
 * left the normal boot so short of pmalloc pages that BLE role transitions and
 * connects died in "No enough physics memory" assert loops (hardware-verified).
 * BLE-MIDI packets are self-capped at 64 bytes, so 128 keeps 2x headroom. */
#define ATT_LOCAL_MTU_SIZE        (128)

//ATT缓存的buffer支持缓存数据包个数
#define ATT_PACKET_NUMS_MAX       (2)

//ATT缓存的buffer大小,  note: need >= 23,可修改
#define ATT_SEND_CBUF_SIZE        (256) /* was 512; see ATT_LOCAL_MTU_SIZE note */

// 广播周期 (unit:0.625ms)
// Apple 规范: 先用 20ms 广播至少 30 秒，之后可用 152.5ms 等特定值
// 20ms = 32 * 0.625ms
// 152.5ms = 244 * 0.625ms
#define ADV_INTERVAL_FAST         (32)   // 20ms - Apple 要求的快速广播间隔
#define ADV_INTERVAL_SLOW         (244)  // 152.5ms - Apple 允许的慢速广播间隔
#define ADV_INTERVAL_MIN          ADV_INTERVAL_FAST  // 默认使用快速间隔

#define TEST_TRANS_CHANNEL_DATA      0 /*测试记录收发数据速度*/
// 修改日期: 2026-02-10, 修改说明: TEST handle切换为BLE MIDI IO characteristic
#define TEST_TRANS_NOTIFY_HANDLE     ATT_CHARACTERISTIC_BLE_MIDI_IO_01_VALUE_HANDLE /*主动发送hanlde,为空则不测试发数*/
#if CONFIG_BLE_HIGH_SPEED
#define TEST_TRANS_TIMER_MS          5
#else
#define TEST_TRANS_TIMER_MS          500
#endif
#define TEST_PAYLOAD_LEN            (244)/*发送配PDU长度是251的包*/

static u32 trans_recieve_test_count;
static u32 trans_send_test_count;

//---------------
//连接参数更新请求设置
//是否使能参数请求更新,0--disable, 1--enable
static uint8_t trans_connection_update_enable = 1; ///0--disable, 1--enable

//请求的参数数组表,排队方式请求;哪组对方接受就用那组
// 修改日期: 2026-02-15, 修改说明: 优化连接参数，参考HID项目配置，优先快速iOS参数
static const struct conn_update_param_t trans_connection_param_table[] = {
    {6,  12,  30, 600}, // ios fast 优先 (7.5-15ms, latency 30)
    {12, 12,  30, 600}, // ios 标准 (15ms, latency 30)
    {8,  20,  20, 600}, // 通用回退 (10-25ms, latency 20)
};

//共可用的参数组数
#define CONN_PARAM_TABLE_CNT      (sizeof(trans_connection_param_table)/sizeof(struct conn_update_param_t))

#define EIR_TAG_STRING   0xd6, 0x05, 0x08, 0x00, 'J', 'L', 'A', 'I', 'S', 'D','K'
static const char user_tag_string[] = {EIR_TAG_STRING};

//定义的产品信息,for test
#define  PNP_VID_SOURCE   0x02
#define  PNP_VID          0x05ac //0x05d6
#define  PNP_PID          0x022C //
#define  PNP_PID_VERSION  0x011b //1.1.11

static const u8 trans_PNP_ID[] = {PNP_VID_SOURCE, PNP_VID & 0xFF, PNP_VID >> 8, PNP_PID & 0xFF, PNP_PID >> 8, PNP_PID_VERSION & 0xFF, PNP_PID_VERSION >> 8};

// 修改日期: 2026-02-10, 修改说明: BLE MIDI Service UUID128(小端格式，用于广播服务列表)
static const u8 ble_midi_service_uuid128_le[16] = {0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03};

// 修改日期: 2026-02-15, 修改说明: 配对信息定义已移至文件前部（角色切换代码之前）

static u8  trans_adv_data[ADV_RSP_PACKET_MAX];//max is 31
static u8  trans_scan_rsp_data[ADV_RSP_PACKET_MAX];//max is 31
static u8  trans_test_read_write_buf[4];
static adv_cfg_t trans_server_adv_config;
//-------------------------------------------------------------------------------------
static uint16_t trans_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int trans_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int trans_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param);
extern void uart_db_regiest_recieve_callback(void *rx_cb);
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
//输入passkey 加密
#define PASSKEY_ENABLE                     0

/* Non-const so bt_ble_init() can relax it for BLE OTA mode: the normal app
 * forces bonded+MITM pairing (for iOS BLE-MIDI auto-pair), which a plain host
 * central (btleplug, no pairing) cannot satisfy — the link stalls at the
 * security request. In BLE OTA mode the RCSP GATT is opened without forced
 * pairing so the host can connect and drive the update. */
static sm_cfg_t trans_sm_init_config = {
    .slave_security_auto_req = 0,  // 不主动强制配对：允许不配对的host(如btleplug/Windows)连接RCSP; iOS仍可自行发起配对
    .slave_set_wait_security = 0,  // 不等待加密完成，连接后即可访问(RCSP OTA不需要加密)

#if PASSKEY_ENABLE
    .io_capabilities = IO_CAPABILITY_DISPLAY_ONLY,
#else
    .io_capabilities = IO_CAPABILITY_NO_INPUT_NO_OUTPUT,
#endif

    .authentication_req_flags = SM_AUTHREQ_BONDING | SM_AUTHREQ_MITM_PROTECTION,
    .min_key_size = 7,
    .max_key_size = 16,
    .sm_cb_packet_handler = NULL,
};

const gatt_server_cfg_t trans_server_init_cfg = {
    .att_read_cb = &trans_att_read_callback,
    .att_write_cb = &trans_att_write_callback,
    .event_packet_handler = &trans_event_packet_handler,
};

static gatt_ctrl_t trans_gatt_control_block = {
    //public
    .mtu_size = ATT_LOCAL_MTU_SIZE,
    .cbuffer_size = ATT_SEND_CBUF_SIZE,
    .multi_dev_flag	= 0,

    //config
#if CONFIG_BT_GATT_SERVER_NUM
    .server_config = &trans_server_init_cfg,
#else
    .server_config = NULL,
#endif

#if CONFIG_BT_GATT_CLIENT_NUM
    .client_config = &trans_client_init_cfg,
#else
    .client_config = NULL,
#endif

#if CONFIG_BT_SM_SUPPORT_ENABLE
    .sm_config = &trans_sm_init_config,
#else
    .sm_config = NULL,
#endif
    //cbk,event handle
    .hci_cb_packet_handler = NULL,
};


#define TEST_AUDIO_DATA_UPLOAD       0//测试文件上传

#if TEST_AUDIO_DATA_UPLOAD
static const u8 test_audio_data_file[1024] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9
};

/*************************************************************************************************/
/*!
 *  \brief      测试上传文件
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
#define AUDIO_ONE_PACKET_LEN  128
static void trans_test_send_audio_data(int init_flag)
{
    static u32 send_pt = 0;
    static u32 start_flag = 0;

    if (!trans_con_handle) {
        return;
    }

    if (init_flag) {
        log_info("audio send init\n");
        send_pt = 0;
        start_flag = 1;
    }

    if (!start_flag) {
        return;
    }

    u32 file_size = sizeof(test_audio_data_file);
    u8 *file_ptr = test_audio_data_file;

    if (send_pt >= file_size) {
        log_info("audio send Complete\n");
        start_flag = 0;
        return;
    }

    u32 send_len = file_size - send_pt;
    if (send_len > AUDIO_ONE_PACKET_LEN) {
        send_len = AUDIO_ONE_PACKET_LEN;
    }

    while (1) {
        if (ble_comm_cbuffer_vaild_len(trans_con_handle) > send_len) {
            log_info("audio send %08x\n", send_pt);
            if (ble_comm_att_send_data(trans_con_handle, ATT_CHARACTERISTIC_ae3c_01_VALUE_HANDLE, &file_ptr[send_pt], send_len, ATT_OP_AUTO_READ_CCC)) {
                log_info("audio send fail!\n");
                break;
            } else {
                send_pt += send_len;
            }
        } else {
            break;
        }
    }
}

#endif

/*************************************************************************************************/
/*!
 *  \brief      串口接收转发到BLE
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void trans_uart_rx_to_ble(u8 *packet, u32 size)
{
    // 修改日期: 2026-02-10, 修改说明: 串口数据写入BLE MIDI统一tx_buf，由发送泵按角色发送
    if (trans_ble_midi_send(packet, size)) {
        log_info("drop uart data!!!\n");
    }
}

/*************************************************************************************************/
/*!
 *  \brief      发送请求连接参数表
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void trans_send_connetion_updata_deal(u16 conn_handle)
{
    if (trans_connection_update_enable) {
        printf("\n[DEBUG] ========== Requesting Connection Parameter Update ==========\n");
        printf("[DEBUG] Connection handle: 0x%04x\n", conn_handle);
        printf("[DEBUG] Parameter table count: %d\n", CONN_PARAM_TABLE_CNT);

        // 使用临时变量避免结构体访问问题
        for (int i = 0; i < CONN_PARAM_TABLE_CNT; i++) {
            const struct conn_update_param_t *param = &trans_connection_param_table[i];
            u16 imin = param->interval_min;
            u16 imax = param->interval_max;
            u16 lat = param->latency;
            u16 tmo = param->timeout;

            printf("[DEBUG] Param[%d]: interval_min=%d (%.2f ms), interval_max=%d (%.2f ms), latency=%d, timeout=%d (%d ms)\n",
                   i,
                   imin, imin * 1.25,
                   imax, imax * 1.25,
                   lat,
                   tmo, tmo * 10);
        }

        if (0 == ble_gatt_server_connetion_update_request(conn_handle, trans_connection_param_table, CONN_PARAM_TABLE_CNT)) {
            trans_connection_update_enable = 0;
            printf("[DEBUG] Connection parameter update request sent successfully\n");
        } else {
            printf("[DEBUG] ERROR: Failed to send connection parameter update request\n");
        }
        printf("[DEBUG] ===============================================================\n\n");
    } else {
        printf("[DEBUG] Connection parameter update already requested or disabled\n");
    }
}

/*************************************************************************************************/
/*!
 *  \brief      配对信息VM读写
 *
 *  \param      [in] info - 配对信息结构体指针
 *  \param      [in] rw_flag - 0:读取, 1:写入
 *
 *  \return
 *
 *  \note       修改日期: 2026-02-15, 修改说明: 添加配对信息VM存储，实现自动回连
 */
/*************************************************************************************************/
static void trans_conn_pair_vm_do(struct trans_pair_info_t *info, u8 rw_flag)
{
    int vm_len = sizeof(struct trans_pair_info_t);
    int ret;
    struct trans_pair_info_t verify_info;

    log_info("-trans_pair_info vm_do:%d\n", rw_flag);

    if (!info) {
        return;
    }

    if (rw_flag && !md400_persist_in_task) {
        md400_persist_submit(MD400_PERSIST_MSG_BLE_PAIR, 0, 0);
        return;
    }

    if (rw_flag == 0) {
        // 读取VM
        ret = syscfg_read(CFG_BLE_BONDING_REMOTE_INFO, (u8 *)info, vm_len);
        if (vm_len != ret) {
            log_info("-null--\n");
            memset(info, 0, vm_len);
        }

        // 修改日期: 2026-04-03, 修改说明: 只有tag、pair_flag、地址都有效才认为VM中存在已配对设备
        if (trans_pair_info_is_valid(info)) {
            log_info("-exist--\n");
            printf("[PAIR] Found paired device in VM\n");
            printf("[PAIR] Device address: ");
            put_buf(info->peer_address_info, 7);
        } else {
            log_info("-reset vm--\n");
            memset(info, 0, vm_len);
            info->head_tag = BLE_VM_HEAD_TAG;
            info->tail_tag = BLE_VM_TAIL_TAG;
            printf("[PAIR] No paired device found in VM\n");
        }
    } else {
        // 修改日期: 2026-04-03, 修改说明: 写VM前统一整理配对结构，并回读校验写入结果
        if (info->pair_flag) {
            info->head_tag = BLE_VM_HEAD_TAG;
            info->tail_tag = BLE_VM_TAIL_TAG;
        } else {
            memset(info->peer_address_info, 0, sizeof(info->peer_address_info));
            info->direct_adv_cnt = 0;
            info->head_tag = BLE_VM_HEAD_TAG;
            info->tail_tag = BLE_VM_TAIL_TAG;
        }

        ret = syscfg_write(CFG_BLE_BONDING_REMOTE_INFO, (u8 *)info, vm_len);
        printf("[PAIR] VM write ret=%d expect=%d\n", ret, vm_len);

        memset(&verify_info, 0, sizeof(verify_info));
        ret = syscfg_read(CFG_BLE_BONDING_REMOTE_INFO, (u8 *)&verify_info, vm_len);
        printf("[PAIR] VM verify ret=%d expect=%d\n", ret, vm_len);

        if ((ret == vm_len) && (memcmp(&verify_info, info, vm_len) == 0)) {
            printf("[PAIR] Paired device saved to VM\n");
            if (info->pair_flag) {
                printf("[PAIR] Device address: ");
                put_buf(info->peer_address_info, 7);
            }
        } else {
            printf("[PAIR] ERROR: VM verify mismatch\n");
            printf("[PAIR] Expect: ");
            put_buf((u8 *)info, vm_len);
            printf("[PAIR] Actual: ");
            put_buf((u8 *)&verify_info, vm_len);
        }
    }
    log_info_hexdump(info, vm_len);
}

/*************************************************************************************************/
/*!
 *  \brief      回连状态，使能所有profile
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      配对绑定的方式，主机回连不是在使能server的通知开关，需要自己打开
 */
/*************************************************************************************************/
static void trans_resume_all_ccc_enable(u16 conn_handle, u8 update_request)
{
    log_info("resume_all_ccc_enable\n");

#if RCSP_BTMATE_EN
    ble_gatt_server_characteristic_ccc_set(conn_handle, ATT_CHARACTERISTIC_ae02_02_CLIENT_CONFIGURATION_HANDLE, ATT_OP_NOTIFY);
#endif
    // 修改日期: 2026-02-10, 修改说明: 仅恢复BLE MIDI IO characteristic的notify
    ble_gatt_server_characteristic_ccc_set(conn_handle, ATT_CHARACTERISTIC_BLE_MIDI_IO_01_CLIENT_CONFIGURATION_HANDLE, ATT_OP_NOTIFY);

    if (update_request) {
        trans_send_connetion_updata_deal(conn_handle);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      反馈检查对方的操作系统
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note 参考识别手机系统
 */
/*************************************************************************************************/
static void trans_check_remote_result(u16 con_handle, remote_type_e remote_type)
{
    char *str;
    if (REMOTE_TYPE_IOS == remote_type) {
        str = "is";
    } else {
        str = "not";
    }

    log_info("trans_check %02x:remote_type= %02x, %s ios", con_handle, remote_type, str);
}

/*************************************************************************************************/
/*!
 *  \brief      处理gatt 返回的事件（hci && gatt）
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int trans_event_packet_handler(int event, u8 *packet, u16 size, u8 *ext_param)
{
    /* log_info("event: %02x,size= %d\n",event,size); */

    switch (event) {

    case GATT_COMM_EVENT_CAN_SEND_NOW:
#if TEST_AUDIO_DATA_UPLOAD
        trans_test_send_audio_data(0);
#endif
        break;

    case GATT_COMM_EVENT_SERVER_INDICATION_COMPLETE:
        log_info("INDICATION_COMPLETE:con_handle= %04x,att_handle= %04x\n", \
                 little_endian_read_16(packet, 0), little_endian_read_16(packet, 2));
        break;


    case GATT_COMM_EVENT_CONNECTION_COMPLETE:
        // DEBUG: 打印连接建立事件的详细信息
        printf("\n========== BLE CONNECTION COMPLETE ==========\n");
        printf("[DEBUG] New connection handle: 0x%04x\n", little_endian_read_16(packet, 0));
        printf("[DEBUG] Current trans_comm_connected_handle: 0x%04x\n", trans_comm_connected_handle);

        // 修改日期: 2026-02-10, 修改说明: 只允许保持一个BLE连接(新连接建立时若已有连接则立即断开新连接)
        if (trans_comm_connected_handle && (trans_comm_connected_handle != little_endian_read_16(packet, 0))) {
            printf("[DEBUG] Already connected, rejecting new connection 0x%04x\n", little_endian_read_16(packet, 0));
            ble_comm_disconnect(little_endian_read_16(packet, 0));
            break;
        }
        md400_nb_ble_link_reset();
        trans_con_handle = little_endian_read_16(packet, 0);

        // 修改日期: 2026-02-15, 修改说明: 保存当前连接的对方地址信息
        memcpy(trans_cur_peer_addr_info, &ext_param[7], 7);
        printf("[DEBUG] Peer device address: ");
        put_buf(trans_cur_peer_addr_info, 7);

        // 修改日期: 2026-02-10, 修改说明: server连接也纳入统一连接状态(连接后停止分时切换/禁止再开scan&adv)
        trans_comm_connected_handle_on_connect(trans_con_handle);
        trans_connection_update_enable = 1;
        // 修改日期: 2026-02-10, 修改说明: 指示灯由统一连接状态控制，这里不再单独处理

        log_info("connection_handle:%04x\n", little_endian_read_16(packet, 0));
        log_info("connection_handle:%04x, rssi= %d\n", trans_con_handle, ble_vendor_get_peer_rssi(trans_con_handle));
        log_info("peer_address_info:");
        put_buf(&ext_param[7], 7);

        printf("[DEBUG] Connection parameters:\n");
        printf("[DEBUG]   Interval: %d (%.2f ms)\n", little_endian_read_16(ext_param, 14 + 0), little_endian_read_16(ext_param, 14 + 0) * 1.25);
        printf("[DEBUG]   Latency: %d\n", little_endian_read_16(ext_param, 14 + 2));
        printf("[DEBUG]   Timeout: %d (%d ms)\n", little_endian_read_16(ext_param, 14 + 4), little_endian_read_16(ext_param, 14 + 4) * 10);

        log_info("con_interval = %d\n", little_endian_read_16(ext_param, 14 + 0));
        log_info("con_latency = %d\n", little_endian_read_16(ext_param, 14 + 2));
        log_info("cnn_timeout = %d\n", little_endian_read_16(ext_param, 14 + 4));

#if ATT_MTU_REQUEST_ENALBE
        /* Skip the proactive MTU grow in BLE OTA mode: a larger negotiated MTU
         * forces a bigger per-connection buffer allocation that overflows the
         * AC632 heap. RCSP OTA works fine at the default MTU. */
        if (!md400_ota_boot_is_ble()) {
            printf("[DEBUG] Requesting MTU exchange...\n");
            att_server_set_exchange_mtu(trans_con_handle);/*主动请求MTU长度交换*/
        } else {
            printf("[MD400_OTA] MTU exchange skipped (small buffers)\n");
        }
#else
        printf("[DEBUG] MTU exchange disabled\n");
#endif

#if (TCFG_UART0_RX_PORT != NO_CONFIG_PORT) && CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
        //for test 串口数据直通到蓝牙
        uart_db_regiest_recieve_callback(trans_uart_rx_to_ble);
#endif

#if ATT_CHECK_REMOTE_REQUEST_ENALBE
        att_server_set_check_remote(trans_con_handle, trans_check_remote_result);
#endif
        printf("============================================\n\n");
        break;

    case GATT_COMM_EVENT_DISCONNECT_COMPLETE:
        printf("\n========== BLE DISCONNECTION ==========\n");
        printf("[DEBUG] Disconnect handle: 0x%04x\n", little_endian_read_16(packet, 0));
        printf("[DEBUG] Disconnect reason: 0x%02x ", packet[2]);
        // 打印断开原因的详细说明
        switch(packet[2]) {
            case 0x08: printf("(Connection Timeout)\n"); break;
            case 0x13: printf("(Remote User Terminated)\n"); break;
            case 0x14: printf("(Remote Device Low Resources)\n"); break;
            case 0x15: printf("(Remote Device Power Off)\n"); break;
            case 0x16: printf("(Connection Terminated by Local Host)\n"); break;
            case 0x22: printf("(LMP Response Timeout)\n"); break;
            case 0x3B: printf("(Unacceptable Connection Parameters)\n"); break;
            case 0x3E: printf("(Advertising Timeout)\n"); break;
            default: printf("(Unknown)\n"); break;
        }
        printf("[DEBUG] trans_con_handle: 0x%04x\n", trans_con_handle);
        printf("=======================================\n\n");

        log_info("disconnect_handle:%04x,reason= %02x\n", little_endian_read_16(packet, 0), packet[2]);
        if (trans_con_handle == little_endian_read_16(packet, 0)) {
#if CONFIG_BT_GATT_CLIENT_NUM
            trans_client_search_remote_stop(trans_con_handle);
#endif
            md400_nb_ble_link_reset();
            trans_con_handle = 0;
        }

        // 注意：断开时不更新配对地址，配对地址应在配对成功时保存
        // ble_list_get_last_id_addr 可能返回错误的地址

        // 修改日期: 2026-02-10, 修改说明: server断开也更新统一连接状态(断开后恢复分时切换)
        trans_comm_connected_handle_on_disconnect(little_endian_read_16(packet, 0));
        break;

    case GATT_COMM_EVENT_CONNECTION_COMPLETE_FAIL:
        printf("\n========== CONNECTION FAILED ==========\n");
        printf("[DEBUG] Connection attempt failed\n");
        printf("=======================================\n\n");
#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
        trans_role_on_disconnected();
#endif
        break;

    case GATT_COMM_EVENT_ENCRYPTION_CHANGE:
        printf("[DEBUG] ENCRYPTION_CHANGE: handle=0x%04x, state=%d, process=%d\n",
               little_endian_read_16(packet, 0), packet[2], packet[3]);
        log_info("ENCRYPTION_CHANGE:handle=%04x,state=%d,process =%d", little_endian_read_16(packet, 0), packet[2], packet[3]);

        if (packet[2] == 0) { // 加密成功
            if (packet[3] == LINK_ENCRYPTION_RECONNECT) {
                // 回连：检查地址是否匹配，如果不匹配则更新
                printf("[DEBUG] Encryption successful (RECONNECT)\n");
                if ((!trans_pair_info.pair_flag) || memcmp(trans_cur_peer_addr_info, trans_pair_info.peer_address_info, 7)) {
                    memcpy(trans_pair_info.peer_address_info, trans_cur_peer_addr_info, 7);
                    trans_pair_info.pair_flag = 1;
                    trans_conn_pair_vm_do(&trans_pair_info, 1);
                    printf("[DEBUG] Updated pair address on reconnect\n");
                }

                // 回连时从配对表中获取 remote_type
                u8 tmp_buf[6];
                u8 remote_type = 0;
                swapX(&trans_cur_peer_addr_info[1], tmp_buf, 6);
                ble_list_get_remote_type(tmp_buf, trans_cur_peer_addr_info[0], &remote_type);
                trans_check_remote_result(trans_con_handle, remote_type);
            } else {
                // 首次配对：保存地址并设置 check_remote 回调
                printf("[DEBUG] Encryption successful (FIRST PAIRING)\n");
                memcpy(trans_pair_info.peer_address_info, trans_cur_peer_addr_info, 7);
                att_server_set_check_remote(trans_con_handle, trans_check_remote_result);
                trans_pair_info.pair_flag = 1;
                trans_conn_pair_vm_do(&trans_pair_info, 1);
                printf("[DEBUG] Pairing info saved\n");
            }

            printf("[DEBUG] Enabling MIDI notify CCC...\n");
            trans_resume_all_ccc_enable(little_endian_read_16(packet, 0), 1);
        }
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_COMPLETE:
        printf("\n[DEBUG] Connection parameters updated:\n");
        printf("[DEBUG]   Handle: 0x%04x\n", little_endian_read_16(packet, 0));
        printf("[DEBUG]   New Interval: %d (%.2f ms)\n", little_endian_read_16(ext_param, 6 + 0), little_endian_read_16(ext_param, 6 + 0) * 1.25);
        printf("[DEBUG]   New Latency: %d\n", little_endian_read_16(ext_param, 6 + 2));
        printf("[DEBUG]   New Timeout: %d (%d ms)\n\n", little_endian_read_16(ext_param, 6 + 4), little_endian_read_16(ext_param, 6 + 4) * 10);

        log_info("conn_param update_complete:%04x\n", little_endian_read_16(packet, 0));
        log_info("update_interval = %d\n", little_endian_read_16(ext_param, 6 + 0));
        log_info("update_latency = %d\n", little_endian_read_16(ext_param, 6 + 2));
        log_info("update_timeout = %d\n", little_endian_read_16(ext_param, 6 + 4));
        break;

    case GATT_COMM_EVENT_CONNECTION_UPDATE_REQUEST_RESULT:
        printf("[DEBUG] Connection update request result received\n");
        break;

    case GATT_COMM_EVENT_MTU_EXCHANGE_COMPLETE:
        md400_nb_ble_mtu = little_endian_read_16(packet, 2);
        if (md400_nb_ble_mtu > ATT_LOCAL_MTU_SIZE) md400_nb_ble_mtu = ATT_LOCAL_MTU_SIZE;
        printf("[DEBUG] MTU Exchange Complete: handle=0x%04x, MTU=%u\n",
               little_endian_read_16(packet, 0), little_endian_read_16(packet, 2));
        log_info("con_handle= %02x, ATT MTU = %u\n", little_endian_read_16(packet, 0), little_endian_read_16(packet, 2));
        break;

    case GATT_COMM_EVENT_SERVER_STATE:
        log_info("server_state: handle=%02x,%02x\n", little_endian_read_16(packet, 1), packet[0]);
        break;

    case GATT_COMM_EVENT_ENCRYPTION_REQUEST:
        // 参考 SDK 原版：加密请求时保存对方地址
        log_info("[PAIR] ENCRYPTION_REQUEST\n");
        memcpy(trans_pair_info.peer_address_info, trans_cur_peer_addr_info, 7);
        trans_pair_info.pair_flag = 0;
        break;

    case GATT_COMM_EVENT_DIRECT_ADV_TIMEOUT:
        // 参考 SDK 原版：定向广播超时，减少计数，重新配置广播
        log_info("[ADV] DIRECT_ADV_TIMEOUT, cnt=%d\n", trans_pair_info.direct_adv_cnt);
        if (trans_pair_info.direct_adv_cnt) {
            trans_pair_info.direct_adv_cnt--;
        }
        trans_adv_config_set();
        break;

    case GATT_COMM_EVENT_SM_PASSKEY_INPUT: {
        u32 *key = little_endian_read_32(packet, 2);
        *key = 888888;
        r_printf("input_key:%6u\n", *key);
    }
    break;

    default:
        break;
    }
    return 0;
}


/*************************************************************************************************/
/*!
 *  \brief      处理client 读操作
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      profile的读属性uuid 有配置 DYNAMIC 关键字，就有read_callback 回调
 */
/*************************************************************************************************/
// ATT Client Read Callback for Dynamic Data
// - if buffer == NULL, don't copy data, just return size of value
// - if buffer != NULL, copy data and return number bytes copied
// @param con_handle of hci le connection
// @param attribute_handle to be read
// @param offset defines start of attribute value
// @param buffer
// @param buffer_size
static uint16_t trans_att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    uint16_t  att_value_len = 0;
    uint16_t handle = att_handle;

    log_info("read_callback,conn_handle =%04x, handle=%04x\n", connection_handle, handle);

    switch (handle) {
    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE: {
        char *gap_name = ble_comm_get_gap_name();
        att_value_len = strlen(gap_name);
        if (buffer && (offset < att_value_len)) {
            u16 copy_len = att_value_len - offset;
            if (copy_len > buffer_size) copy_len = buffer_size;
            memcpy(buffer, &gap_name[offset], copy_len);
            att_value_len = copy_len;
        }
    }
    break;

    case ATT_CHARACTERISTIC_2a01_01_VALUE_HANDLE:
        // Appearance: Generic HID = 0x03C0, or 0x0000 for unknown
        att_value_len = 2;
        if (buffer) {
            buffer[0] = 0x00;
            buffer[1] = 0x00;
        }
        break;

    case ATT_CHARACTERISTIC_2a04_01_VALUE_HANDLE:
        // Peripheral Preferred Connection Parameters: min_interval, max_interval, latency, timeout
        att_value_len = 8;
        if (buffer) {
            buffer[0] = 0x06; buffer[1] = 0x00;  // min interval = 6 (7.5ms)
            buffer[2] = 0x0c; buffer[3] = 0x00;  // max interval = 12 (15ms)
            buffer[4] = 0x1e; buffer[5] = 0x00;  // latency = 30
            buffer[6] = 0x58; buffer[7] = 0x02;  // timeout = 600 (6000ms)
        }
        break;

    case ATT_CHARACTERISTIC_2a29_01_VALUE_HANDLE: {
        // Manufacturer Name
        const char *mfr = "JieLi";
        att_value_len = strlen(mfr);
        if (buffer) { memcpy(buffer, mfr, att_value_len); }
    }
    break;

    case ATT_CHARACTERISTIC_2a24_01_VALUE_HANDLE: {
        // Model Number
        const char *model = "BLE-MIDI";
        att_value_len = strlen(model);
        if (buffer) { memcpy(buffer, model, att_value_len); }
    }
    break;

    case ATT_CHARACTERISTIC_BLE_MIDI_IO_01_VALUE_HANDLE:
        att_value_len = 0;
        break;

    case ATT_CHARACTERISTIC_BLE_MIDI_IO_01_CLIENT_CONFIGURATION_HANDLE:
    case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
        att_value_len = 2;
        if (buffer) {
            buffer[0] = ble_gatt_server_characteristic_ccc_get(connection_handle, handle);
            buffer[1] = 0;
        }
        break;

    default:
        break;
    }

    log_info("att_value_len= %d\n", att_value_len);
    return att_value_len;
}


/*************************************************************************************************/
/*!
 *  \brief      处理client write操作
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      profile的写属性uuid 有配置 DYNAMIC 关键字，就有write_callback 回调
 */
/*************************************************************************************************/
// ATT Client Write Callback for Dynamic Data
// @param con_handle of hci le connection
// @param attribute_handle to be written
// @param transaction - ATT_TRANSACTION_MODE_NONE for regular writes, ATT_TRANSACTION_MODE_ACTIVE for prepared writes and ATT_TRANSACTION_MODE_EXECUTE
// @param offset into the value - used for queued writes and long attributes
// @param buffer
// @param buffer_size
// @param signature used for signed write commmands
// @returns 0 if write was ok, ATT_ERROR_PREPARE_QUEUE_FULL if no space in queue, ATT_ERROR_INVALID_OFFSET if offset is larger than max buffer

static int trans_att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size)
{
    int result = 0;
    u16 tmp16;

    u16 handle = att_handle;

    printf("[ATT_WRITE] conn=0x%04x, handle=0x%04x, size=%d, offset=%d\n",
           connection_handle, handle, buffer_size, offset);

#if !TEST_TRANS_CHANNEL_DATA
    log_info("write_callback,conn_handle =%04x, handle =%04x,size =%d\n", connection_handle, handle, buffer_size);
#endif

    switch (handle) {

    case ATT_CHARACTERISTIC_2a00_01_VALUE_HANDLE:
        printf("[ATT_WRITE] GAP Name (ignored)\n");
        break;

    case ATT_CHARACTERISTIC_BLE_MIDI_IO_01_CLIENT_CONFIGURATION_HANDLE:
        printf("[ATT_WRITE] BLE MIDI CCC: 0x%02x%02x\n", buffer[1], buffer[0]);
        if (buffer[0] & 0x01) {
            printf("[ATT_WRITE] *** NOTIFY ENABLED ***\n");
        } else {
            printf("[ATT_WRITE] *** NOTIFY DISABLED ***\n");
        }
        trans_send_connetion_updata_deal(connection_handle);
        log_info("\n------write ccc:%04x,%02x\n", handle, buffer[0]);
        ble_gatt_server_characteristic_ccc_set(connection_handle, handle, buffer[0]);
        break;

    case ATT_CHARACTERISTIC_2a05_01_CLIENT_CONFIGURATION_HANDLE:
        printf("[ATT_WRITE] Service Changed CCC: 0x%02x%02x\n", buffer[1], buffer[0]);
        trans_send_connetion_updata_deal(connection_handle);
        log_info("\n------write ccc:%04x,%02x\n", handle, buffer[0]);
        ble_gatt_server_characteristic_ccc_set(connection_handle, handle, buffer[0]);
        break;

    case ATT_CHARACTERISTIC_BLE_MIDI_IO_01_VALUE_HANDLE:
        printf("[ATT_WRITE] BLE MIDI Data: %d bytes\n", buffer_size);
        // 修改日期: 2026-02-10, 修改说明: BLE MIDI server接收统一入口(落地rx_buf并触发上层回调)
        trans_ble_midi_on_rx(connection_handle, buffer, buffer_size);
        break;

#if RCSP_BTMATE_EN
    case ATT_CHARACTERISTIC_ae02_02_CLIENT_CONFIGURATION_HANDLE:
        ble_op_latency_skip(connection_handle, 0xffff); //
        ble_gatt_server_set_update_send(connection_handle, ATT_CHARACTERISTIC_ae02_02_VALUE_HANDLE, ATT_OP_AUTO_READ_CCC);
#endif
        /* trans_send_connetion_updata_deal(connection_handle); */
        log_info("------write ccc:%04x,%02x\n", handle, buffer[0]);
        ble_gatt_server_characteristic_ccc_set(connection_handle, handle, buffer[0]);
        break;

#if RCSP_BTMATE_EN
    case ATT_CHARACTERISTIC_ae01_02_VALUE_HANDLE:
        if (!md400_ota_mode) {
            /* Normal boot: RCSP is not initialized (see app_rcsp_connect_allow),
             * so ae01 only accepts the pure-BLE OTA arm command. The host writes
             * the magic, we flag the BLE-OTA boot and reset — same mode switch as
             * NanoBridge 0x51, but reachable without USB. */
            if (md400_ble_ota_arm_check(buffer, buffer_size)) {
                printf("[MD400_OTA] BLE arm write accepted -> OTA mode\r\n");
            } else {
                printf("[MD400_OTA] ae01 write ignored in normal boot len=%d\r\n",
                       buffer_size);
            }
            break;
        }
        /* BLE-OTA boot: this is RCSP traffic; feed the SDK and keep the OTA
         * watchdog from rebooting mid-session. */
        md400_ota_watchdog_kick();
        printf("[MD400_OTA] rcsp rx len=%u free_pages=%u\r\n",
               buffer_size, (unsigned)pmalloc_get_unused_pages());
        ble_gatt_server_receive_update_data(NULL, buffer, buffer_size);
        break;
#endif

    default:
        printf("[ATT_WRITE] Unknown handle: 0x%04x\n", handle);
        break;
    }
    return 0;
}


/*************************************************************************************************/
/*!
 *  \brief      组织adv包数据，放入buff
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static u8  adv_name_ok = 0;
static int trans_make_set_adv_data(u8 adv_reconnect)
{
    u8 offset = 0;
    u8 *buf = trans_adv_data;

#if DOUBLE_BT_SAME_MAC
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_LE_AND_EDR_SAME_CONTROLLER, 1);
#else
    offset += make_eir_packet_val(&buf[offset], offset, HCI_EIR_DATATYPE_FLAGS, FLAGS_GENERAL_DISCOVERABLE_MODE | FLAGS_EDR_NOT_SUPPORTED, 1);
#endif

    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_128BIT_SERVICE_UUIDS, (void *)ble_midi_service_uuid128_le, 16);

    if (!adv_reconnect) {
        char *gap_name = ble_comm_get_gap_name();
        u8 name_len = strlen(gap_name);
        u8 vaild_len = ADV_RSP_PACKET_MAX - (offset + 2);
        if (name_len < vaild_len) {
            offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)gap_name, name_len);
            adv_name_ok = 1;
        } else {
            adv_name_ok = 0;
        }
    } else {
        adv_name_ok = 0;
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        puts("***trans_adv_data overflow!!!!!!\n");
        return -1;
    }
    log_info("trans_adv_data(%d):", offset);
    log_info_hexdump(buf, offset);
    trans_server_adv_config.adv_data_len = offset;
    trans_server_adv_config.adv_data = trans_adv_data;
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      组织rsp包数据，放入buff
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static int trans_make_set_rsp_data(u8 adv_reconnect)
{
    u8 offset = 0;
    u8 *buf = trans_scan_rsp_data;

#if RCSP_BTMATE_EN
    u8  tag_len = sizeof(user_tag_string);
    offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_MANUFACTURER_SPECIFIC_DATA, (void *)user_tag_string, tag_len);
#endif

    // 修改日期: 2026-04-03, 修改说明: 始终在扫描响应包中添加设备名称，确保Windows能识别BLE MIDI设备
    if (!adv_name_ok) {
        char *gap_name = ble_comm_get_gap_name();
        u8 name_len = strlen(gap_name);
        u8 vaild_len = ADV_RSP_PACKET_MAX - (offset + 2);
        if (name_len > vaild_len) {
            name_len = vaild_len;
        }
        offset += make_eir_packet_data(&buf[offset], offset, HCI_EIR_DATATYPE_COMPLETE_LOCAL_NAME, (void *)gap_name, name_len);
    }

    if (offset > ADV_RSP_PACKET_MAX) {
        puts("***rsp_data overflow!!!!!!\n");
        return -1;
    }

    log_info("rsp_data(%d):", offset);
    log_info_hexdump(buf, offset);
    trans_server_adv_config.rsp_data_len = offset;
    trans_server_adv_config.rsp_data = trans_scan_rsp_data;
    return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      配置广播参数
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note      开广播前配置都有效
 *              修改日期: 2026-02-15, 修改说明: 支持定向广播，实现自动回连
 */
/*************************************************************************************************/
static void trans_adv_config_set(void)
{
    int ret = 0;
    int adv_reconnect_flag = 0;

    // 参考 SDK 原版 hogp_adv_config_set 逻辑
    trans_server_adv_config.adv_interval = ADV_INTERVAL_MIN;
    trans_server_adv_config.adv_auto_do = 1;
    trans_server_adv_config.adv_channel = ADV_CHANNEL_ALL;

    if (PAIR_RECONNECT_ADV_EN && trans_pair_info.pair_flag && trans_pair_info.direct_adv_cnt) {
        // 有配对信息：使用配置的广播类型
        trans_server_adv_config.adv_type = trans_pair_reconnect_adv_type;

        if (trans_pair_reconnect_adv_type == ADV_IND) {
            // 普通广播：不设置定向地址，iOS会自动扫描并连接已配对设备
            memset(trans_server_adv_config.direct_address_info, 0, 7);
            trans_server_adv_config.adv_interval = PAIR_RECONNECT_ADV_INTERVAL;
            adv_reconnect_flag = 1;
            log_info("[ADV] RECONNECT using ADV_IND (iOS auto-connect)\n");
        } else {
            // 定向广播：设置目标地址
            memcpy(trans_server_adv_config.direct_address_info,
                   trans_pair_info.peer_address_info, 7);

            if (trans_pair_reconnect_adv_type != ADV_DIRECT_IND) {
                // 低占空比定向广播：设置快速间隔和超时
                trans_server_adv_config.adv_interval = PAIR_RECONNECT_ADV_INTERVAL;
                if (trans_pair_reconnect_adv_timeout) {
                    if (trans_pair_adv_timeout_id) {
                        sys_timeout_del(trans_pair_adv_timeout_id);
                        trans_pair_adv_timeout_id = 0;
                    }
                    // 超时后切换到普通广播
                    extern void trans_reconnect_adv_timeout(void);
                    trans_pair_adv_timeout_id = sys_timeout_add(0, trans_reconnect_adv_timeout,
                                                                 trans_pair_reconnect_adv_timeout);
                }
            }
            adv_reconnect_flag = 1;
            log_info("[ADV] RECONNECT type=%d addr:", trans_pair_reconnect_adv_type);
            put_buf(trans_server_adv_config.direct_address_info, 7);
        }
    } else {
        // 无配对信息或重试耗尽：普通广播
        trans_server_adv_config.adv_type = ADV_IND;
        memset(trans_server_adv_config.direct_address_info, 0, 7);
        log_info("[ADV] NORMAL adv\n");
    }

    printf("[ADV] type=%u interval=%u paired=%u cnt=%u\n",
           (unsigned)trans_server_adv_config.adv_type,
           (unsigned)trans_server_adv_config.adv_interval,
           (unsigned)trans_pair_info.pair_flag,
           (unsigned)trans_pair_info.direct_adv_cnt);

    ret |= trans_make_set_adv_data(adv_reconnect_flag);
    ret |= trans_make_set_rsp_data(adv_reconnect_flag);

    if (ret) {
        log_info("adv_setup_init fail!!!\n");
        return;
    }
    ble_gatt_server_set_adv_config(&trans_server_adv_config);
}

// 回连广播超时回调：超时后切换到普通广播（参考SDK原版__hogp_reconnect_low_timeout_handle）
void trans_reconnect_adv_timeout(void)
{
    trans_pair_adv_timeout_id = 0;
    if (trans_con_handle == 0 && ble_gatt_server_get_work_state() == BLE_ST_ADV) {
        log_info("[ADV] reconnect timeout, switch to normal adv\n");
        ble_gatt_server_adv_enable(0);
        trans_pair_info.direct_adv_cnt = 0;
        trans_adv_config_set();
        ble_gatt_server_adv_enable(1);
    } else {
        trans_pair_info.direct_adv_cnt = 0;
        trans_adv_config_set();
    }
}

/*************************************************************************************************/
/*!
 *  \brief      server init初始化
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void trans_server_init(void)
{
    log_info("%s", __FUNCTION__);

    trans_conn_pair_vm_do(&trans_pair_info, 0);
    if (trans_pair_info.pair_flag) {
        trans_pair_info.direct_adv_cnt = REPEAT_DIRECT_ADV_COUNT;
        log_info("[INIT] paired device found, will use directed adv\n");
        put_buf(trans_pair_info.peer_address_info, 7);
    } else {
        log_info("[INIT] no paired device, normal adv\n");
    }

    ble_gatt_server_set_profile(trans_profile_data, sizeof(trans_profile_data));
    trans_adv_config_set();
}

/*************************************************************************************************/
/*!
 *  \brief      断开连接
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void trans_disconnect(void)
{
    log_info("%s", __FUNCTION__);
    if (trans_con_handle) {
        ble_comm_disconnect(trans_con_handle);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      清除配对信息
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note       修改日期: 2026-02-15, 修改说明: 添加清除配对信息功能
 *              用于取消配对，下次启动将不会自动回连
 */
/*************************************************************************************************/
void trans_clear_pair_info(void)
{
    log_info("%s", __FUNCTION__);
    printf("\n[PAIR] ========== Clearing Pairing Info ==========\n");

    memset(&trans_pair_info, 0, sizeof(trans_pair_info));
    trans_pair_info.head_tag = BLE_VM_HEAD_TAG;
    trans_pair_info.tail_tag = BLE_VM_TAIL_TAG;
    trans_pair_info.pair_flag = 0;
    trans_pair_info.direct_adv_cnt = 0;

    trans_conn_pair_vm_do(&trans_pair_info, 1);  // 写入VM

    printf("[PAIR] ✓ Pairing info cleared from VM\n");
    printf("[PAIR] ✓ Next boot will use NORMAL advertising\n");
    printf("[PAIR] ✓ Device will wait for manual pairing\n");
    printf("[PAIR] ==================================================\n\n");
}


/*************************************************************************************************/
/*!
 *  \brief      协议栈初始化前调用
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/

void bt_ble_before_start_init(void)
{
    log_info("%s", __FUNCTION__);
    ble_comm_init(&trans_gatt_control_block);
}


/*************************************************************************************************/
/*!
 *  \brief
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
static void trans_test_send_data(void)
{
#if TEST_TRANS_CHANNEL_DATA
    static u32 count = 0;
    static u32 send_index;

    int i, ret = 0;
    int send_len = TEST_PAYLOAD_LEN;
    u32 time_index_max = 1000 / TEST_TRANS_TIMER_MS;

    if (!trans_con_handle) {
        return;
    }

    send_index++;

#if TEST_TRANS_NOTIFY_HANDLE
    do {
        if (ble_comm_att_check_send(trans_con_handle, send_len) && ble_gatt_server_characteristic_ccc_get(trans_con_handle, TEST_TRANS_NOTIFY_HANDLE + 1)) {
            count++;
            ret = ble_comm_att_send_data(trans_con_handle, TEST_TRANS_NOTIFY_HANDLE, &count, send_len, ATT_OP_AUTO_READ_CCC);
            if (!ret) {
                /* putchar('T'); */
                trans_send_test_count += send_len;
            }
        } else {
            break;
        }
    } while (ret == 0);
#endif

    if (send_index >= time_index_max) {
        if (trans_send_test_count) {
            log_info(">>>>>> send_rate= %d byte/s\n", trans_send_test_count);
        }
        send_index = 0;
        trans_send_test_count = 0;

        if (trans_recieve_test_count) {
            log_info("<<<<<<< recieve_rate= %d byte/s\n", trans_recieve_test_count);
            trans_recieve_test_count = 0;
        }
    }
#endif
}
/*************************************************************************************************/
/*!
 *  \brief      控制应答对方READ/WRITE行为的响应包RESPONSE的回复
 *
 *  \param      [in]流控使能 en: 1-停止收数 or 0-继续收数
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
int ble_trans_flow_enable(u8 en)
{
    int ret = -1;

#if ATT_DATA_RECIEVT_FLOW
    if (trans_con_handle) {
        att_server_flow_hold(trans_con_handle, en);
        ret = 0;
        log_info("ble_trans_flow_enable:%d\n", en);
    }
#endif

    return ret;
}

//for test
static void timer_trans_flow_test(void)
{
    static u8 sw = 0;
    if (trans_con_handle) {
        sw = !sw;
        ble_trans_flow_enable(sw);
    }
}

/*************************************************************************************************/
/*!
 *  \brief      模块初始化
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void bt_ble_init(void)
{
    log_info("%s\n", __FUNCTION__);
    log_info("ble_file: %s", __FILE__);

    /* BLE OTA mode: open RCSP GATT without forced pairing so a plain host
     * central can connect, and shrink the per-connection ATT buffers so the
     * link setup fits the AC632's tight RAM (a full-size 200-MTU / 512-cbuf
     * allocation on top of btstack+RCSP overflows the heap -> "no enough
     * physics memory" assert on connect). A small MTU only slows the RCSP
     * transfer, it doesn't break it. Must run before trans_server_init()
     * registers this GATT/SM config below. */
    if (md400_ota_boot_is_ble()) {
        trans_sm_init_config.slave_security_auto_req = 0;
        trans_sm_init_config.slave_set_wait_security = 0;
        trans_sm_init_config.authentication_req_flags = 0;
        trans_gatt_control_block.mtu_size = 64;
        trans_gatt_control_block.cbuffer_size = 128;
        /* Drop the GATT client role entirely: the OTA boot is a pure RCSP
         * peripheral, and the client machinery's buffers were the ~1.7 KB that
         * kept rcsp_init's protocol buffer from fitting ("Rom No enough physics
         * memory" assert, hardware-verified). */
        trans_gatt_control_block.client_config = NULL;
        printf("[MD400_OTA] BLE OTA mode: SM relaxed + ATT shrunk + client role off\r\n");
    }

#if DOUBLE_BT_SAME_NAME
    ble_comm_set_config_name(bt_get_local_name(), 0);
#else
    ble_comm_set_config_name(bt_get_local_name(), 1);
#endif
    trans_con_handle = 0;
    // 修改日期: 2026-02-10, 修改说明: 初始化统一连接句柄(避免client连接后分时切换判断失效)
    trans_comm_connected_handle = 0;
    // 修改日期: 2026-02-16, 修改说明: 初始化6个MIDI循环buffer(不预留1字节，按功能规划.md)
    memset(ble_midi_rx_buf, 0, sizeof(ble_midi_rx_buf));
    memset(ble_midi_tx_buf, 0, sizeof(ble_midi_tx_buf));
    ble_midi_rx_buf_wp = 0;
    ble_midi_rx_buf_rp = 0;
    ble_midi_rx_full = 0;
    ble_midi_tx_buf_rp = 0;
    ble_midi_tx_buf_wp = 0;
    ble_midi_tx_full = 0;
    ble_midi_rx_packet_bytes = 0;
    ble_midi_rx_queued_bytes = 0;
    ble_midi_rx_dropped_bytes = 0;
    ble_midi_tx_queued_bytes = 0;
    ble_midi_tx_written_bytes = 0;
    ble_midi_tx_packet_bytes = 0;
    ble_midi_tx_dropped_bytes = 0;
    ble_midi_tx_send_failures = 0;
    ble_midi_tx_ccc_disabled_drops = 0;
    ble_midi_connect_count = 0;
    ble_midi_disconnect_count = 0;

    memset(usb_midi_rx_buf, 0, sizeof(usb_midi_rx_buf));
    memset(usb_midi_tx_buf, 0, sizeof(usb_midi_tx_buf));
    usb_midi_rx_buf_wp = 0;
    usb_midi_rx_buf_rp = 0;
    usb_midi_rx_full = 0;
    usb_midi_tx_buf_rp = 0;
    usb_midi_tx_buf_wp = 0;
    usb_midi_tx_full = 0;
    usb_midi_tx_running_status = 0;
    usb_midi_tx_sysex_active = 0;

    memset(uart_midi_rx_buf, 0, sizeof(uart_midi_rx_buf));
    memset(uart_midi_tx_buf, 0, sizeof(uart_midi_tx_buf));
    uart_midi_rx_buf_wp = 0;
    uart_midi_rx_buf_rp = 0;
    uart_midi_rx_full = 0;
    uart_midi_tx_buf_rp = 0;
    uart_midi_tx_buf_wp = 0;
    uart_midi_tx_full = 0;
    trans_ble_midi_rx_cb = NULL;

    printf("[MD400_DIAG] encoder_cc_only=%u raw_usb_tx_disabled\r\n",
           (unsigned)CONFIG_MD400_ENCODER_CC_ONLY_TEST);
    md400_control_init();
    /* BLE OTA mode: skip the MS32 inter-MCU UART bridge, its pump timer, and the
     * MIDI-OUT UART — none are needed for the RCSP download and their timers +
     * UART IRQs only add load/interference during the BLE transfer. */
    if (!md400_ota_boot_is_ble()) {
        // 修改日期: 2026-02-16, 修改说明: 初始化UART(按工程配置引脚 31.25K 8N1)与大循环泵(三路RX->两路TX分发 + UART TX发送)
#if CONFIG_MD400_MS32_ENCODER_ENABLE || CONFIG_MD400_UART_MIDI_BRIDGE_ENABLE
        ble_midi_uart_init();
        ble_midi_uart_timer = sys_timer_add(NULL, ble_midi_uart_bridge_pump_tick, 2);
#else
        printf("[MD400_MS32] disabled for USB recovery test\r\n");
        ble_midi_uart_timer = sys_timer_add(NULL, ble_midi_uart_bridge_pump_tick, 2);
#endif
#if CONFIG_MD400_MIDI_OUT_ENABLE
        md400_midi_out_uart_init();
#endif
    }
    trans_server_init();

#if defined(TCFG_BT_STATUS_LED_ENABLE) && (TCFG_BT_STATUS_LED_ENABLE)
    bt_status_led_init();
#endif

#if CONFIG_BT_GATT_CLIENT_NUM
    if (!md400_ota_boot_is_ble()) {
        /* Client role is unregistered in the BLE OTA boot (see bt_ble_init). */
        trans_client_init();
    }
#endif

#if CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_SM_SUPPORT_ENABLE
    if (!md400_ota_boot_is_ble()) {
        trans_ios_services_init();
    }
#endif

#if ATT_DATA_RECIEVT_FLOW
    log_info("att_server_flow_enable\n");
    att_server_flow_enable(1);
//    sys_timer_add(0, timer_trans_flow_test, 5000);
#endif

    if (md400_ota_boot_is_ble()) {
        /* BLE OTA mode: force advertising on as a stable RCSP peripheral,
         * ignoring the persisted user BT toggle, so the host can connect and
         * drive the JL RCSP BLE_APP_UPDATA loader-download. Skip the dual-role
         * MIDI scheduler entirely — its central-scan windows drop advertising
         * and were what blocked the host from connecting. The RCSP AE00 GATT
         * service was already registered by trans_server_init() above. */
        printf("[MD400_OTA] BLE OTA mode: forcing RCSP peripheral advertising\r\n");
        ble_module_enable(1);
        trans_role_enter_peripheral();
        /* Cancel the 5s adv->central switch timeout so advertising stays up
         * continuously; the host needs a stable connectable peripheral, not the
         * dual-role adv/scan cycling. */
        trans_role_timeout_del();
#if RCSP_BTMATE_EN
        /* The pmalloc pool cannot fit rcsp's ~1.7 KB protocol buffer in this
         * boot (asserts "Rom No enough physics memory" even at boot time,
         * hardware-verified), so lend rcsp the preset store: preset/UI code is
         * skipped in the OTA boot, and a failed/finished OTA reboots before
         * any preset code touches it again. Then init rcsp up front so the
         * SDK's on-connect rcsp_init() no-ops via rcsp_run_flag. */
        extern void rcsp_set_prealloc_buffer(u8 *buf, u32 size);
        rcsp_set_prealloc_buffer((u8 *)&md400_store, sizeof(md400_store));
        rcsp_init();
        printf("[MD400_OTA] rcsp pre-init done (buf=%u) free_pages=%u\r\n",
               (unsigned)sizeof(md400_store),
               (unsigned)pmalloc_get_unused_pages());
#endif
    } else {
        /* apply the persisted user BT on/off (edit page encoder 2) */
        ble_module_enable(md400_bt_user_enable_load());
        md400_bt_indicator_sync();
        if (!md400_bt_user_enable) {
            printf("[MD400_BT] user BT off at boot\r\n");
        }

#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
        trans_role_start();
#endif
    }

#if TEST_TRANS_CHANNEL_DATA
    if (TEST_TRANS_TIMER_MS < 10) {
        sys_s_hi_timer_add(0, trans_test_send_data, TEST_TRANS_TIMER_MS);
    } else {
        sys_timer_add(0, trans_test_send_data, TEST_TRANS_TIMER_MS);
    }
#endif

}

/*************************************************************************************************/
/*!
 *  \brief      模块退出
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void bt_ble_exit(void)
{
    log_info("%s\n", __FUNCTION__);

#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
    trans_role_stop();
#endif

#if defined(TCFG_BT_STATUS_LED_ENABLE) && (TCFG_BT_STATUS_LED_ENABLE)
    bt_status_led_deinit();
#endif

    // 修改日期: 2026-02-10, 修改说明: 退出时停止BLE MIDI发送泵
    if (ble_midi_tx_timer) {
        sys_timer_del(ble_midi_tx_timer);
        ble_midi_tx_timer = 0;
    }

    // 修改日期: 2026-02-10, 修改说明: 退出时停止UART<->BLE MIDI桥接泵并关闭UART
    if (ble_midi_uart_timer) {
        sys_timer_del(ble_midi_uart_timer);
        ble_midi_uart_timer = 0;
    }
    if (ble_midi_uart_bus) {
        uart_dev_close((uart_bus_t *)ble_midi_uart_bus);
        ble_midi_uart_bus = NULL;
    }
#if CONFIG_MD400_MIDI_OUT_ENABLE
    if (md400_midi_out_uart_bus) {
        uart_dev_close((uart_bus_t *)md400_midi_out_uart_bus);
        md400_midi_out_uart_bus = NULL;
    }
#endif

#if CONFIG_BT_GATT_CLIENT_NUM
    trans_client_exit();
#endif

#if CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_SM_SUPPORT_ENABLE
    trans_ios_services_exit();
#endif

    ble_module_enable(0);
    // 修改日期: 2026-02-10, 修改说明: 模块退出时清理统一连接句柄
    trans_comm_connected_handle = 0;
    trans_con_handle = 0;
    ble_comm_exit();
}

/*************************************************************************************************/
/*!
 *  \brief      模块开发使能
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ble_module_enable(u8 en)
{
#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
    trans_role_enabled = !!en;
    if (!en) {
        trans_role_stop();
    }
#endif
    ble_comm_module_enable(en);
#if (CONFIG_BT_GATT_CLIENT_NUM && CONFIG_BT_GATT_SERVER_NUM)
    if (en && !trans_comm_connected_handle) {
        trans_role_enter_peripheral();
    }
#endif
}

/*************************************************************************************************/
/*!
 *  \brief      testbox 按键测试
 *
 *  \param      [in]
 *
 *  \return
 *
 *  \note
 */
/*************************************************************************************************/
void ble_server_send_test_key_num(u8 key_num)
{
    if (trans_con_handle) {
        if (get_remote_test_flag()) {
            ble_op_test_key_num(trans_con_handle, key_num);
        } else {
            log_info("-not conn testbox\n");
        }
    }
}


#endif
