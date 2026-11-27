/* Modified for Nexfoot; upstream notices are retained. */
// binary representation
// attribute size in bytes (16), flags(16), handle (16), uuid (16/128), value(...)

#ifndef _BLE_TRANS_H
#define _BLE_TRANS_H

/*
 * 修改日期: 2026-02-10
 * 修改说明: 增加BLE MIDI client收发接口声明
 */

#include <stdint.h>
#include "app_config.h"
#include "gatt_common/le_gatt_common.h"

/*
 * 修改日期: 2026-02-10
 * 修改说明: 按项目规则更新BLE MIDI统一收发缓存尺寸(主机/从机共用)
 */
#define BLE_MIDI_RX_BUFF_SIZE 256
#define BLE_MIDI_TX_BUFF_SIZE 256
extern uint8_t ble_midi_rx_buf[BLE_MIDI_RX_BUFF_SIZE];
extern uint8_t ble_midi_tx_buf[BLE_MIDI_TX_BUFF_SIZE];
extern uint16_t ble_midi_rx_buf_rp;
extern uint16_t ble_midi_rx_buf_wp;
extern uint16_t ble_midi_tx_buf_rp;
extern uint16_t ble_midi_tx_buf_wp;

#define USB_MIDI_RX_BUFF_SIZE 256
#define USB_MIDI_TX_BUFF_SIZE 256
extern uint8_t usb_midi_rx_buf[USB_MIDI_RX_BUFF_SIZE];
extern uint8_t usb_midi_tx_buf[USB_MIDI_TX_BUFF_SIZE];
extern uint16_t usb_midi_rx_buf_rp;
extern uint16_t usb_midi_rx_buf_wp;
extern uint16_t usb_midi_tx_buf_rp;
extern uint16_t usb_midi_tx_buf_wp;

#define UART_MIDI_RX_BUFF_SIZE 256
#define UART_MIDI_TX_BUFF_SIZE 256
extern uint8_t uart_midi_rx_buf[UART_MIDI_RX_BUFF_SIZE];
extern uint8_t uart_midi_tx_buf[UART_MIDI_TX_BUFF_SIZE];
extern uint16_t uart_midi_rx_buf_rp;
extern uint16_t uart_midi_rx_buf_wp;
extern uint16_t uart_midi_tx_buf_rp;
extern uint16_t uart_midi_tx_buf_wp;

#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
#define USB1_MIDI_RX_BUFF_SIZE 256
#define USB1_MIDI_TX_BUFF_SIZE 256
extern uint8_t usb1_midi_rx_buf[USB1_MIDI_RX_BUFF_SIZE];
extern uint8_t usb1_midi_tx_buf[USB1_MIDI_TX_BUFF_SIZE];
extern uint16_t usb1_midi_rx_buf_rp;
extern uint16_t usb1_midi_rx_buf_wp;
extern uint16_t usb1_midi_tx_buf_rp;
extern uint16_t usb1_midi_tx_buf_wp;
#endif

extern const gatt_client_cfg_t trans_client_init_cfg;
void trans_client_init(void);
void trans_client_exit(void);
int trans_client_search_remote_profile(u16 conn_handle);
int trans_client_search_remote_stop(u16 conn_handle);

int trans_ble_midi_set_rx_handler(void (*cb)(u16 conn_handle, const u8 *data, u16 len));
int trans_ble_midi_send(const u8 *data, u16 len);
void md400_control_inject_event(u8 event, u8 control, u8 repeat);
void md400_display_boot_state_prepare(void);
u8 md400_runtime_preset_store_version(void);
void md400_runtime_fw_version(u8 *major, u8 *minor, u8 *patch);
u8 md400_runtime_target_mask(void);
u8 md400_runtime_feature_flags(void);
u8 md400_runtime_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_assignment_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_action_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_pedal_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_set_encoder_assignment(u8 encoder, const u8 *fields, u8 len);
u8 md400_runtime_set_pedal_assignment(const u8 *fields, u8 len);
u8 md400_runtime_midi_out_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_midi_out_trace_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_usb_host_state_snapshot(u8 *payload, u8 max_len);
#if CONFIG_MD400_USB1_HOST_MIDI_ENABLE
void md400_on_usb0_slave_online(void);
void md400_usb1_host_rearm(void);
void md400_usb1_host_schedule_recover(void);
void md400_usb1_host_note_send_result(int ret);
#endif
u8 md400_runtime_led_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_ble_midi_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_display_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_edit_key_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_protocol_state_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_gesture_assignment_count(void);
u8 md400_runtime_gesture_assignment_snapshot(u8 index, u8 *payload, u8 max_len);
u8 md400_runtime_set_gesture_assignment(u8 event, u8 control, const u8 *fields, u8 len);
u8 md400_runtime_save_active_preset(void);
u8 md400_runtime_load_preset_from_vm(u8 preset_number);
u8 md400_runtime_factory_reset(void);
u8 md400_runtime_event_log_snapshot(u8 *payload, u8 max_len);
u8 md400_runtime_get_preset_label(u8 preset_number, u8 *label, u8 *label_len, u8 max_len);
u8 md400_runtime_set_preset_label(u8 preset_number, const u8 *label, u8 label_len);
u8 md400_runtime_exit_edit(void);

#define MD400_CONTROL_EVENT_ENCODER_LEFT    0x01
#define MD400_CONTROL_EVENT_ENCODER_RIGHT   0x02
#define MD400_CONTROL_EVENT_ENCODER_PRESS   0x03
#define MD400_CONTROL_EVENT_ENCODER_RELEASE 0x04
#define MD400_CONTROL_EVENT_ENCODER_SHORT   0x05
#define MD400_CONTROL_EVENT_ENCODER_LONG    0x06
#define MD400_CONTROL_EVENT_ENCODER_DOUBLE  0x07
#define MD400_CONTROL_EVENT_COMBO_PRESS     0x08
#define MD400_CONTROL_EVENT_COMBO_SHORT     0x09
#define MD400_CONTROL_EVENT_COMBO_LONG      0x0A
#define MD400_CONTROL_EVENT_EDIT_SHORT      0x10
#define MD400_CONTROL_EVENT_EDIT_LONG       0x11
#define MD400_CONTROL_EVENT_EDIT_DOUBLE     0x12
#define MD400_CONTROL_EVENT_PEDAL_VALUE     0x13
/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI统一接收入口(用于client notify与server write共用)
 */
void trans_ble_midi_on_rx(u16 conn_handle, const u8 *data, u16 len);

#if CONFIG_BT_GATT_CLIENT_NUM
/*
 * 修改日期: 2026-02-10
 * 修改说明: BLE MIDI client侧立即发送接口(用于统一发送泵内部调用)
 */
u8 trans_ble_midi_client_is_ready(void);
int trans_ble_midi_client_send_now(const u8 *data, u16 len);
void trans_ble_midi_client_diag(u16 *conn_handle, u16 *io_handle,
                                u16 *write_handle, u16 *notify_handle);
/*
 * 修改日期: 2026-02-10
 * 修改说明: 增加分时互斥角色控制接口(主机扫描/从机广播互斥时使用)
 */
void trans_client_set_midi_scan_connect_enable(u8 en);
#endif

/*
 * 修改日期: 2026-02-10
 * 修改说明: 统一client/server连接状态(任一角色连接后停止分时切换)
 */
void trans_comm_connected_handle_on_connect(u16 conn_handle);
void trans_comm_connected_handle_on_disconnect(u16 conn_handle);
u16 trans_comm_connected_handle_get(void);

void trans_ios_services_init(void);
void trans_ios_services_exit(void);

u8 md400_usb_mask_update_schedule(void);
#endif
