/* Modified for Nexfoot; upstream notices are retained. */
#ifndef MD400_MS32_H
#define MD400_MS32_H

#include "typedef.h"

#define MD400_MS32_ENCODER_LEFT   0x01
#define MD400_MS32_ENCODER_RIGHT  0x02
#define MD400_MS32_ENCODER_PRESS  0x03
#define MD400_MS32_ENCODER_RELEASE 0x04

typedef void (*md400_ms32_encoder_event_cb_t)(u8 encoder, u8 event, int value);
typedef void (*md400_ms32_adc_event_cb_t)(u8 channel, u8 value);

void md400_ms32_init(void);
void md400_ms32_rx_bytes(const u8 *data, u16 len);
void md400_ms32_set_encoder_event_cb(md400_ms32_encoder_event_cb_t cb);
void md400_ms32_set_adc_event_cb(md400_ms32_adc_event_cb_t cb);
void md400_ms32_inject_encoder_event(u8 encoder, u8 event);

int md400_ms32_get_encoder_value(u8 encoder);
void md400_ms32_set_encoder_value(u8 encoder, u8 value);
u8 md400_ms32_get_encoder_pressed(u8 encoder);
u8 md400_ms32_get_adc_value(void);
void md400_ms32_get_adc_snapshot(u8 *channel, u8 *value, u32 *count,
                                  u8 *min_value, u8 *max_value);
void md400_ms32_get_pedal_mode_ack_snapshot(u8 *mode, u32 *count, u8 *seq);
void md400_ms32_print_debug_state(void);

void md400_led_init(void);
void md400_led_set_mask(u8 mask);
u8 md400_led_get_mask(void);

#endif
