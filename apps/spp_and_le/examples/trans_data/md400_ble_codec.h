/* Modified for Nexfoot; upstream notices are retained. */
#ifndef MD400_BLE_CODEC_H
#define MD400_BLE_CODEC_H
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t running;
    uint8_t message[3];
    uint8_t used;
    uint8_t needed;
    uint8_t sysex;
} md400_ble_decoder_t;

/* Decode BLE timestamps before MIDI parsing. Preserve SysEx and running status
 * across ATT fragments; timestamps must never enter a NanoBridge checksum. */
static inline uint16_t md400_ble_decode(md400_ble_decoder_t *s,
        const uint8_t *in, uint16_t length, uint8_t *out, uint16_t capacity)
{
    uint16_t i = 1, n = 0;
    uint8_t boundary = 1;
    if (!in || length < 2 || !(in[0] & 0x80)) return 0;
    while (i < length) {
        uint8_t b = in[i++];
        if (boundary && (b & 0x80)) {
            boundary = 0;
            continue;
        }
        boundary = 0;
        if (s->sysex) {
            if ((b & 0x80) && i < length &&
                    (in[i] == 0xf7 || in[i] >= 0xf8)) b = in[i++];
            if (b < 0x80 || b == 0xf7 || b >= 0xf8) {
                if (n == capacity) break;
                out[n++] = b;
                if (b == 0xf7) { s->sysex = 0; boundary = 1; }
                continue;
            }
            /* A new status abandons an incomplete SysEx instead of retaining
             * stale command bytes across a malformed stream. */
            s->sysex = 0;
        }
        if (b >= 0xf8) {
            if (n == capacity) break;
            out[n++] = b;
            boundary = 1;
            continue;
        }
        if (b == 0xf0) {
            s->running = s->used = s->needed = 0;
            s->sysex = 1;
            if (n == capacity) break;
            out[n++] = b;
            continue;
        }
        if (b & 0x80) {
            s->used = 0;
            s->running = b < 0xf0 ? b : 0;
            if (b < 0xf0) s->needed = ((b & 0xe0) == 0xc0) ? 2 : 3;
            else if (b == 0xf1 || b == 0xf3) s->needed = 2;
            else if (b == 0xf2) s->needed = 3;
            else {
                if (n == capacity) break;
                out[n++] = b;
                s->needed = 0;
                boundary = 1;
                continue;
            }
            s->message[s->used++] = b;
            continue;
        }
        if (!s->used) {
            if (!s->running) continue;
            s->message[0] = s->running;
            s->used = 1;
            s->needed = ((s->running & 0xe0) == 0xc0) ? 2 : 3;
        }
        s->message[s->used++] = b;
        if (s->used == s->needed) {
            if (n + s->used > capacity) { s->used = 0; break; }
            memcpy(out + n, s->message, s->used);
            n += s->used;
            s->used = 0;
            boundary = 1;
        }
    }
    return n;
}

#define MD400_NB_STREAM_LIMIT 64
typedef struct {
    uint8_t data[MD400_NB_STREAM_LIMIT];
    uint8_t length;
    uint8_t passthrough;
    uint8_t discard;
} md400_nb_stream_t;
typedef void (*md400_nb_stream_emit_t)(void *, const uint8_t *, uint16_t);

/* Only intercept F0 7D 'N' 'B'. Other musical SysEx and real-time bytes pass
 * through unchanged, including a prefix split between BLE packets. */
static inline void md400_nb_stream_feed(md400_nb_stream_t *s,
        const uint8_t *data, uint16_t length, void *context,
        md400_nb_stream_emit_t control, md400_nb_stream_emit_t performance)
{
    static const uint8_t prefix[4] = {0xf0, 0x7d, 'N', 'B'};
    uint16_t i;
    for (i = 0; i < length; ++i) {
        uint8_t b = data[i];
        if (b >= 0xf8) { performance(context, &b, 1); continue; }
        if (b == 0xf0) { s->length = s->passthrough = s->discard = 0; }
        if (s->passthrough || (!s->length && b != 0xf0 && !s->discard)) {
            performance(context, &b, 1);
            if (b == 0xf7) s->passthrough = 0;
            continue;
        }
        if (!s->discard) {
            if (s->length == sizeof(s->data)) s->discard = 1;
            else {
                s->data[s->length++] = b;
                if (s->length <= sizeof(prefix) && b != prefix[s->length - 1]) {
                    performance(context, s->data, s->length);
                    s->length = 0;
                    s->passthrough = b != 0xf7;
                    continue;
                }
            }
        }
        if (b == 0xf7) {
            if (!s->discard && s->length >= 9) control(context, s->data, s->length);
            s->length = s->discard = 0;
        }
    }
}
#endif
