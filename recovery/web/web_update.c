/* SPDX-License-Identifier: Apache-2.0 */
#include "common.h"
#include "delay.h"
#include "norflash.h"
#include "usb/usb_stack.h"
#include "usb/hid.h"
#include "wdt.h"
#include "printf.h"
extern void chip_reset(void);

#define APP_START 0x3000u
#define APP_END 0x48000u
#define SECTOR_SIZE 4096u
#define FOOTER_SIZE 16u
#define WEB_REPORT_SIZE 64u

enum {
    INFO = 1, BEGIN, DATA, FINISH, REBOOT
};
enum {
    OK, BAD_REQUEST, BAD_STATE, BAD_FLASH, VERIFY_FAILED, AMBIGUOUS_FOOTER
};

extern int usb_device_mode(usb_dev usb_id, u32 classes);
extern u32 usb_g_intr_read(usb_dev usb_id, u32 ep, u8 *data, u32 len, u32 block);
extern u32 hid_tx_data(struct usb_device_t *device, const u8 *data, u32 len);

static volatile u8 receive_pending;
static u8 sector[SECTOR_SIZE] __attribute__((aligned(4)));
static u8 footer[FOOTER_SIZE] __attribute__((aligned(4)));
static u32 image_end, next_sector, sector_used, expected_crc;
static u8 active, dirty, verified;

void set_new_data_flag(int flag)
{
    receive_pending = !!flag;
}

static u32 read_u32(const u8 *data)
{
    return (u32)data[0] | ((u32)data[1] << 8) | ((u32)data[2] << 16) | ((u32)data[3] << 24);
}

static void write_u32(u8 *out, u32 value)
{
    for (u8 i = 0; i < 4; ++i) {
        out[i] = value >> (8 * i);
    }
}

static u32 crc32_update(u32 crc, const u8 *data, u32 length)
{
    while (length--) {
        crc ^= *data++;
        for (u8 bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0);
        }
    }
    return crc;
}

static int matches(u32 address, const u8 *expected, u32 size)
{
    u8 actual[64] __attribute__((aligned(4)));
    for (u32 at = 0; at < size; at += sizeof(actual)) {
        norflash_read(actual, address + at, sizeof(actual));
        if (memcmp(actual, expected + at, sizeof(actual))) {
            return 0;
        }
    }
    return 1;
}

static u8 begin(u32 end, u32 crc)
{
    if (active || dirty) {
        return BAD_STATE;
    }
    if (end <= APP_START || end > APP_END || (end & (SECTOR_SIZE - 1))) {
        return BAD_REQUEST;
    }
    struct flash_info_t *info = get_flash_info();
    if (!info || info->id != 0xcd7113u || get_flash_capacity() != 0x80000u) {
        return BAD_FLASH;
    }
    // Prefer the footer at the new image's end. Otherwise use one
    // unambiguous existing ISDU from the application area.
    norflash_read(footer, end - FOOTER_SIZE, FOOTER_SIZE);
    if (memcmp(footer, "ISDU", 4)) {
        unsigned count = 0;
        memset(footer, 0xff, sizeof(footer));
        for (u32 at = APP_START + SECTOR_SIZE; at <= APP_END; at += SECTOR_SIZE) {
            u8 candidate[FOOTER_SIZE] __attribute__((aligned(4)));
            norflash_read(candidate, at - FOOTER_SIZE, FOOTER_SIZE);
            if (!memcmp(candidate, "ISDU", 4)) {
                memcpy(footer, candidate, sizeof(footer));
                ++count;
            }
        }
        if (count > 1) {
            return AMBIGUOUS_FOOTER;
        }
    }
    active = 1;
    verified = 0;
    image_end = end;
    next_sector = end - SECTOR_SIZE;
    sector_used = 0;
    expected_crc = crc;
    return OK;
}

static u8 append_data(u32 address, const u8 *data, u8 length)
{
    if (!active || next_sector < APP_START) {
        return BAD_STATE;
    }
    if (!length || length > 56 || address != next_sector + sector_used ||
        length > SECTOR_SIZE - sector_used) {
        return BAD_REQUEST;
    }
    memcpy(sector + sector_used, data, length);
    sector_used += length;
    if (sector_used != SECTOR_SIZE) {
        return OK;
    }
    if (next_sector + SECTOR_SIZE == image_end) {
        for (u8 i = 0; i < FOOTER_SIZE; ++i) {
            if (sector[SECTOR_SIZE - FOOTER_SIZE + i] != 0xff) {
                return BAD_REQUEST;
            }
        }
        memcpy(sector + SECTOR_SIZE - FOOTER_SIZE, footer, sizeof(footer));
    }

    if (!dirty) {
        // Invalidate the old application directory before changing its data.
        // The last sector written contains the new directory.
        dirty = 1;
        norflash_eraser(WINBOND_SECTOR_ERASE, APP_START);
        u8 header[64] __attribute__((aligned(4)));
        norflash_read(header, APP_START, sizeof(header));
        for (u8 i = 0; i < sizeof(header); ++i) {
            if (header[i] != 0xff) {
                return BAD_FLASH;
            }
        }
    }
    if (next_sector != APP_START && !matches(next_sector, sector, sizeof(sector))) {
        wdt_clr();
        norflash_eraser(WINBOND_SECTOR_ERASE, next_sector);
        norflash_write(sector, next_sector, sizeof(sector));
        if (next_sector != APP_START && !matches(next_sector, sector, sizeof(sector))) {
            return VERIFY_FAILED;
        }
    }
    next_sector -= SECTOR_SIZE;
    sector_used = 0;
    return OK;
}

static u8 finish(void)
{
    if (!active || next_sector >= APP_START || !dirty) {
        return BAD_STATE;
    }
    u8 block[64] __attribute__((aligned(4)));
    u32 crc = 0xffffffffu;
    for (u32 at = APP_START; at < image_end; at += sizeof(block)) {
        wdt_clr();
        if (at < APP_START + SECTOR_SIZE) {
            memcpy(block, sector + at - APP_START, sizeof(block));
        } else {
            norflash_read(block, at, sizeof(block));
        }
        if (at + sizeof(block) == image_end) {
            memset(block + sizeof(block) - FOOTER_SIZE, 0xff, FOOTER_SIZE);
        }
        crc = crc32_update(crc, block, sizeof(block));
    }
    if (~crc != expected_crc) {
        return VERIFY_FAILED;
    }
    // Commit the new directory only after all application bytes are verified.
    norflash_write(sector, APP_START, sizeof(sector));
    if (!matches(APP_START, sector, sizeof(sector))) {
        norflash_eraser(WINBOND_SECTOR_ERASE, APP_START);
        return VERIFY_FAILED;
    }
    active = 0;
    verified = 1;
    return OK;
}

void nexfoot_web_update(void)
{
    u8 request[WEB_REPORT_SIZE] __attribute__((aligned(4)));
    u8 reply[WEB_REPORT_SIZE] __attribute__((aligned(4)));
    usb_device_mode(0, HID_CLASS);
    for (;;) {
        wdt_clr();
        if (!receive_pending) {
            continue;
        }
        receive_pending = 0;
        if (usb_g_intr_read(0, HID_EP_OUT, request, sizeof(request), 0) != sizeof(request)) {
            continue;
        }
        memset(reply, 0, sizeof(reply));
        memcpy(reply, request, 4);
        u8 op = request[0];
        u8 length = request[1];
        u32 argument = read_u32(request + 4);
        u8 status = BAD_REQUEST;
        if (op == INFO && length == 0) {
            memcpy(reply + 8, "NEXWEB1", 7);
            write_u32(reply + 16, APP_START);
            write_u32(reply + 20, APP_END);
            u32 prefix_crc = 0xffffffffu;
            u8 block[64] __attribute__((aligned(4)));
            for (u32 at = 0; at < APP_START; at += sizeof(block)) {
                norflash_read(block, at, sizeof(block));
                prefix_crc = crc32_update(prefix_crc, block, sizeof(block));
            }
            write_u32(reply + 24, ~prefix_crc);
            status = OK;
        } else if (op == BEGIN && length == 4) {
            status = begin(argument, read_u32(request + 8));
        } else if (op == DATA) {
            status = append_data(argument, request + 8, length);
            if (status != OK) {
                active = 0;
            }
        } else if (op == FINISH && length == 0) {
            status = finish();
        } else if (op == REBOOT && length == 0) {
            status = !dirty || verified ? OK : BAD_STATE;
        }
        reply[4] = status;
        hid_tx_data(usb_id2device(0), reply, sizeof(reply));
        if (op == REBOOT && status == OK) {
            mdelay(100);
            chip_reset();
        }
    }
}
