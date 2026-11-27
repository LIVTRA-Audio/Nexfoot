/* Modified for Nexfoot; upstream notices are retained. */
#ifndef MD400_FLASH_STORE_H
#define MD400_FLASH_STORE_H
#include <stdint.h>

#define MD400_FS_BASE 0x5a000u
#define MD400_FS_SIZE 0x20000u
#define MD400_FS_SECTOR 4096u
#define MD400_FS_MAX_RECORD 1020u
#define MD400_FS_NOT_FOUND (-2)
#define MD400_FS_IO (-3)
#define MD400_FS_FULL (-4)
#define MD400_FS_BAD_RECORD (-5)

/* Offsets passed to this interface are relative to the reserved partition.
 * The caller serializes access. Reads and programs return zero on success.
 * Program must only clear bits; erase addresses are sector aligned. */
typedef struct {
    int (*read)(uint32_t, void *, uint32_t);
    int (*program)(uint32_t, const void *, uint32_t);
    int (*erase)(uint32_t);
} md400_fs_io;
typedef struct {
    const md400_fs_io *io;
    uint32_t sequence;
    uint32_t erases;
    /* Four-byte offset units; zero means absent. Matches SDK's 8-bit IDs. */
    uint16_t index[256];
} md400_fs;
int md400_fs_init(md400_fs *, const md400_fs_io *);
int md400_fs_get(md400_fs *, uint16_t, void *, uint16_t);
int md400_fs_put(md400_fs *, uint16_t, const void *, uint16_t);
#endif
