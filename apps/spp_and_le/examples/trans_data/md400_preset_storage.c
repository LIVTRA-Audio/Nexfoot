/* Modified for Nexfoot; upstream notices are retained. */
#include "system/includes.h"
#include "md400_preset_storage.h"
#include "md400_flash_store.h"

/* SDK device operations retain a leading device argument even though LTO
 * removes it from internal calls. Do not infer these prototypes from the
 * optimized ABI. */
extern int norflash_read(void *device, void *buf, u32 len, u32 address);
extern int norflash_write(void *device, const void *buf, u32 len, u32 address);
extern int norflash_ioctl(void *device, u32 command, u32 argument);
static md400_fs store;
static OS_MUTEX lock;
static int ready;
static volatile u32 write_epoch;

static int read_flash(uint32_t at, void *buf, uint32_t len)
{
    int r;
    static unsigned reads;
    if (at > MD400_FS_SIZE || len > MD400_FS_SIZE - at) return -1;
    if (reads < 2) printf("[MD400_STORE] read %x %u\r\n", at, len);
    /* Use the SDK mapped read, with its complete device-operation ABI.
     * Avoid suspending SFC for every small journal header during a scan. */
    r = norflash_read(NULL, buf, len, MD400_FS_BASE + at);
    if (reads < 2) printf("[MD400_STORE] read result %d\r\n", r);
    reads++;
    return r == len ? 0 : -1;
}
static int program_flash(uint32_t at, const void *buf, uint32_t len)
{
    if (at > MD400_FS_SIZE || len > MD400_FS_SIZE - at) return -1;
    return norflash_write(NULL, buf, len, MD400_FS_BASE + at) == len ? 0 : -1;
}
static int erase_flash(uint32_t at)
{
    int r;
    if (at >= MD400_FS_SIZE || at % MD400_FS_SECTOR) return -1;
    printf("[MD400_STORE] erase %x\r\n", at);
    r = norflash_ioctl(NULL, 200, MD400_FS_BASE + at);
    printf("[MD400_STORE] erase result %d\r\n", r);
    return r;
}
static const md400_fs_io io = {read_flash, program_flash, erase_flash};

static int preset_id(u16 id)
{
    return (id >= 1 && id <= 18) || (id >= 22 && id <= 25) ||
           (id >= 128 && id <= 249);
}

static u32 content_hash(const void *data, u16 len)
{
    const u8 *bytes = data;
    u32 hash = 2166136261u;
    while (len--) hash = (hash ^ *bytes++) * 16777619u;
    return hash;
}

/* Called once on app_core before USB/BLE/display initialization.
 * Old VM records remain untouched, including pairing and global settings. */
int md400_preset_storage_init(void *scratch, u16 capacity)
{
    int r, migrated = 0, restart = 0, length;
    u32 expected;
    u16 id;
    if (ready) return ready > 0 ? 0 : ready;
    if (capacity < MD400_FS_MAX_RECORD) return MD400_FS_BAD_RECORD;
    os_mutex_create(&lock);
    printf("[MD400_STORE] recovery begin\r\n");
    r = md400_fs_init(&store, &io);
    printf("[MD400_STORE] recovery result=%d\r\n", r);
    if (r) { ready = r; return r; }
    /* A UBOOT layout migration seeds exact legacy records plus MIG1. Restore
     * SDK globals once, then reboot because user_cfg already cached the MAC
     * and pairing settings before app_main. The durable completion marker
     * prevents later firmware updates from restoring stale settings. */
    r = md400_fs_get(&store, 254, scratch, capacity);
    if (r == 4 && !memcmp(scratch, "MIG1", 4) &&
        md400_fs_get(&store, 255, scratch, capacity) == MD400_FS_NOT_FOUND) {
        for (id = 1; id < 254; id++) {
            if (preset_id(id)) continue;
            length = md400_fs_get(&store, id, scratch, capacity);
            if (length == MD400_FS_NOT_FOUND) continue;
            if (length < 0) { ready = length; return length; }
            expected = content_hash(scratch, length);
            if (syscfg_write(id, scratch, length) != length ||
                syscfg_read(id, scratch, length) != length ||
                content_hash(scratch, length) != expected) {
                ready = MD400_FS_IO;
                return ready;
            }
        }
        if (md400_fs_put(&store, 255, "MIG1", 4) != 4) {
            ready = MD400_FS_IO;
            return ready;
        }
        restart = 1;
    }
    for (id = 1; id <= 248; id++) {
        if (!preset_id(id)) continue;
        r = md400_fs_get(&store, id, scratch, capacity);
        if (r == MD400_FS_NOT_FOUND) {
            r = syscfg_read(id, scratch, capacity);
            if (r > 0) {
                r = md400_fs_put(&store, id, scratch, r);
                if (r < 0) { ready = r; return r; }
                migrated++;
                printf("[MD400_STORE] migrated id=%u\r\n", id);
            }
        } else if (r < 0) { ready = r; return r; }
    }
    ready = 1;
    printf("[MD400_STORE] journal base=5a000 size=20000 migrated=%d\r\n", migrated);
    return restart;
}

int md400_preset_storage_read(u16 id, void *data, u16 len)
{
    int r, attempt;
    u32 epoch;
    if (id == 250 || id == 251) {
        if (ready <= 0) return MD400_FS_IO;
        return md400_fs_get(&store, id, data, len);
    }
    if (!preset_id(id)) return syscfg_read(id, data, len);
    if (ready <= 0) return ready ? ready : MD400_FS_IO;
    /* USB/timer reads can preempt the app_core writer. Never wait for that
     * writer's mutex here: it cannot finish until the callback returns.
     * Committed journal records remain visible throughout relocation. */
    for (attempt = 0; attempt < 3; attempt++) {
        epoch = write_epoch;
        r = md400_fs_get(&store, id, data, len);
        if (epoch != write_epoch) continue;
        if (r == MD400_FS_NOT_FOUND) r = syscfg_read(id, data, len);
        return r;
    }
    return MD400_FS_IO;
}

int md400_preset_storage_write(u16 id, void *data, u16 len)
{
    int r;
    if (!preset_id(id) && id != 250 && id != 251) return syscfg_write(id, data, len);
    if (ready <= 0) return ready ? ready : MD400_FS_IO;
    os_mutex_pend(&lock, 0);
    write_epoch++;
    r = md400_fs_put(&store, id, data, len);
    write_epoch++;
    os_mutex_post(&lock);
    return r < 0 ? r : len;
}
