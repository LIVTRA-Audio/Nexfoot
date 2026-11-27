/* Modified for Nexfoot; upstream notices are retained. */
#include "md400_flash_store.h"
#include <string.h>

#define MAGIC 0x3153464eu /* NFS1, little endian */
#define EMPTY 0xffffffffu
#define COMMIT 0x00000000u
#define SECTORS (MD400_FS_SIZE / MD400_FS_SECTOR)
#define ALIGN4(n) (((n) + 3u) & ~3u)
#define SECTOR_MAGIC 0x31534a4eu
#define COPYING 0xffffffffu
#define COPIED 0xfffffffeu
#define RETIRED 0xfffffffcu
typedef struct {
    uint32_t magic, source, crc, state;
} sector_header;

/* All fields are four-byte aligned. CRC includes identity, sequence and data.
 * A record only becomes visible after the final commit word is programmed. */
typedef struct {
    uint32_t magic;
    uint32_t sequence;
    uint16_t id;
    uint16_t length;
    uint32_t crc;
    uint32_t commit;
} record;
typedef char header_size_check[(sizeof(record) == 20) ? 1 : -1];

static uint32_t crc_more(uint32_t crc, const void *data, uint32_t len)
{
    const unsigned char *p = data;
    unsigned bit;
    while (len--) {
        crc ^= *p++;
        for (bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
        }
    }
    return crc;
}

static int sector_read(md400_fs *s, uint32_t at, sector_header *h)
{
    if (s->io->read(at, h, sizeof(*h))) return MD400_FS_IO;
    if (h->magic == EMPTY && h->source == EMPTY && h->crc == EMPTY && h->state == EMPTY) return 0;
    if (h->magic != SECTOR_MAGIC || h->crc != crc_more(EMPTY, h, 8) ||
        (h->source != EMPTY && (h->source >= MD400_FS_SIZE || h->source % MD400_FS_SECTOR))) return -1;
    return 1;
}

static int sector_start(md400_fs *s, uint32_t at, uint32_t source)
{
    sector_header h, check;
    h.magic = SECTOR_MAGIC;
    h.source = source;
    h.crc = crc_more(EMPTY, &h, 8);
    h.state = source == EMPTY ? RETIRED : COPYING;
    if (s->io->program(at, &h, sizeof(h))) return MD400_FS_IO;
    if (sector_read(s, at, &check) != 1 || memcmp(&h, &check, sizeof(h))) return MD400_FS_IO;
    return 0;
}

/* 1 = complete header, 0 = erased tail, -1 = torn/invalid tail, -3 = IO. */
static int header(md400_fs *s, uint32_t at, record *h)
{
    uint32_t left = MD400_FS_SECTOR - (at % MD400_FS_SECTOR);
    if (left < sizeof(*h)) {
        return -1;
    }
    if (s->io->read(at, h, sizeof(*h))) {
        return MD400_FS_IO;
    }
    if (h->magic == EMPTY && h->sequence == EMPTY &&
        h->id == 0xffff && h->length == 0xffff &&
        h->crc == EMPTY && h->commit == EMPTY) {
        return 0;
    }
    if (h->magic != MAGIC || !h->length || h->length > MD400_FS_MAX_RECORD ||
        !h->sequence || h->sequence == EMPTY || h->commit != COMMIT ||
        ALIGN4(sizeof(*h) + h->length) > left) {
        return -1;
    }
    return 1;
}

static int valid(md400_fs *s, uint32_t at, const record *h)
{
    unsigned char chunk[64];
    uint32_t pos = 0, n, crc = crc_more(EMPTY, h, 12);
    while (pos < h->length) {
        n = h->length - pos;
        if (n > sizeof(chunk)) n = sizeof(chunk);
        if (s->io->read(at + sizeof(*h) + pos, chunk, n)) return MD400_FS_IO;
        crc = crc_more(crc, chunk, n);
        pos += n;
    }
    return crc == h->crc;
}

/* Recovery fallback for a damaged record. Equal sequence duplicates occur
 * only during an interrupted relocation. */
static int scan_latest(md400_fs *s, uint16_t id, uint32_t *found, record *out)
{
    uint32_t sector, at, seq = 0;
    record h;
    sector_header sh;
    int r;
    for (sector = 0; sector < MD400_FS_SIZE; sector += MD400_FS_SECTOR) {
        r = sector_read(s, sector, &sh);
        if (r == MD400_FS_IO) return r;
        if (r != 1 || sh.state != RETIRED) continue;
        at = sector + sizeof(sh);
        while ((r = header(s, at, &h)) == 1) {
            if (h.id == id && h.sequence >= seq) {
                r = valid(s, at, &h);
                if (r < 0) return r;
                if (r) {
                    seq = h.sequence;
                    *found = at;
                    *out = h;
                }
            }
            at += ALIGN4(sizeof(h) + h.length);
            if (at >= sector + MD400_FS_SECTOR) break;
        }
        if (r == MD400_FS_IO) return r;
    }
    return seq ? 0 : MD400_FS_NOT_FOUND;
}

static int latest(md400_fs *s, uint16_t id, uint32_t *found, record *out)
{
    int r;
    if (id >= 256) return MD400_FS_BAD_RECORD;
    if (!s->index[id]) return MD400_FS_NOT_FOUND;
    *found = (uint32_t)s->index[id] * 4;
    r = header(s, *found, out);
    if (r == MD400_FS_IO) return r;
    if (r == 1 && out->id == id) {
        r = valid(s, *found, out);
        if (r == MD400_FS_IO) return r;
        if (r == 1) return 0;
    }
    return scan_latest(s, id, found, out);
}

int md400_fs_init(md400_fs *s, const md400_fs_io *io)
{
    uint32_t sector, at;
    record h, previous;
    sector_header sh;
    uint32_t state = RETIRED;
    int r;
    memset(s, 0, sizeof(*s));
    s->io = io;
    /* Complete an interrupted relocation before admitting any user write.
     * COPYING never erased its source; COPIED never reused the source yet. */
    for (sector = 0; sector < MD400_FS_SIZE; sector += MD400_FS_SECTOR) {
        r = sector_read(s, sector, &sh);
        if (r == MD400_FS_IO) return r;
        if (r == 1 && sh.source != EMPTY && sh.state != RETIRED) {
            if (sh.state == COPYING) {
                if (s->io->erase(sector)) return MD400_FS_IO;
            } else if (sh.state == COPIED) {
                if (s->io->erase(sh.source) || s->io->program(sector + 12, &state, 4)) return MD400_FS_IO;
            } else return MD400_FS_BAD_RECORD;
        }
    }
    for (sector = 0; sector < MD400_FS_SIZE; sector += MD400_FS_SECTOR) {
        r = sector_read(s, sector, &sh);
        if (r == MD400_FS_IO) return r;
        /* A sector header is verified before its first record is written.
         * A torn header (or interrupted discard erase) owns no unique data;
         * reclaim it so a power cut cannot consume the relocation reserve. */
        if (r == -1) {
            /* Refuse to discard a damaged sector header if committed records
             * survive beneath it. Torn first-header writes have no records. */
            at = sector + sizeof(sh);
            while (at + sizeof(h) <= sector + MD400_FS_SECTOR) {
                r = header(s, at, &h);
                if (r == MD400_FS_IO) return r;
                if (r == 1) {
                    r = valid(s, at, &h);
                    if (r == MD400_FS_IO) return r;
                    if (r == 1) return MD400_FS_BAD_RECORD;
                }
                at += 4;
            }
            if (s->io->erase(sector)) return MD400_FS_IO;
            continue;
        }
        if (r != 1) continue;
        at = sector + sizeof(sh);
        while ((r = header(s, at, &h)) == 1) {
            r = valid(s, at, &h);
            if (r == MD400_FS_IO) return r;
            if (r == 1 && h.id < 256) {
                if (!s->index[h.id] ||
                    (header(s, (uint32_t)s->index[h.id] * 4, &previous) == 1 &&
                     h.sequence >= previous.sequence)) s->index[h.id] = at / 4;
                if (h.sequence > s->sequence) s->sequence = h.sequence;
            }
            at += ALIGN4(sizeof(h) + h.length);
            if (at >= sector + MD400_FS_SECTOR) break;
        }
        if (r == MD400_FS_IO) return r;
    }
    return 0;
}

int md400_fs_get(md400_fs *s, uint16_t id, void *data, uint16_t capacity)
{
    uint32_t at;
    record h;
    int r = latest(s, id, &at, &h);
    if (r) return r;
    if (h.length > capacity) return MD400_FS_BAD_RECORD;
    if (s->io->read(at + sizeof(h), data, h.length)) return MD400_FS_IO;
    /* Also check the returned bytes: a task reader may have been preempted
     * between locating a record and copying it during sector relocation. */
    if (crc_more(crc_more(EMPTY, &h, 12), data, h.length) != h.crc)
        return MD400_FS_IO;
    return h.length;
}

/* Source is either caller RAM or an already committed flash record. */
static int append(md400_fs *s, uint32_t at, record *h,
                  const void *data, uint32_t source)
{
    unsigned char chunk[64];
    uint32_t pos = 0, n, committed = COMMIT;
    record verify;
    h->commit = EMPTY;
    if (s->io->program(at, h, sizeof(*h))) return MD400_FS_IO;
    while (pos < h->length) {
        n = h->length - pos;
        if (n > sizeof(chunk)) n = sizeof(chunk);
        if (data) memcpy(chunk, (const unsigned char *)data + pos, n);
        else if (s->io->read(source + sizeof(*h) + pos, chunk, n)) return MD400_FS_IO;
        if (s->io->program(at + sizeof(*h) + pos, chunk, n)) return MD400_FS_IO;
        pos += n;
    }
    if (valid(s, at, h) != 1) return MD400_FS_IO;
    if (s->io->program(at + 16, &committed, sizeof(committed))) return MD400_FS_IO;
    if (header(s, at, &verify) != 1 || valid(s, at, &verify) != 1) return MD400_FS_IO;
    return 0;
}

static int room(md400_fs *s, uint32_t need, uint32_t *target)
{
    uint32_t sector, at, blank = EMPTY, blanks = 0, tail = EMPTY;
    uint32_t victim = EMPTY, best = MD400_FS_SECTOR + 1, live, found;
    record h, newest;
    sector_header sh;
    uint32_t state;
    int r;
    for (sector = 0; sector < MD400_FS_SIZE; sector += MD400_FS_SECTOR) {
        r = sector_read(s, sector, &sh);
        if (r == MD400_FS_IO) return r;
        if (r == 0) { blank = sector; blanks++; continue; }
        if (r != 1) continue;
        at = sector + sizeof(sh);
        while ((r = header(s, at, &h)) == 1) {
            at += ALIGN4(sizeof(h) + h.length);
            if (at >= sector + MD400_FS_SECTOR) break;
        }
        if (r == MD400_FS_IO) return r;
        if (r == 0 && sector + MD400_FS_SECTOR - at >= need) {
            tail = at;
        }
    }
    if (tail != EMPTY) { *target = tail; return 0; }
    if (blanks > 1) {
        if (sector_start(s, blank, EMPTY)) return MD400_FS_IO;
        *target = blank + sizeof(sh);
        return 0;
    }
    if (!blanks) return MD400_FS_FULL;

    /* Keep one erased sector as a relocation reserve. Never erase live data
     * until every still-current record has a verified committed replacement. */
    for (sector = 0; sector < MD400_FS_SIZE; sector += MD400_FS_SECTOR) {
        if (sector == blank) continue;
        r = sector_read(s, sector, &sh);
        if (r == MD400_FS_IO) return r;
        if (r != 1) { best = 0; victim = sector; break; }
        at = sector + sizeof(sh);
        live = 0;
        while ((r = header(s, at, &h)) == 1) {
            r = latest(s, h.id, &found, &newest);
            if (r == MD400_FS_IO) return r;
            if (!r && found == at) live += ALIGN4(sizeof(h) + h.length);
            at += ALIGN4(sizeof(h) + h.length);
            if (at >= sector + MD400_FS_SECTOR) break;
        }
        if (r == MD400_FS_IO) return r;
        if (live < best) { best = live; victim = sector; }
    }
    if (victim == EMPTY || best + need > MD400_FS_SECTOR - sizeof(sh)) return MD400_FS_FULL;
    if (sector_start(s, blank, victim)) return MD400_FS_IO;
    at = victim + sizeof(sh);
    *target = blank + sizeof(sh);
    while ((r = header(s, at, &h)) == 1) {
        r = latest(s, h.id, &found, &newest);
        if (r == MD400_FS_IO) return r;
        if (!r && found == at) {
            if (append(s, *target, &h, NULL, at)) return MD400_FS_IO;
            *target += ALIGN4(sizeof(h) + h.length);
        }
        at += ALIGN4(sizeof(h) + h.length);
        if (at >= victim + MD400_FS_SECTOR) break;
    }
    if (r == MD400_FS_IO) return r;
    state = COPIED;
    if (s->io->program(blank + 12, &state, 4)) return MD400_FS_IO;
    /* Publish all verified copies before erasing their old addresses. A read
     * callback may preempt the writer at any point after this publication. */
    at = blank + sizeof(sh);
    while (at < *target && header(s, at, &h) == 1) {
        if (h.id < 256) s->index[h.id] = at / 4;
        at += ALIGN4(sizeof(h) + h.length);
    }
    if (s->io->erase(victim)) return MD400_FS_IO;
    state = RETIRED;
    if (s->io->program(blank + 12, &state, 4)) return MD400_FS_IO;
    s->erases++;
    return 0;
}

int md400_fs_put(md400_fs *s, uint16_t id, const void *data, uint16_t length)
{
    record h;
    uint32_t at;
    int r;
    if (id >= 256 || !length || length > MD400_FS_MAX_RECORD || !data) return MD400_FS_BAD_RECORD;
    if (s->sequence >= EMPTY - 1) return MD400_FS_FULL;
    r = room(s, ALIGN4(sizeof(h) + length), &at);
    if (r) return r;
    h.magic = MAGIC;
    h.sequence = ++s->sequence;
    h.id = id;
    h.length = length;
    h.crc = crc_more(crc_more(EMPTY, &h, 12), data, length);
    r = append(s, at, &h, data, 0);
    if (!r) s->index[id] = at / 4;
    return r ? r : length;
}
