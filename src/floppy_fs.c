/*
 * FAT file system on a USB floppy drive: FatFs disk I/O driver, cylinder cache, VFS mount, FAT12 mkfs
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ff.h"
#include "diskio_impl.h"
#include "esp_vfs_fat.h"
#include "usb/floppy_fs.h"

static const char *TAG = "floppy_fs";

#define STATUS_POLL_MS      1000 /* TEST UNIT READY at most this often to notice a removed or swapped disk */
#define INIT_WAIT_READY_MS  3000

typedef struct {
    uint32_t cyl;   /* cylinder number (lba / sectors per cylinder) */
    uint32_t used;  /* last use, for LRU */
    bool valid;     /* the whole cylinder is in the slot */
    bool dirty;     /* written sectors not on the disk yet (bitmap in dirty_map) */
} cache_entry_t;

struct floppy_fs {
    floppy_host_device_handle_t dev;
    BYTE pdrv;
    char *base_path;
    FATFS *fatfs;
    SemaphoreHandle_t lock;    /* disk state and cache */

    /* Disk state, set by disk_initialize */
    bool ready;
    bool no_disk;              /* last initialize found no disk */
    uint32_t media_changes;    /* floppy_host_get_media_change_count() when initialized */
    floppy_media_info_t media;
    TickType_t last_poll;
    esp_err_t last_err;        /* last drive error, to explain a failed mount */

    /* Cylinder cache */
    uint8_t cache_cylinders;
    uint32_t slot_bytes;       /* size of a cache slot */
    uint8_t *cache_mem;
    cache_entry_t *cache;
    uint32_t *dirty_map;       /* map_words per slot */
    uint32_t map_words;
    uint32_t cache_clock;
    esp_err_t flush_err;       /* a write-back failed: reported by the next sync */
};

static floppy_fs_handle_t s_fs[FF_VOLUMES];
static portMUX_TYPE s_fs_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------------ */
/* Cylinder cache                                                           */
/* ------------------------------------------------------------------------ */

/*
 * The cache unit is a cylinder (both heads). The drive doesn't cache anything itself: every command waits for its
 * first sector to come around. Measured with a TEAC FD-05PUB and a 1.44 MB disk (300 rpm, 200 ms per revolution):
 *  - reading one track (18 sectors) 300 ms, one cylinder in one command 600 ms, as two commands 800 ms;
 *  - writing consecutive sectors one per command: 210 ms each, the next sector has just passed when it arrives.
 * So a cylinder is read with one command (costs little more than one sector), and writes are kept in the cache
 * (write-back) and written as one command per run of consecutive sectors when FatFs syncs: at the end of every
 * fclose(), fsync(), mkdir(), rename(), unlink()..., when a slot is reused, and on unmount.
 */
static uint32_t unit_sectors(floppy_fs_handle_t fs)
{
    return fs->media.geometry.sectors_per_track * MAX(fs->media.geometry.heads, 1);
}

static bool cache_usable(floppy_fs_handle_t fs)
{
    return fs->cache_mem && unit_sectors(fs) && unit_sectors(fs) * fs->media.capacity.block_size <= fs->slot_bytes;
}

static uint8_t *slot_data(floppy_fs_handle_t fs, int i)
{
    return fs->cache_mem + (size_t)i * fs->slot_bytes;
}

static uint32_t *slot_map(floppy_fs_handle_t fs, int i)
{
    return fs->dirty_map + (size_t)i * fs->map_words;
}

static inline bool map_get(const uint32_t *map, uint32_t s)
{
    return (map[s / 32] >> (s % 32)) & 1;
}

static inline void map_set(uint32_t *map, uint32_t s)
{
    map[s / 32] |= 1u << (s % 32);
}

/* Sectors of cylinder @p cyl that exist (the last cylinder may be short) */
static uint32_t cyl_sectors(floppy_fs_handle_t fs, uint32_t cyl)
{
    uint32_t lba = cyl * unit_sectors(fs);
    return lba < fs->media.capacity.blocks ? MIN(unit_sectors(fs), fs->media.capacity.blocks - lba) : 0;
}

static void slot_reset(floppy_fs_handle_t fs, int i, uint32_t cyl)
{
    fs->cache[i] = (cache_entry_t) {
        .cyl = cyl, .used = ++fs->cache_clock
    };
    memset(slot_map(fs, i), 0, fs->map_words * sizeof(uint32_t));
}

/* Drop everything, unwritten changes included */
static void cache_invalidate(floppy_fs_handle_t fs)
{
    for (int i = 0; fs->cache && fs->dirty_map && i < fs->cache_cylinders; i++) {
        slot_reset(fs, i, 0);
    }
}

static int cache_dirty_slots(floppy_fs_handle_t fs)
{
    int n = 0;
    for (int i = 0; fs->cache && i < fs->cache_cylinders; i++) {
        n += fs->cache[i].dirty;
    }
    return n;
}

/* Size the cache for the disk's cylinders (PSRAM preferred). No cache if the geometry is unknown. */
static void cache_setup(floppy_fs_handle_t fs)
{
    uint32_t bytes = unit_sectors(fs) * fs->media.capacity.block_size;
    if (!fs->cache_cylinders || !bytes) {
        return;
    }
    if (bytes > fs->slot_bytes) {
        heap_caps_free(fs->cache_mem);
        free(fs->dirty_map);
        fs->dirty_map = NULL;
        fs->slot_bytes = 0;
        fs->map_words = (unit_sectors(fs) + 31) / 32;
        size_t size = (size_t)bytes * fs->cache_cylinders;
        fs->cache_mem = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!fs->cache_mem) {
            fs->cache_mem = heap_caps_malloc(size, MALLOC_CAP_8BIT);
        }
        fs->dirty_map = calloc(fs->cache_cylinders * fs->map_words, sizeof(uint32_t));
        if (!fs->cache_mem || !fs->dirty_map) {
            ESP_LOGW(TAG, "No memory for a %u-cylinder cache (%u KB), running without", fs->cache_cylinders,
                     (unsigned)(size / 1024));
            heap_caps_free(fs->cache_mem);
            fs->cache_mem = NULL;
            return;
        }
        fs->slot_bytes = bytes;
    }
    cache_invalidate(fs);
}

static int cache_find(floppy_fs_handle_t fs, uint32_t cyl)
{
    for (int i = 0; i < fs->cache_cylinders; i++) {
        if ((fs->cache[i].valid || fs->cache[i].dirty) && fs->cache[i].cyl == cyl) {
            fs->cache[i].used = ++fs->cache_clock;
            return i;
        }
    }
    return -1;
}

/* Write a slot's unwritten sectors, one command per run of consecutive sectors */
static esp_err_t slot_flush(floppy_fs_handle_t fs, int i)
{
    cache_entry_t *e = &fs->cache[i];
    if (!e->dirty) {
        return ESP_OK;
    }
    uint32_t *map = slot_map(fs, i);
    uint32_t count = cyl_sectors(fs, e->cyl), lba = e->cyl * unit_sectors(fs), bs = fs->media.capacity.block_size;
    esp_err_t err = ESP_OK;
    for (uint32_t s = 0; s < count && err == ESP_OK;) {
        if (!map_get(map, s)) {
            s++;
            continue;
        }
        uint32_t end = s;
        while (end < count && map_get(map, end)) {
            end++;
        }
        err = floppy_host_write(fs->dev, lba + s, end - s, bs, slot_data(fs, i) + s * bs);
        s = end;
    }
    memset(map, 0, fs->map_words * sizeof(uint32_t));
    e->dirty = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Writing cylinder %" PRIu32 " failed (%s): changes lost", e->cyl, floppy_err_to_name(err));
        e->valid = false;
        if (fs->flush_err == ESP_OK) {
            fs->flush_err = err;
        }
    }
    return err;
}

static esp_err_t cache_flush(floppy_fs_handle_t fs)
{
    esp_err_t first = ESP_OK;
    for (int i = 0; fs->cache && i < fs->cache_cylinders; i++) {
        esp_err_t err = slot_flush(fs, i);
        if (first == ESP_OK) {
            first = err;
        }
    }
    return first;
}

/* The slot holding @p cyl, or the least recently used one, written back first and reset for @p cyl. A failed
 * write-back is reported by the next sync, not to the caller. */
static int slot_get(floppy_fs_handle_t fs, uint32_t cyl)
{
    int i = cache_find(fs, cyl);
    if (i >= 0) {
        return i;
    }
    int victim = 0;
    for (i = 0; i < fs->cache_cylinders; i++) {
        if (!fs->cache[i].valid && !fs->cache[i].dirty) {
            victim = i;
            break;
        }
        if (fs->cache[i].used < fs->cache[victim].used) {
            victim = i;
        }
    }
    slot_flush(fs, victim);
    slot_reset(fs, victim, cyl);
    return victim;
}

/* Read the whole cylinder into the slot (its unwritten sectors are written first) */
static esp_err_t slot_load(floppy_fs_handle_t fs, int i)
{
    cache_entry_t *e = &fs->cache[i];
    slot_flush(fs, i);
    e->valid = false;
    esp_err_t err = floppy_host_read(fs->dev, e->cyl * unit_sectors(fs), cyl_sectors(fs, e->cyl),
                                     fs->media.capacity.block_size, slot_data(fs, i));
    e->valid = err == ESP_OK;
    return err;
}

/* Are sectors first..first+n-1 of the slot in the cache? */
static bool slot_has(floppy_fs_handle_t fs, int i, uint32_t first, uint32_t n)
{
    if (fs->cache[i].valid) {
        return true;
    }
    const uint32_t *map = slot_map(fs, i);
    for (uint32_t s = first; s < first + n; s++) {
        if (!map_get(map, s)) {
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Disk state                                                               */
/* ------------------------------------------------------------------------ */

static bool is_media_err(esp_err_t err)
{
    return err == ESP_ERR_FLOPPY_NO_MEDIUM || err == ESP_ERR_FLOPPY_MEDIUM_CHANGED ||
           err == ESP_ERR_FLOPPY_NOT_READY || err == ESP_ERR_INVALID_STATE;
}

/* Forget the disk: FatFs re-mounts at the next access (disk_status returns STA_NOINIT) */
static void forget_disk(floppy_fs_handle_t fs, esp_err_t why)
{
    if (fs->ready) {
        int dirty = cache_dirty_slots(fs);
        if (dirty) {
            /* Not reported by a later sync: that would be the next disk's. Open files fail on their own. */
            ESP_LOGE(TAG, "Disk %s with unwritten changes on %d cylinder(s): lost (sync or close files first)",
                     why == ESP_ERR_FLOPPY_NO_MEDIUM ? "removed" : "changed", dirty);
        } else {
            ESP_LOGI(TAG, "Disk %s", why == ESP_ERR_FLOPPY_NO_MEDIUM ? "removed" : "changed");
        }
    }
    fs->ready = false;
    fs->no_disk = why == ESP_ERR_FLOPPY_NO_MEDIUM;
    cache_invalidate(fs);
}

static DSTATUS current_status(floppy_fs_handle_t fs)
{
    if (!fs->ready) {
        return STA_NOINIT | (fs->no_disk ? STA_NODISK : 0);
    }
    return fs->media.write_protected ? STA_PROTECT : 0;
}

/* A disk change seen by any command since the disk was initialized? */
static bool changed_since_init(floppy_fs_handle_t fs)
{
    if (fs->ready && floppy_host_get_media_change_count(fs->dev) != fs->media_changes) {
        forget_disk(fs, ESP_ERR_FLOPPY_MEDIUM_CHANGED);
        return true;
    }
    return !fs->ready;
}

static esp_err_t init_disk(floppy_fs_handle_t fs)
{
    fs->ready = false;
    fs->no_disk = false;
    fs->flush_err = ESP_OK;
    cache_invalidate(fs);
    esp_err_t err = floppy_host_wait_ready(fs->dev, INIT_WAIT_READY_MS);
    if (err == ESP_OK) {
        err = floppy_host_get_media_info(fs->dev, &fs->media);
    }
    if (err != ESP_OK) {
        fs->no_disk = err == ESP_ERR_FLOPPY_NO_MEDIUM;
        fs->last_err = err;
        return err;
    }
    uint32_t bs = fs->media.capacity.block_size;
    if (bs < FF_MIN_SS || bs > FF_MAX_SS || (bs & (bs - 1))) {
        ESP_LOGE(TAG, "Sector size %" PRIu32 " not supported by FatFs (%d..%d)", bs, FF_MIN_SS, FF_MAX_SS);
        fs->last_err = ESP_ERR_NOT_SUPPORTED;
        return ESP_ERR_NOT_SUPPORTED;
    }
    cache_setup(fs);
    fs->media_changes = floppy_host_get_media_change_count(fs->dev);
    fs->last_poll = xTaskGetTickCount();
    fs->ready = true;
    ESP_LOGI(TAG, "Disk: %" PRIu32 " x %" PRIu32 " bytes, %u/%u/%u%s", fs->media.capacity.blocks, bs,
             fs->media.geometry.cylinders, fs->media.geometry.heads, fs->media.geometry.sectors_per_track,
             fs->media.write_protected ? ", write protected" : "");
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* FatFs disk I/O                                                           */
/* ------------------------------------------------------------------------ */

static DSTATUS fd_initialize(BYTE pdrv)
{
    floppy_fs_handle_t fs = s_fs[pdrv];
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    init_disk(fs);
    DSTATUS st = current_status(fs);
    xSemaphoreGive(fs->lock);
    return st;
}

static DSTATUS fd_status(BYTE pdrv)
{
    floppy_fs_handle_t fs = s_fs[pdrv];
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    if (!changed_since_init(fs) && xTaskGetTickCount() - fs->last_poll >= pdMS_TO_TICKS(STATUS_POLL_MS)) {
        /* Nothing tells us when a disk is removed: ask */
        esp_err_t err = floppy_host_test_unit_ready(fs->dev);
        fs->last_poll = xTaskGetTickCount();
        if (is_media_err(err)) {
            forget_disk(fs, err);
        } else {
            changed_since_init(fs);
        }
    }
    DSTATUS st = current_status(fs);
    xSemaphoreGive(fs->lock);
    return st;
}

static DRESULT to_dresult(floppy_fs_handle_t fs, esp_err_t err)
{
    if (err == ESP_OK) {
        return RES_OK;
    }
    fs->last_err = err;
    if (is_media_err(err)) {
        forget_disk(fs, err);
        return RES_NOTRDY;
    }
    return err == ESP_ERR_FLOPPY_WRITE_PROTECTED ? RES_WRPRT : RES_ERROR;
}

static DRESULT fd_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    floppy_fs_handle_t fs = s_fs[pdrv];
    esp_err_t err = ESP_OK;
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    if (changed_since_init(fs)) {
        xSemaphoreGive(fs->lock);
        return RES_NOTRDY;
    }
    uint32_t bs = fs->media.capacity.block_size;
    if (!cache_usable(fs)) {
        err = floppy_host_read(fs->dev, sector, count, bs, buff);
    }
    while (cache_usable(fs) && count && err == ESP_OK) {
        uint32_t unit = unit_sectors(fs);
        uint32_t cyl = sector / unit;
        uint32_t first = sector % unit;
        uint32_t n = MIN(count, unit - first);
        int slot = slot_get(fs, cyl);
        if (!slot_has(fs, slot, first, n)) {
            err = slot_load(fs, slot);
            if (err != ESP_OK && !is_media_err(err)) {
                /* A bad sector somewhere on the cylinder: read just the requested ones (the slot was written
                 * back by slot_load, so the disk is up to date) */
                ESP_LOGD(TAG, "Cylinder %" PRIu32 " unreadable (%s), reading sectors %" PRIu32 "+%" PRIu32, cyl,
                         floppy_err_to_name(err), sector, n);
                err = floppy_host_read(fs->dev, sector, n, bs, buff);
                if (err != ESP_OK) {
                    break;
                }
            }
        }
        if (err == ESP_OK && slot_has(fs, slot, first, n)) {
            memcpy(buff, slot_data(fs, slot) + first * bs, n * bs);
        }
        sector += n;
        count -= n;
        buff += n * bs;
    }
    DRESULT res = to_dresult(fs, err);
    xSemaphoreGive(fs->lock);
    return res;
}

static DRESULT fd_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    floppy_fs_handle_t fs = s_fs[pdrv];
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    if (changed_since_init(fs)) {
        xSemaphoreGive(fs->lock);
        return RES_NOTRDY;
    }
    if (fs->media.write_protected) {
        xSemaphoreGive(fs->lock);
        return RES_WRPRT;
    }
    uint32_t bs = fs->media.capacity.block_size;
    esp_err_t err = ESP_OK;
    if (!cache_usable(fs)) {
        err = floppy_host_write(fs->dev, sector, count, bs, buff);
    }
    /* Write-back: into the cache, written to the disk at the next sync */
    while (cache_usable(fs) && count) {
        uint32_t unit = unit_sectors(fs);
        uint32_t first = sector % unit;
        uint32_t n = MIN(count, unit - first);
        int slot = slot_get(fs, sector / unit);
        cache_entry_t *e = &fs->cache[slot];
        uint32_t *map = slot_map(fs, slot);
        memcpy(slot_data(fs, slot) + first * bs, buff, n * bs);
        for (uint32_t s = first; s < first + n; s++) {
            map_set(map, s);
        }
        e->dirty = true;
        if (!e->valid && slot_has(fs, slot, 0, cyl_sectors(fs, e->cyl))) {
            e->valid = true; /* completely written */
        }
        sector += n;
        count -= n;
        buff += n * bs;
    }
    DRESULT res = to_dresult(fs, err);
    xSemaphoreGive(fs->lock);
    return res;
}

static DRESULT fd_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    floppy_fs_handle_t fs = s_fs[pdrv];
    DRESULT res = RES_OK;
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    if (!fs->ready && cmd != CTRL_SYNC) {
        res = RES_NOTRDY;
    } else {
        switch (cmd) {
        case CTRL_SYNC: {
            esp_err_t err = fs->ready ? cache_flush(fs) : ESP_OK;
            if (fs->flush_err != ESP_OK) {
                err = fs->flush_err; /* an earlier write-back failed, or changes were lost with the disk */
                fs->flush_err = ESP_OK;
            }
            res = to_dresult(fs, err);
            break;
        }
        case GET_SECTOR_COUNT:
            *(LBA_t *)buff = fs->media.capacity.blocks;
            break;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = fs->media.capacity.block_size;
            break;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            break;
#if FF_USE_TRIM
        case CTRL_TRIM: /* nothing to do on a floppy */
            break;
#endif
        default:
            res = RES_PARERR;
            break;
        }
    }
    xSemaphoreGive(fs->lock);
    return res;
}

static const ff_diskio_impl_t s_diskio = {
    .init = fd_initialize,
    .status = fd_status,
    .read = fd_read,
    .write = fd_write,
    .ioctl = fd_ioctl,
};

/* ------------------------------------------------------------------------ */
/* Mount                                                                    */
/* ------------------------------------------------------------------------ */

static bool is_mounted(floppy_host_device_handle_t dev)
{
    bool found = false;
    portENTER_CRITICAL(&s_fs_lock);
    for (int i = 0; i < FF_VOLUMES; i++) {
        found |= s_fs[i] && s_fs[i]->dev == dev;
    }
    portEXIT_CRITICAL(&s_fs_lock);
    return found;
}

static void free_fs(floppy_fs_handle_t fs)
{
    if (fs->lock) {
        vSemaphoreDelete(fs->lock);
    }
    heap_caps_free(fs->cache_mem);
    free(fs->dirty_map);
    free(fs->cache);
    free(fs->base_path);
    free(fs);
}

static esp_err_t fresult_to_err(floppy_fs_handle_t fs, FRESULT res)
{
    switch (res) {
    case FR_OK:            return ESP_OK;
    case FR_NO_FILESYSTEM: return ESP_ERR_NOT_FOUND;
    case FR_NOT_ENOUGH_CORE: return ESP_ERR_NO_MEM;
    case FR_NOT_READY:
    case FR_DISK_ERR:
        return fs->no_disk ? ESP_ERR_FLOPPY_NO_MEDIUM : fs->last_err != ESP_OK ? fs->last_err : ESP_FAIL;
    default:               return ESP_FAIL;
    }
}

esp_err_t floppy_fs_mount(floppy_host_device_handle_t dev, const floppy_fs_config_t *config, floppy_fs_handle_t *out)
{
    ESP_RETURN_ON_FALSE(dev && config && config->base_path && out, ESP_ERR_INVALID_ARG, TAG, "");
    ESP_RETURN_ON_FALSE(!is_mounted(dev), ESP_ERR_INVALID_STATE, TAG, "already mounted");

    BYTE pdrv = FF_DRV_NOT_USED;
    ESP_RETURN_ON_FALSE(ff_diskio_get_drive(&pdrv) == ESP_OK && pdrv != FF_DRV_NOT_USED, ESP_ERR_NO_MEM, TAG,
                        "no free FatFs drive (CONFIG_FATFS_VOLUME_COUNT)");

    esp_err_t ret = ESP_OK;
    floppy_fs_handle_t fs = calloc(1, sizeof(struct floppy_fs));
    ESP_RETURN_ON_FALSE(fs, ESP_ERR_NO_MEM, TAG, "");
    fs->dev = dev;
    fs->pdrv = pdrv;
    fs->cache_cylinders = config->cache_cylinders;
    fs->base_path = strdup(config->base_path);
    fs->lock = xSemaphoreCreateMutex();
    if (fs->cache_cylinders) {
        fs->cache = calloc(fs->cache_cylinders, sizeof(cache_entry_t));
    }
    if (!fs->base_path || !fs->lock || (fs->cache_cylinders && !fs->cache)) {
        free_fs(fs);
        return ESP_ERR_NO_MEM;
    }

    s_fs[pdrv] = fs;
    ff_diskio_register(pdrv, &s_diskio);
    char drv[3] = {(char)('0' + pdrv), ':', 0};
    const esp_vfs_fat_conf_t conf = {
        .base_path = config->base_path,
        .fat_drive = drv,
        .max_files = config->max_files > 0 ? config->max_files : 4,
    };
    ret = esp_vfs_fat_register(&conf, &fs->fatfs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VFS register at %s failed: %s", config->base_path, floppy_err_to_name(ret));
        ret = ret == ESP_ERR_INVALID_STATE ? ESP_ERR_INVALID_STATE : ret;
        fs->fatfs = NULL;
        goto fail;
    }

    FRESULT res = f_mount(fs->fatfs, drv, 1);
    if (res != FR_OK) {
        ret = fresult_to_err(fs, res);
        ESP_LOGW(TAG, "Mount failed: FatFs error %d (%s)", res, floppy_err_to_name(ret));
        f_mount(NULL, drv, 0);
        esp_vfs_fat_unregister_path(config->base_path);
        goto fail;
    }
    ESP_LOGI(TAG, "Mounted FAT%s at %s", fs->fatfs->fs_type == FS_FAT12 ? "12" : fs->fatfs->fs_type == FS_FAT16 ?
             "16" : "32", config->base_path);
    *out = fs;
    return ESP_OK;

fail:
    ff_diskio_unregister(pdrv);
    portENTER_CRITICAL(&s_fs_lock);
    s_fs[pdrv] = NULL;
    portEXIT_CRITICAL(&s_fs_lock);
    free_fs(fs);
    return ret;
}

esp_err_t floppy_fs_unmount(floppy_fs_handle_t fs)
{
    ESP_RETURN_ON_FALSE(fs, ESP_ERR_INVALID_ARG, TAG, "");
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    if (fs->ready && !changed_since_init(fs)) {
        cache_flush(fs);
    }
    xSemaphoreGive(fs->lock);
    char drv[3] = {(char)('0' + fs->pdrv), ':', 0};
    f_mount(NULL, drv, 0);
    esp_err_t err = esp_vfs_fat_unregister_path(fs->base_path);
    ff_diskio_unregister(fs->pdrv);
    portENTER_CRITICAL(&s_fs_lock);
    s_fs[fs->pdrv] = NULL;
    portEXIT_CRITICAL(&s_fs_lock);
    free_fs(fs);
    return err;
}

/* ------------------------------------------------------------------------ */
/* Info                                                                     */
/* ------------------------------------------------------------------------ */

static inline uint16_t le16(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

static inline uint32_t le32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void copy_label(char *dst, const uint8_t *src)
{
    memcpy(dst, src, 11);
    dst[11] = '\0';
    for (int i = 10; i >= 0 && dst[i] == ' '; i--) {
        dst[i] = '\0';
    }
    if (dst[0] == 0x05) {
        dst[0] = (char)0xE5; /* DOS escape for a leading 0xE5 */
    }
}

/* Read sectors through the cache (fs->lock held) */
static esp_err_t read_cached(floppy_fs_handle_t fs, uint32_t sector, uint32_t count, uint8_t *buf)
{
    if (changed_since_init(fs)) {
        return ESP_ERR_FLOPPY_MEDIUM_CHANGED;
    }
    xSemaphoreGive(fs->lock);
    DRESULT r = fd_read(fs->pdrv, buf, sector, count);
    xSemaphoreTake(fs->lock, portMAX_DELAY);
    return r == RES_OK ? ESP_OK : fs->last_err != ESP_OK ? fs->last_err : ESP_FAIL;
}

/* Label and serial from the boot sector; the label from the root directory entry if there is one (that's the one
 * DOS and Windows show and change). */
static esp_err_t read_label(floppy_fs_handle_t fs, floppy_fs_info_t *info)
{
    uint32_t bs = fs->media.capacity.block_size;
    uint8_t *buf = malloc(bs);
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "");
    esp_err_t err = read_cached(fs, 0, 1, buf);
    if (err == ESP_OK) {
        uint16_t bps = le16(&buf[11]);
        uint16_t rsvd = le16(&buf[14]);
        uint8_t nfats = buf[16];
        uint16_t root_entries = le16(&buf[17]);
        uint32_t spf = le16(&buf[22]);
        bool fat32 = spf == 0;
        const uint8_t *ext = fat32 ? &buf[0x40] : &buf[0x24]; /* extended BPB: drive, reserved, signature... */
        if (ext[2] == 0x29) {
            info->serial = le32(&ext[3]);
            copy_label(info->label, &ext[7]);
            if (!strcmp(info->label, "NO NAME")) {
                info->label[0] = '\0';
            }
        }
        /* FAT12/16 root directory */
        uint32_t root_sectors = bps ? ((uint32_t)root_entries * 32 + bps - 1) / bps : 0;
        uint32_t sector = rsvd + nfats * spf;
        for (uint32_t s = 0; !fat32 && bps == bs && s < root_sectors && err == ESP_OK; s++) {
            err = read_cached(fs, sector + s, 1, buf);
            for (uint32_t off = 0; err == ESP_OK && off + 32 <= bs; off += 32) {
                const uint8_t *e = &buf[off];
                if (e[0] == 0x00) {
                    s = root_sectors; /* end of directory */
                    break;
                }
                if (e[0] != 0xE5 && (e[11] & 0x08) && (e[11] & 0x0F) != 0x0F) {
                    copy_label(info->label, e);
                    s = root_sectors;
                    break;
                }
            }
        }
    }
    free(buf);
    return err;
}

esp_err_t floppy_fs_get_info(floppy_fs_handle_t fs, floppy_fs_info_t *info)
{
    ESP_RETURN_ON_FALSE(fs && info, ESP_ERR_INVALID_ARG, TAG, "");
    memset(info, 0, sizeof(*info));
    /* Through FatFs first: it re-mounts after a disk change */
    ESP_RETURN_ON_ERROR(esp_vfs_fat_info(fs->base_path, &info->total_bytes, &info->free_bytes), TAG, "free space");
    info->fat_type = fs->fatfs->fs_type == FS_FAT12 ? 12 : fs->fatfs->fs_type == FS_FAT16 ? 16 : 32;

    xSemaphoreTake(fs->lock, portMAX_DELAY);
    info->media = fs->media;
    esp_err_t err = read_label(fs, info);
    xSemaphoreGive(fs->lock);
    return err;
}

/* ------------------------------------------------------------------------ */
/* mkfs                                                                     */
/* ------------------------------------------------------------------------ */

/* DOS boot sector parameters of the standard formats */
static const struct {
    uint32_t blocks;
    uint16_t bps;
    uint8_t spc;        /* sectors per cluster */
    uint16_t root;      /* root directory entries */
    uint8_t media;      /* media descriptor */
    uint8_t spf;        /* sectors per FAT */
    uint8_t spt;
    uint8_t heads;
} s_bpb[] = {
    {2880, 512,  1, 224, 0xF0, 9, 18, 2}, /* 1.44 MB */
    {1440, 512,  2, 112, 0xF9, 3, 9,  2}, /* 720 KB */
    {2400, 512,  1, 224, 0xF9, 7, 15, 2}, /* 1.2 MB */
    {1232, 1024, 1, 192, 0xFE, 2, 8,  2}, /* 1.25 MB PC-98 */
    {5760, 512,  2, 240, 0xF0, 9, 36, 2}, /* 2.88 MB */
    {1280, 512,  2, 112, 0xFB, 2, 8,  2}, /* 640 KB */
    {720,  512,  2, 112, 0xFD, 2, 9,  2}, /* 360 KB */
};

/* Boot code of a non-system disk: print the message, wait for a key, reboot (int 19h) */
static const uint8_t s_boot_code[] = {
    0x0E, 0x1F, 0xBE, 0x5B, 0x7C, 0xAC, 0x22, 0xC0, 0x74, 0x0B, 0x56, 0xB4, 0x0E, 0xBB, 0x07, 0x00,
    0xCD, 0x10, 0x5E, 0xEB, 0xF0, 0x32, 0xE4, 0xCD, 0x16, 0xCD, 0x19, 0xEB, 0xFE,
};
static const char s_boot_msg[] = "This is not a bootable disk.\r\nInsert a system disk and press any key...\r\n";

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = v;
    p[1] = v >> 8;
}

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, v);
    put16(p + 2, v >> 16);
}

static void make_label(uint8_t dst[11], const char *label)
{
    memset(dst, ' ', 11);
    for (int i = 0; label && label[i] && i < 11; i++) {
        char c = toupper((unsigned char)label[i]);
        dst[i] = strchr("\"*+,./:;<=>?[\\]|", c) || (unsigned char)c < 0x20 ? '_' : c;
    }
}

esp_err_t floppy_fs_mkfs(floppy_host_device_handle_t dev, const char *label)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "");
    ESP_RETURN_ON_FALSE(!is_mounted(dev), ESP_ERR_INVALID_STATE, TAG, "unmount first");

    ESP_RETURN_ON_ERROR(floppy_host_wait_ready(dev, INIT_WAIT_READY_MS), TAG, "drive not ready");
    floppy_media_info_t media;
    ESP_RETURN_ON_ERROR(floppy_host_get_media_info(dev, &media), TAG, "media info");
    ESP_RETURN_ON_FALSE(!media.write_protected, ESP_ERR_FLOPPY_WRITE_PROTECTED, TAG, "write protected");

    int f = -1;
    for (size_t i = 0; i < sizeof(s_bpb) / sizeof(s_bpb[0]); i++) {
        if (s_bpb[i].blocks == media.capacity.blocks && s_bpb[i].bps == media.capacity.block_size) {
            f = i;
        }
    }
    ESP_RETURN_ON_FALSE(f >= 0, ESP_ERR_NOT_SUPPORTED, TAG, "%" PRIu32 " x %" PRIu32 " is not a standard format",
                        media.capacity.blocks, media.capacity.block_size);

    const uint16_t bps = s_bpb[f].bps;
    const uint32_t root_sectors = (s_bpb[f].root * 32 + bps - 1) / bps;
    const uint32_t sys_sectors = 1 + 2 * s_bpb[f].spf + root_sectors;
    uint8_t *buf = calloc(sys_sectors, bps);
    ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "");

    uint8_t vol_label[11];
    make_label(vol_label, label && label[0] ? label : "NO NAME");
    uint32_t serial = esp_random();

    /* Boot sector */
    uint8_t *b = buf;
    b[0] = 0xEB;
    b[1] = 0x3C;
    b[2] = 0x90;
    memcpy(&b[3], "MSDOS5.0", 8);
    put16(&b[11], bps);
    b[13] = s_bpb[f].spc;
    put16(&b[14], 1);                      /* reserved sectors */
    b[16] = 2;                             /* FATs */
    put16(&b[17], s_bpb[f].root);
    put16(&b[19], media.capacity.blocks);
    b[21] = s_bpb[f].media;
    put16(&b[22], s_bpb[f].spf);
    put16(&b[24], s_bpb[f].spt);
    put16(&b[26], s_bpb[f].heads);
    put32(&b[28], 0);                      /* hidden sectors */
    put32(&b[32], 0);                      /* 32-bit sector count (unused) */
    b[36] = 0x00;                          /* drive number */
    b[38] = 0x29;                          /* extended boot signature */
    put32(&b[39], serial);
    memcpy(&b[43], vol_label, 11);
    memcpy(&b[54], "FAT12   ", 8);
    memcpy(&b[62], s_boot_code, sizeof(s_boot_code));
    memcpy(&b[62 + sizeof(s_boot_code)], s_boot_msg, sizeof(s_boot_msg));
    b[510] = 0x55;
    b[511] = 0xAA;

    /* FATs: media descriptor and end-of-chain for the two reserved clusters */
    for (int i = 0; i < 2; i++) {
        uint8_t *fat = buf + (size_t)(1 + i * s_bpb[f].spf) * bps;
        fat[0] = s_bpb[f].media;
        fat[1] = 0xFF;
        fat[2] = 0xFF;
    }

    /* Root directory: volume label entry */
    if (label && label[0]) {
        uint8_t *e = buf + (size_t)(1 + 2 * s_bpb[f].spf) * bps;
        memcpy(e, vol_label, 11);
        e[11] = 0x08;
        DWORD t = get_fattime();
        put16(&e[22], t & 0xFFFF);
        put16(&e[24], t >> 16);
    }

    ESP_LOGI(TAG, "Writing FAT12 file system: %" PRIu32 " sectors, %u per cluster, %u root entries, label '%.11s'",
             media.capacity.blocks, s_bpb[f].spc, s_bpb[f].root, (const char *)vol_label);
    esp_err_t err = floppy_host_write(dev, 0, sys_sectors, bps, buf);
    free(buf);
    return err;
}
