/*
 * USB Host floppy driver - UFI command set
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "floppy_private.h"

static const char *TAG = "floppy_ufi";

/* UFI opcodes */
#define UFI_TEST_UNIT_READY       0x00
#define UFI_REQUEST_SENSE         0x03
#define UFI_FORMAT_UNIT           0x04
#define UFI_READ_FORMAT_CAPACITIES 0x23
#define UFI_READ_CAPACITY         0x25
#define UFI_READ_10               0x28
#define UFI_WRITE_10              0x2A
#define UFI_VERIFY                0x2F
#define UFI_MODE_SENSE_10         0x5A

#define MODE_PAGE_FLEXIBLE_DISK   0x05
#define MODE_PAGE_ALL             0x3F

static inline void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static inline uint16_t get_be16(const uint8_t *p)
{
    return (p[0] << 8) | p[1];
}

static inline uint32_t get_be24(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

static inline uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

/* ------------------------------------------------------------------------ */
/* Command execution with auto sense                                        */
/* ------------------------------------------------------------------------ */

static esp_err_t request_sense(floppy_device_t *dev, floppy_sense_t *sense)
{
    uint8_t cdb[6] = {UFI_REQUEST_SENSE, 0, 0, 0, 18, 0};
    uint8_t resp[18] = {0};
    size_t len = 0;
    esp_err_t err = floppy_transport_command(dev, cdb, sizeof(cdb), resp, sizeof(resp), true,
                                             FLOPPY_TIMEOUT_DEFAULT_MS, &len);
    if (err == FLOPPY_STATUS_UNKNOWN) {
        err = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "REQUEST SENSE failed");
    sense->key = len > 2 ? (resp[2] & 0x0F) : 0;
    sense->asc = len > 12 ? resp[12] : 0;
    sense->ascq = len > 13 ? resp[13] : 0;
    return ESP_OK;
}

/* Error code for a failed command's sense data */
static esp_err_t sense_to_err(floppy_device_t *dev, const floppy_sense_t *s)
{
    if (s->key == FLOPPY_SENSE_NOT_READY && s->asc == 0x3A) {
        return ESP_ERR_FLOPPY_NO_MEDIUM;
    }
    if (s->key == FLOPPY_SENSE_UNIT_ATTENTION && s->asc == 0x28) {
        dev->media_changes++;
        return ESP_ERR_FLOPPY_MEDIUM_CHANGED;
    }
    if (s->key == FLOPPY_SENSE_DATA_PROTECT || s->asc == 0x27) {
        return ESP_ERR_FLOPPY_WRITE_PROTECTED;
    }
    if (s->key == FLOPPY_SENSE_MEDIUM_ERROR) {
        return ESP_ERR_FLOPPY_MEDIUM_ERROR;
    }
    return ESP_ERR_FLOPPY_CHECK_CONDITION;
}

/* "Operation in progress" (07h), "format in progress" (04h), or the vendor-specific FFh a TEAC FD-05PUB reports
 * while writing */
static bool in_progress(const floppy_sense_t *s)
{
    return s->key == FLOPPY_SENSE_NOT_READY && s->asc == 0x04 &&
           (s->ascq == 0xFF || s->ascq == 0x07 || s->ascq == 0x04);
}

/* Execute a command with the device lock held. Fetches sense data when the command failed, or when the transport
 * doesn't report the status (CB). */
static esp_err_t run_cmd(floppy_device_t *dev, const uint8_t *cdb, size_t cdb_len,
                         void *data, size_t data_len, bool data_in, uint32_t timeout_ms, size_t *actual)
{
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t err = floppy_transport_command(dev, cdb, cdb_len, data, data_len, data_in, timeout_ms, actual);
    if (err == ESP_ERR_FLOPPY_CHECK_CONDITION || err == FLOPPY_STATUS_UNKNOWN) {
        floppy_sense_t sense = {0};
        esp_err_t sense_err = request_sense(dev, &sense);
        /* CB: the status is asked for as soon as the data is sent, while a write still waits for its sectors to
         * come around (up to a revolution, plus spin-up). Ask again until the drive is done. */
        TickType_t start = xTaskGetTickCount();
        while (err == FLOPPY_STATUS_UNKNOWN && sense_err == ESP_OK && in_progress(&sense) &&
                xTaskGetTickCount() - start < pdMS_TO_TICKS(timeout_ms)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            sense_err = request_sense(dev, &sense);
        }
        if (sense_err != ESP_OK) {
            err = err == FLOPPY_STATUS_UNKNOWN ? sense_err : err;
        } else if (err == FLOPPY_STATUS_UNKNOWN && sense.key <= FLOPPY_SENSE_RECOVERED_ERROR) {
            err = ESP_OK; /* CB: the command passed */
        } else {
            dev->last_sense = sense;
            err = sense_to_err(dev, &sense);
            ESP_LOGD(TAG, "CMD 0x%02x: sense %X/%02X/%02X (%s)", cdb[0], sense.key, sense.asc, sense.ascq,
                     floppy_sense_to_str(&sense));
        }
    }
    xSemaphoreGive(dev->lock);
    return err;
}

esp_err_t floppy_host_execute_cdb(floppy_host_device_handle_t handle,
                                  const uint8_t *cdb, size_t cdb_len,
                                  void *data, size_t data_len, bool data_in,
                                  uint32_t timeout_ms, size_t *actual_len)
{
    ESP_RETURN_ON_FALSE(handle && cdb, ESP_ERR_INVALID_ARG, TAG, "");
    ESP_RETURN_ON_FALSE(data_len <= FLOPPY_MAX_XFER_SIZE, ESP_ERR_INVALID_SIZE, TAG, "transfer too large");
    return run_cmd(handle, cdb, cdb_len, data, data_len, data_in,
                   timeout_ms ? timeout_ms : FLOPPY_TIMEOUT_DEFAULT_MS, actual_len);
}

esp_err_t floppy_host_get_last_sense(floppy_host_device_handle_t handle, floppy_sense_t *sense)
{
    ESP_RETURN_ON_FALSE(handle && sense, ESP_ERR_INVALID_ARG, TAG, "");
    *sense = handle->last_sense;
    return ESP_OK;
}

uint32_t floppy_host_get_media_change_count(floppy_host_device_handle_t handle)
{
    return handle ? handle->media_changes : 0;
}

const char *floppy_err_to_name(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_FLOPPY_CHECK_CONDITION: return "ESP_ERR_FLOPPY_CHECK_CONDITION";
    case ESP_ERR_FLOPPY_NO_MEDIUM:       return "ESP_ERR_FLOPPY_NO_MEDIUM";
    case ESP_ERR_FLOPPY_NOT_READY:       return "ESP_ERR_FLOPPY_NOT_READY";
    case ESP_ERR_FLOPPY_TRANSPORT:       return "ESP_ERR_FLOPPY_TRANSPORT";
    case ESP_ERR_FLOPPY_STALL:           return "ESP_ERR_FLOPPY_STALL";
    case ESP_ERR_FLOPPY_WRITE_PROTECTED: return "ESP_ERR_FLOPPY_WRITE_PROTECTED";
    case ESP_ERR_FLOPPY_MEDIUM_CHANGED:  return "ESP_ERR_FLOPPY_MEDIUM_CHANGED";
    case ESP_ERR_FLOPPY_MEDIUM_ERROR:    return "ESP_ERR_FLOPPY_MEDIUM_ERROR";
    default:                             return esp_err_to_name(err);
    }
}

const char *floppy_sense_to_str(const floppy_sense_t *s)
{
    switch (s->asc) {
    case 0x00: return s->key == 0 ? "No sense" : "No additional sense information";
    case 0x01: return "No index/sector signal";
    case 0x02: return "No seek complete";
    case 0x03: return "Write fault";
    case 0x04:
        switch (s->ascq) {
        case 0x01: return "Becoming ready";
        case 0x02: return "Initializing command required";
        case 0x04: return "Format in progress";
        default:   return "Not ready";
        }
    case 0x06: return "No reference position found (track 0)";
    case 0x08: return "Logical unit communication failure";
    case 0x0C: return "Write error";
    case 0x10: return "ID CRC error";
    case 0x11: return "Unrecovered read error";
    case 0x12: return "Address mark not found for ID field";
    case 0x13: return "Address mark not found for data field";
    case 0x14: return "Recorded entity not found";
    case 0x15: return "Seek error";
    case 0x1A: return "Parameter list length error";
    case 0x20: return "Invalid command opcode";
    case 0x21: return "Logical block address out of range";
    case 0x24: return "Invalid field in CDB";
    case 0x25: return "Logical unit not supported";
    case 0x26: return "Invalid field in parameter list";
    case 0x27: return "Write protected";
    case 0x28: return "Medium changed";
    case 0x29: return "Power on, reset or bus device reset";
    case 0x2E: return "Insufficient time for operation";
    case 0x30: return "Incompatible medium";
    case 0x31: return "Medium format corrupted / format failed";
    case 0x3A: return "Medium not present";
    case 0x40: return "Diagnostic failure";
    case 0x53: return "Media load or eject failed";
    case 0x57: return "Unable to recover table of contents";
    default:   return "Unknown";
    }
}

/* ------------------------------------------------------------------------ */
/* Unit state                                                               */
/* ------------------------------------------------------------------------ */

esp_err_t floppy_host_test_unit_ready(floppy_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[6] = {UFI_TEST_UNIT_READY};
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, FLOPPY_TIMEOUT_DEFAULT_MS, NULL);
}

esp_err_t floppy_host_wait_ready(floppy_host_device_handle_t handle, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    TickType_t start = xTaskGetTickCount();
    esp_err_t err;
    int attentions = 0;

    do {
        err = floppy_host_test_unit_ready(handle);
        if (err == ESP_OK || err == ESP_ERR_FLOPPY_NO_MEDIUM) {
            return err;
        }
        const floppy_sense_t *s = &handle->last_sense;
        if (err == ESP_ERR_FLOPPY_MEDIUM_CHANGED ||
                (err == ESP_ERR_FLOPPY_CHECK_CONDITION &&
                 (s->key == FLOPPY_SENSE_UNIT_ATTENTION || s->key == FLOPPY_SENSE_NO_SENSE))) {
            if (++attentions < 8) {
                continue; /* disk changed / reset: retry immediately */
            }
        } else if (err != ESP_ERR_FLOPPY_CHECK_CONDITION) {
            return err; /* transport error */
        } else if (s->key != FLOPPY_SENSE_NOT_READY) {
            ESP_LOGW(TAG, "Unexpected sense while waiting: %X/%02X/%02X (%s)", s->key, s->asc, s->ascq,
                     floppy_sense_to_str(s));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); /* becoming ready */
    } while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms));

    return ESP_ERR_FLOPPY_NOT_READY;
}

/* ------------------------------------------------------------------------ */
/* Capacity and geometry                                                    */
/* ------------------------------------------------------------------------ */

esp_err_t floppy_host_read_format_capacities(floppy_host_device_handle_t handle, floppy_format_capacities_t *caps)
{
    ESP_RETURN_ON_FALSE(handle && caps, ESP_ERR_INVALID_ARG, TAG, "");
    memset(caps, 0, sizeof(*caps));

    /* Header and the current/maximum capacity descriptor first, then exactly the length the header reports: some
     * USB bridges don't end the data phase with a short packet when the allocation length is larger than the
     * reply (seen with optical drives). */
    uint8_t resp[4 + 8 * (1 + FLOPPY_MAX_FORMATS)] = {0};
    uint8_t cdb[12] = {UFI_READ_FORMAT_CAPACITIES};
    size_t len = 0;
    put_be16(&cdb[7], 12);
    ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), resp, 12, true, FLOPPY_TIMEOUT_DEFAULT_MS, &len),
                        TAG, "READ FORMAT CAPACITIES header");
    ESP_RETURN_ON_FALSE(len >= 12 && resp[3] >= 8, ESP_ERR_INVALID_RESPONSE, TAG,
                        "READ FORMAT CAPACITIES too short (%u bytes, list %u)", (unsigned)len, resp[3]);
    size_t want = MIN((size_t)4 + (resp[3] & ~7), sizeof(resp));
    if (want > len) {
        put_be16(&cdb[7], want);
        ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), resp, want, true, FLOPPY_TIMEOUT_DEFAULT_MS, &len),
                            TAG, "READ FORMAT CAPACITIES");
    }
    size_t list_end = MIN(len, (size_t)4 + resp[3]);

    caps->current.blocks = get_be32(&resp[4]);
    caps->state = (floppy_media_state_t)(resp[8] & 0x03);
    caps->current.block_size = get_be24(&resp[9]);
    for (size_t off = 12; off + 8 <= list_end && caps->num_formats < FLOPPY_MAX_FORMATS; off += 8) {
        floppy_capacity_t *f = &caps->formats[caps->num_formats++];
        f->blocks = get_be32(&resp[off]);
        f->block_size = get_be24(&resp[off + 5]);
    }
    return ESP_OK;
}

esp_err_t floppy_host_read_capacity(floppy_host_device_handle_t handle, floppy_capacity_t *capacity)
{
    ESP_RETURN_ON_FALSE(handle && capacity, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[10] = {UFI_READ_CAPACITY};
    uint8_t resp[8] = {0};
    size_t len = 0;
    ESP_RETURN_ON_ERROR(run_cmd(handle, cdb, sizeof(cdb), resp, sizeof(resp), true, FLOPPY_TIMEOUT_DEFAULT_MS, &len),
                        TAG, "READ CAPACITY");
    ESP_RETURN_ON_FALSE(len == sizeof(resp), ESP_ERR_INVALID_RESPONSE, TAG, "READ CAPACITY: %u bytes", (unsigned)len);
    capacity->blocks = get_be32(&resp[0]) + 1;
    capacity->block_size = get_be32(&resp[4]);
    return ESP_OK;
}

/* Standard formats. UFI drives format 1.44 MB, 720 KB, and most 1.2 MB and 1.25 MB (PC-98); the others are only
 * here for the geometry. */
static const struct {
    uint32_t blocks;
    uint16_t block_size;
    floppy_geometry_t geometry;
} s_formats[] = {
    {2880, 512,  {80, 2, 18}}, /* 1.44 MB, 3.5" HD */
    {1440, 512,  {80, 2, 9}},  /* 720 KB, 3.5" DD */
    {2400, 512,  {80, 2, 15}}, /* 1.2 MB, 5.25" HD (IBM) */
    {1232, 1024, {77, 2, 8}},  /* 1.25 MB, PC-98 / Japanese 3-mode drives */
    {2464, 512,  {77, 2, 16}}, /* 1.25 MB with 512-byte sectors */
    {5760, 512,  {80, 2, 36}}, /* 2.88 MB, 3.5" ED */
    {3360, 512,  {80, 2, 21}}, /* 1.68 MB DMF (read only) */
    {1280, 512,  {80, 2, 8}},  /* 640 KB, PC-98 2DD */
    {720,  512,  {40, 2, 9}},  /* 360 KB, 5.25" DD */
};

esp_err_t floppy_geometry_from_capacity(const floppy_capacity_t *capacity, floppy_geometry_t *geometry)
{
    ESP_RETURN_ON_FALSE(capacity && geometry, ESP_ERR_INVALID_ARG, TAG, "");
    for (size_t i = 0; i < sizeof(s_formats) / sizeof(s_formats[0]); i++) {
        if (s_formats[i].blocks == capacity->blocks && s_formats[i].block_size == capacity->block_size) {
            *geometry = s_formats[i].geometry;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

/* MODE SENSE(10): header first, then the exact length (see READ FORMAT CAPACITIES) */
static esp_err_t mode_sense(floppy_device_t *dev, uint8_t page, uint8_t *buf, size_t size, size_t *len)
{
    uint8_t cdb[12] = {UFI_MODE_SENSE_10, 0, page};
    put_be16(&cdb[7], 8);
    ESP_RETURN_ON_ERROR(run_cmd(dev, cdb, sizeof(cdb), buf, 8, true, FLOPPY_TIMEOUT_DEFAULT_MS, len), TAG, "");
    ESP_RETURN_ON_FALSE(*len >= 4, ESP_ERR_INVALID_RESPONSE, TAG, "MODE SENSE: %u bytes", (unsigned)*len);
    size_t want = MIN((size_t)get_be16(buf) + 2, size);
    if (want > *len) {
        put_be16(&cdb[7], want);
        ESP_RETURN_ON_ERROR(run_cmd(dev, cdb, sizeof(cdb), buf, want, true, FLOPPY_TIMEOUT_DEFAULT_MS, len), TAG, "");
    }
    return ESP_OK;
}

esp_err_t floppy_host_get_media_info(floppy_host_device_handle_t handle, floppy_media_info_t *info)
{
    ESP_RETURN_ON_FALSE(handle && info, ESP_ERR_INVALID_ARG, TAG, "");
    memset(info, 0, sizeof(*info));
    ESP_RETURN_ON_ERROR(floppy_host_read_capacity(handle, &info->capacity), TAG, "");

    uint8_t buf[8 + 32] = {0}; /* header + flexible disk page */
    size_t len = 0;
    esp_err_t err = mode_sense(handle, MODE_PAGE_FLEXIBLE_DISK, buf, sizeof(buf), &len);
    if (err == ESP_ERR_FLOPPY_CHECK_CONDITION) {
        err = mode_sense(handle, MODE_PAGE_ALL, buf, 8, &len); /* at least the header, for write protection */
    }
    if (err == ESP_OK) {
        info->medium_type = buf[2];
        info->write_protected = (buf[3] & 0x80) != 0;
        size_t page_off = 8 + get_be16(&buf[6]); /* skip block descriptors (none on UFI) */
        const uint8_t *p = &buf[page_off];
        if (page_off + 12 <= len && (p[0] & 0x3F) == MODE_PAGE_FLEXIBLE_DISK) {
            info->transfer_rate_kbps = get_be16(&p[2]);
            info->geometry.heads = p[4];
            info->geometry.sectors_per_track = p[5];
            info->geometry.cylinders = get_be16(&p[8]);
        }
    } else if (err == ESP_ERR_FLOPPY_NO_MEDIUM || err == ESP_ERR_FLOPPY_MEDIUM_CHANGED || err == ESP_ERR_INVALID_STATE ||
               err == ESP_ERR_FLOPPY_TRANSPORT) {
        return err;
    } else {
        ESP_LOGD(TAG, "MODE SENSE not supported (%s)", floppy_err_to_name(err));
    }

    /* Trust the page only if it adds up to the capacity */
    const floppy_geometry_t *g = &info->geometry;
    if ((uint32_t)g->cylinders * g->heads * g->sectors_per_track != info->capacity.blocks) {
        if (g->sectors_per_track) {
            ESP_LOGD(TAG, "Flexible disk page %u/%u/%u doesn't match %" PRIu32 " blocks", g->cylinders, g->heads,
                     g->sectors_per_track, info->capacity.blocks);
        }
        memset(&info->geometry, 0, sizeof(info->geometry));
        floppy_geometry_from_capacity(&info->capacity, &info->geometry);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------------ */
/* Reading and writing                                                      */
/* ------------------------------------------------------------------------ */

static esp_err_t rw_sectors(floppy_device_t *dev, uint8_t opcode, uint32_t lba, uint32_t count, uint32_t block_size,
                            uint8_t *buf)
{
    ESP_RETURN_ON_FALSE(block_size >= 128 && block_size <= FLOPPY_MAX_XFER_SIZE, ESP_ERR_INVALID_ARG, TAG,
                        "bad block size %" PRIu32, block_size);
    const uint32_t max_per_cmd = FLOPPY_MAX_XFER_SIZE / block_size;
    bool in = opcode == UFI_READ_10;

    while (count) {
        uint32_t n = MIN(count, max_per_cmd);
        size_t bytes = n * block_size;
        size_t actual = 0;
        uint8_t cdb[12] = {opcode};
        put_be32(&cdb[2], lba);
        put_be16(&cdb[7], n);

        esp_err_t err = run_cmd(dev, cdb, sizeof(cdb), buf, bytes, in, FLOPPY_TIMEOUT_LONG_MS, &actual);
        if (err != ESP_OK) {
            if (err != ESP_ERR_INVALID_STATE && err != ESP_ERR_FLOPPY_NO_MEDIUM && err != ESP_ERR_FLOPPY_MEDIUM_CHANGED) {
                const floppy_sense_t *s = &dev->last_sense;
                ESP_LOGW(TAG, "%s LBA %" PRIu32 "+%" PRIu32 " failed: %s (sense %X/%02X/%02X %s)",
                         in ? "Read" : "Write", lba, n, floppy_err_to_name(err), s->key, s->asc, s->ascq,
                         floppy_sense_to_str(s));
            }
            return err;
        }
        if (actual != bytes) {
            ESP_LOGW(TAG, "Short %s at LBA %" PRIu32 ": %u of %u bytes", in ? "read" : "write", lba,
                     (unsigned)actual, (unsigned)bytes);
            return ESP_ERR_INVALID_SIZE;
        }
        lba += n;
        count -= n;
        buf += bytes;
    }
    return ESP_OK;
}

esp_err_t floppy_host_read(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count, uint32_t block_size,
                           void *buf)
{
    ESP_RETURN_ON_FALSE(handle && buf, ESP_ERR_INVALID_ARG, TAG, "");
    return rw_sectors(handle, UFI_READ_10, lba, count, block_size, buf);
}

esp_err_t floppy_host_write(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count, uint32_t block_size,
                            const void *buf)
{
    ESP_RETURN_ON_FALSE(handle && buf, ESP_ERR_INVALID_ARG, TAG, "");
    return rw_sectors(handle, UFI_WRITE_10, lba, count, block_size, (uint8_t *)buf);
}

esp_err_t floppy_host_verify(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count)
{
    ESP_RETURN_ON_FALSE(handle && count <= UINT16_MAX, ESP_ERR_INVALID_ARG, TAG, "");
    uint8_t cdb[12] = {UFI_VERIFY};
    put_be32(&cdb[2], lba);
    put_be16(&cdb[7], count);
    return run_cmd(handle, cdb, sizeof(cdb), NULL, 0, false, FLOPPY_TIMEOUT_LONG_MS, NULL);
}

/* ------------------------------------------------------------------------ */
/* Formatting                                                               */
/* ------------------------------------------------------------------------ */

esp_err_t floppy_host_format_track(floppy_host_device_handle_t handle, const floppy_capacity_t *capacity,
                                   uint16_t cylinder, uint8_t head)
{
    ESP_RETURN_ON_FALSE(handle && capacity && cylinder <= 0xFF && head <= 1, ESP_ERR_INVALID_ARG, TAG, "");
    /* UFI 4.3: FmtData = 1, CmpList = 0, Defect List Format = 7 */
    uint8_t cdb[12] = {UFI_FORMAT_UNIT, 0x17, (uint8_t)cylinder};
    uint8_t param[12] = {0};
    put_be16(&cdb[7], sizeof(param));
    /* Defect list header: FOV, DCRT (no certification), SingleTrack, side */
    param[1] = 0x80 | 0x20 | 0x10 | head;
    put_be16(&param[2], 8);
    /* Format descriptor: number of blocks, block length */
    put_be32(&param[4], capacity->blocks);
    param[9] = capacity->block_size >> 16;
    param[10] = capacity->block_size >> 8;
    param[11] = capacity->block_size;
    return run_cmd(handle, cdb, sizeof(cdb), param, sizeof(param), false, FLOPPY_TIMEOUT_LONG_MS, NULL);
}

esp_err_t floppy_host_format(floppy_host_device_handle_t handle, const floppy_capacity_t *capacity, bool verify,
                             floppy_format_progress_cb_t progress, void *arg)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "");
    floppy_capacity_t fmt;
    if (capacity) {
        fmt = *capacity;
    } else {
        floppy_format_capacities_t caps;
        ESP_RETURN_ON_ERROR(floppy_host_read_format_capacities(handle, &caps), TAG, "");
        ESP_RETURN_ON_FALSE(caps.state != FLOPPY_MEDIA_NONE, ESP_ERR_FLOPPY_NO_MEDIUM, TAG, "no disk");
        ESP_RETURN_ON_FALSE(caps.num_formats, ESP_ERR_NOT_SUPPORTED, TAG, "no formattable capacities");
        fmt = caps.formats[0];
        for (int i = 1; i < caps.num_formats; i++) {
            if ((uint64_t)caps.formats[i].blocks * caps.formats[i].block_size > (uint64_t)fmt.blocks * fmt.block_size) {
                fmt = caps.formats[i];
            }
        }
    }
    floppy_geometry_t g;
    ESP_RETURN_ON_FALSE(floppy_geometry_from_capacity(&fmt, &g) == ESP_OK, ESP_ERR_NOT_SUPPORTED, TAG,
                        "%" PRIu32 " x %" PRIu32 " is not a known floppy format", fmt.blocks, fmt.block_size);
    ESP_LOGI(TAG, "Formatting %" PRIu32 " x %" PRIu32 " bytes (%u cylinders, %u heads, %u sectors)", fmt.blocks,
             fmt.block_size, g.cylinders, g.heads, g.sectors_per_track);

    for (uint16_t c = 0; c < g.cylinders; c++) {
        for (uint8_t h = 0; h < g.heads; h++) {
            esp_err_t err = floppy_host_format_track(handle, &fmt, c, h);
            if (err == ESP_OK && verify) {
                err = floppy_host_verify(handle, ((uint32_t)c * g.heads + h) * g.sectors_per_track,
                                         g.sectors_per_track);
            }
            if (err != ESP_OK) {
                const floppy_sense_t *s = &handle->last_sense;
                ESP_LOGE(TAG, "Format failed at cylinder %u head %u: %s (sense %X/%02X/%02X %s)", c, h,
                         floppy_err_to_name(err), s->key, s->asc, s->ascq, floppy_sense_to_str(s));
                return err;
            }
            if (progress && !progress(c, h, &g, arg)) {
                ESP_LOGW(TAG, "Format aborted at cylinder %u head %u", c, h);
                return ESP_ERR_INVALID_STATE;
            }
        }
    }
    return ESP_OK;
}
