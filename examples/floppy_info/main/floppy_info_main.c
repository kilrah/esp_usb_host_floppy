/*
 * esp_usb_host_floppy example: drive and disk info, file listing, read speed, optional write test and format
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "usb/usb_host.h"
#include "usb/floppy_host.h"
#include "usb/floppy_fs.h"
#if CONFIG_EXAMPLE_FORCE_FULL_SPEED
#include "hal/usb_dwc_hal.h"
#include "hal/usb_dwc_ll.h"
#endif

static const char *TAG = "floppy_info";
#define MOUNT_POINT "/floppy"

typedef struct {
    bool connected;
    uint8_t address;                    /* connected */
    floppy_host_device_handle_t handle; /* disconnected */
} app_event_t;

static QueueHandle_t s_events;

/* Called from the driver's event task: don't send commands here, hand over to the main task */
static void floppy_event_cb(const floppy_host_event_t *event, void *arg)
{
    app_event_t e = {.connected = event->event == FLOPPY_DEVICE_CONNECTED};
    if (e.connected) {
        e.address = event->device.address;
    } else {
        e.handle = event->device.handle;
    }
    xQueueSend(s_events, &e, portMAX_DELAY);
}

static void usb_lib_task(void *arg)
{
    usb_host_config_t host_config = {.intr_flags = ESP_INTR_FLAG_LOWMED};
#if CONFIG_EXAMPLE_FORCE_FULL_SPEED
    /* Full speed only on the high-speed port, so that a full-speed drive behind a high-speed hub works (no
     * Transaction Translator support). With an ESP-IDF that has the HAL hook, the USB Host Library does it. */
#ifdef USB_DWC_HAL_INIT_HAS_CONFIG
    host_config.fsls_only = true;
#else
    host_config.root_port_unpowered = true;
#endif
#endif
    ESP_ERROR_CHECK(usb_host_install(&host_config));
#if CONFIG_EXAMPLE_FORCE_FULL_SPEED && !defined(USB_DWC_HAL_INIT_HAS_CONFIG)
    /* HCFG.FSLSSupp before the port is powered: the root port then never enters high speed. A port recovery (after
     * the hub or drive on the root port is unplugged) resets the core and clears it again. */
    usb_dwc_ll_hcfg_set_fsls_supp_only(USB_DWC_LL_GET_HW(0));
    ESP_ERROR_CHECK(usb_host_lib_set_root_port_power(true));
    ESP_LOGI(TAG, "High-speed port forced to full speed");
#endif

    const floppy_host_driver_config_t floppy_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = floppy_event_cb,
    };
    ESP_ERROR_CHECK(floppy_host_install(&floppy_config));
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static const char *media_state_str(floppy_media_state_t s)
{
    switch (s) {
    case FLOPPY_MEDIA_UNFORMATTED: return "unformatted";
    case FLOPPY_MEDIA_FORMATTED:   return "formatted";
    case FLOPPY_MEDIA_NONE:        return "no disk";
    default:                       return "?";
    }
}

static void print_capacity(const char *what, const floppy_capacity_t *c)
{
    floppy_geometry_t g;
    if (floppy_geometry_from_capacity(c, &g) == ESP_OK) {
        ESP_LOGI(TAG, "  %s %6" PRIu32 " x %4" PRIu32 " = %7" PRIu32 " KB  (%u cyl, %u heads, %u sectors)", what,
                 c->blocks, c->block_size, c->blocks * c->block_size / 1024, g.cylinders, g.heads, g.sectors_per_track);
    } else {
        ESP_LOGI(TAG, "  %s %6" PRIu32 " x %4" PRIu32 " = %7" PRIu32 " KB", what, c->blocks, c->block_size,
                 c->blocks * c->block_size / 1024);
    }
}

static void list_dir(const char *path, int depth)
{
    DIR *d = opendir(path);
    if (!d) {
        ESP_LOGE(TAG, "opendir %s failed", path);
        return;
    }
    struct dirent *de;
    char full[300];
    while ((de = readdir(d)) != NULL) {
        snprintf(full, sizeof(full), "%s/%s", path, de->d_name);
        struct stat st = {0};
        stat(full, &st);
        struct tm tm;
        localtime_r(&st.st_mtime, &tm);
        bool dir = de->d_type == DT_DIR;
        char size[16] = "<DIR>";
        if (!dir) {
            snprintf(size, sizeof(size), "%ld", (long)st.st_size);
        }
        ESP_LOGI(TAG, "  %*s%-*s %8s  %04d-%02d-%02d %02d:%02d", depth * 2, "", 40 - depth * 2, de->d_name,
                 size, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min);
        if (dir && depth < 3) {
            list_dir(full, depth + 1);
        }
    }
    closedir(d);
}

/* Largest file in the root directory, for the read test */
static bool find_largest(char *out, size_t out_len, long *size)
{
    DIR *d = opendir(MOUNT_POINT);
    if (!d) {
        return false;
    }
    struct dirent *de;
    *size = -1;
    char full[300];
    while ((de = readdir(d)) != NULL) {
        snprintf(full, sizeof(full), MOUNT_POINT "/%s", de->d_name);
        struct stat st;
        if (de->d_type != DT_DIR && stat(full, &st) == 0 && st.st_size > *size) {
            *size = st.st_size;
            strlcpy(out, full, out_len);
        }
    }
    closedir(d);
    return *size >= 0;
}

static void read_test(void)
{
    char path[300];
    long size;
    if (!find_largest(path, sizeof(path), &size)) {
        return;
    }
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "fopen %s failed", path);
        return;
    }
    static uint8_t buf[4096];
    size_t total = 0, n;
    uint32_t sum = 0;
    int64_t t0 = esp_timer_get_time();
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        for (size_t i = 0; i < n; i++) {
            sum += buf[i];
        }
        total += n;
    }
    int64_t us = esp_timer_get_time() - t0;
    bool err = ferror(f);
    fclose(f);
    ESP_LOGI(TAG, "Read %s: %u bytes in %lld ms (%lld KB/s), byte sum 0x%08" PRIx32 "%s", path, (unsigned)total,
             us / 1000, us ? (int64_t)total * 1000000 / 1024 / us : 0, sum, err ? ", READ ERROR" : "");
}

#if CONFIG_EXAMPLE_WRITE_TEST
static void write_test(void)
{
    const char *path = MOUNT_POINT "/ESPTEST.TXT";
    const size_t len = 20000;
    char *data = malloc(len), *back = malloc(len);
    if (!data || !back) {
        free(data);
        free(back);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = (i % 64 == 63) ? '\n' : 'A' + (i / 64 + i) % 26;
    }
    int64_t t0 = esp_timer_get_time();
    FILE *f = fopen(path, "wb");
    size_t written = f ? fwrite(data, 1, len, f) : 0;
    bool ok = f && fclose(f) == 0 && written == len;
    int64_t t1 = esp_timer_get_time();
    if (!ok) {
        ESP_LOGE(TAG, "Writing %s failed", path);
    } else {
        f = fopen(path, "rb");
        size_t got = f ? fread(back, 1, len, f) : 0;
        if (f) {
            fclose(f);
        }
        ok = got == len && memcmp(data, back, len) == 0;
        ESP_LOGI(TAG, "Write test: %u bytes written in %lld ms, read back %s", (unsigned)len, (t1 - t0) / 1000,
                 ok ? "OK" : "MISMATCH");
    }
    free(data);
    free(back);

    /* Directory operations, then clean up */
    const char *dir = MOUNT_POINT "/ESPDIR", *moved = MOUNT_POINT "/ESPDIR/Long file name.txt";
    struct stat st;
    bool dir_ok = ok && mkdir(dir, 0777) == 0 && rename(path, moved) == 0 && stat(moved, &st) == 0 &&
                  st.st_size == (off_t)len && stat(path, &st) != 0;
    bool clean = unlink(dir_ok ? moved : path) == 0 && (!dir_ok || rmdir(dir) == 0) && stat(dir, &st) != 0;
    ESP_LOGI(TAG, "mkdir/rename/stat: %s, unlink/rmdir: %s", dir_ok ? "OK" : "FAILED", clean ? "OK" : "FAILED");
}
#endif

#if CONFIG_EXAMPLE_FORMAT
static bool format_progress(uint16_t cyl, uint8_t head, const floppy_geometry_t *g, void *arg)
{
    if (head == g->heads - 1 && cyl % 10 == 9) {
        ESP_LOGI(TAG, "  cylinder %u/%u", cyl + 1, g->cylinders);
    }
    return true;
}

static void format_disk(floppy_host_device_handle_t dev)
{
    ESP_LOGW(TAG, "Formatting the disk");
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = floppy_host_format(dev, NULL, true, format_progress, NULL);
    if (err == ESP_OK) {
        err = floppy_fs_mkfs(dev, "ESP-FLOPPY");
    }
    ESP_LOGI(TAG, "Format: %s, %lld s", floppy_err_to_name(err), (esp_timer_get_time() - t0) / 1000000);
}
#endif

static floppy_fs_handle_t show_disk(floppy_host_device_handle_t dev)
{
    floppy_format_capacities_t caps;
    if (floppy_host_read_format_capacities(dev, &caps) == ESP_OK) {
        ESP_LOGI(TAG, "Disk: %s", media_state_str(caps.state));
        print_capacity(caps.state == FLOPPY_MEDIA_FORMATTED ? "current:" : "maximum:", &caps.current);
        for (int i = 0; i < caps.num_formats; i++) {
            print_capacity("can format:", &caps.formats[i]);
        }
    }
    floppy_media_info_t media;
    esp_err_t err = floppy_host_get_media_info(dev, &media);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Media info: %s", floppy_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Media: %" PRIu32 " blocks of %" PRIu32 " bytes, type 0x%02x, %u kbit/s, %u/%u/%u, %s",
                 media.capacity.blocks, media.capacity.block_size, media.medium_type, media.transfer_rate_kbps,
                 media.geometry.cylinders, media.geometry.heads, media.geometry.sectors_per_track,
                 media.write_protected ? "write protected" : "writable");
    }

#if CONFIG_EXAMPLE_FORMAT
    format_disk(dev);
#endif

    floppy_fs_handle_t fs = NULL;
    floppy_fs_config_t cfg = FLOPPY_FS_CONFIG_DEFAULT();
    cfg.base_path = MOUNT_POINT;
    int64_t t0 = esp_timer_get_time();
    err = floppy_fs_mount(dev, &cfg, &fs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", floppy_err_to_name(err));
        return NULL;
    }
    floppy_fs_info_t info;
    if (floppy_fs_get_info(fs, &info) == ESP_OK) {
        ESP_LOGI(TAG, "FAT%u volume '%s', serial %04" PRIX32 "-%04" PRIX32 ", %llu KB, %llu KB free (mount + info %lld ms)",
                 info.fat_type, info.label, info.serial >> 16, info.serial & 0xFFFF, info.total_bytes / 1024,
                 info.free_bytes / 1024, (esp_timer_get_time() - t0) / 1000);
    }
    t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "Files:");
    list_dir(MOUNT_POINT, 0);
    ESP_LOGI(TAG, "Listing took %lld ms", (esp_timer_get_time() - t0) / 1000);
    read_test();
#if CONFIG_EXAMPLE_WRITE_TEST
    if (!media.write_protected) {
        write_test();
    }
#endif
    return fs;
}

void app_main(void)
{
    s_events = xQueueCreate(4, sizeof(app_event_t));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 6, NULL);
    ESP_LOGI(TAG, "Waiting for a floppy drive...");

    floppy_host_device_handle_t dev = NULL;
    floppy_fs_handle_t fs = NULL;
    bool disk_shown = false;
    uint32_t changes = 0;
    while (true) {
        app_event_t e;
        if (xQueueReceive(s_events, &e, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (e.connected && !dev) {
                if (floppy_host_install_device(e.address, &dev) != ESP_OK) {
                    ESP_LOGW(TAG, "Device %d is not a usable floppy drive", e.address);
                    dev = NULL;
                    continue;
                }
                floppy_host_device_info_t info;
                floppy_host_get_device_info(dev, &info);
                ESP_LOGI(TAG, "Drive: %s %s (rev %s), VID 0x%04x PID 0x%04x, %s", info.vendor, info.product,
                         info.revision, info.vid, info.pid,
                         info.transport == FLOPPY_TRANSPORT_CBI ? "CBI" : info.transport == FLOPPY_TRANSPORT_CB ?
                         "CB" : "Bulk-Only");
                disk_shown = false;
            } else if (!e.connected && dev && e.handle == dev) {
                ESP_LOGI(TAG, "Drive disconnected");
                if (fs) {
                    floppy_fs_unmount(fs);
                    fs = NULL;
                }
                floppy_host_uninstall_device(dev);
                dev = NULL;
            }
            continue;
        }
        if (!dev) {
            continue;
        }
        /* Poll the drive: show the disk once it's ready, unmount it when it's removed. A disk swapped between two
         * polls is seen by the change counter (wait_ready() swallows the "medium changed" condition). */
        esp_err_t err = floppy_host_wait_ready(dev, 100);
        bool swapped = disk_shown && floppy_host_get_media_change_count(dev) != changes;
        if ((err == ESP_ERR_FLOPPY_NO_MEDIUM && disk_shown) || swapped) {
            ESP_LOGI(TAG, "Disk %s", swapped && err == ESP_OK ? "changed" : "removed");
            if (fs) {
                floppy_fs_unmount(fs);
                fs = NULL;
            }
            disk_shown = false;
        }
        if (err == ESP_OK && !disk_shown) {
            fs = show_disk(dev);
            changes = floppy_host_get_media_change_count(dev);
            disk_shown = true;
            /* Also pushes the report out: with the USB-Serial-JTAG console, the last line before a quiet period
             * can stay in the TX FIFO until more output comes */
            ESP_LOGI(TAG, "Watching for disk changes...");
        }
    }
}
