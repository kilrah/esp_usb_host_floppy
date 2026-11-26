/*
 * FAT file system on a USB floppy drive: ESP-IDF FatFs + VFS, with a cylinder cache and disk change handling
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "usb/floppy_host.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct floppy_fs *floppy_fs_handle_t;

typedef struct {
    const char *base_path; /*!< VFS mount point, e.g. "/floppy" */
    int max_files;         /*!< Maximum number of files open at the same time */
    uint8_t cache_cylinders; /*!< Cylinders kept in RAM (PSRAM if available), 18 KB each for a 1.44 MB disk. A miss
                                  reads the whole cylinder in one command, which costs little more than one sector
                                  (the drive waits for the sector to come around either way). 0: no cache, every
                                  sector access goes to the drive. */
} floppy_fs_config_t;

#define FLOPPY_FS_CONFIG_DEFAULT() { .base_path = "/floppy", .max_files = 4, .cache_cylinders = 4 }

typedef struct {
    char label[12];        /*!< Volume label (root directory entry, else boot sector), "" if none */
    uint32_t serial;       /*!< Volume serial number */
    uint8_t fat_type;      /*!< 12, 16 or 32 */
    uint64_t total_bytes;  /*!< Size of the data area */
    uint64_t free_bytes;
    floppy_media_info_t media; /*!< Capacity, write protection and geometry of the disk */
} floppy_fs_info_t;

/**
 * @brief Mount the FAT file system of the disk in the drive and register it with the VFS under base_path.
 *
 * Read-write, through the standard C library (fopen, fread, fwrite, opendir, stat, rename, unlink, mkdir...).
 * Writes are kept in the cylinder cache (write-back) and reach the disk when FatFs syncs: at the end of every
 * fclose(), fsync(), mkdir(), rename(), unlink()..., when a cache slot is reused, and on unmount. So nothing is lost
 * when the disk is removed after fclose(); changes not yet synced are lost (an error is logged). Long file names
 * need CONFIG_FATFS_LFN_HEAP or CONFIG_FATFS_LFN_STACK.
 *
 * The disk may be swapped while mounted: the change is detected (sense 06/28, or polling at most once per second),
 * open files of the old disk become invalid (EBADF / EIO) and the new disk is mounted at the next access.
 *
 * @return ESP_OK, ESP_ERR_FLOPPY_NO_MEDIUM, ESP_ERR_NOT_FOUND (no FAT file system: unformatted or foreign disk),
 *         ESP_ERR_NOT_SUPPORTED (sector size not supported by the FatFs configuration), ESP_ERR_INVALID_STATE
 *         (device already mounted, or base_path in use), or a drive error
 */
esp_err_t floppy_fs_mount(floppy_host_device_handle_t dev, const floppy_fs_config_t *config, floppy_fs_handle_t *fs);

/** Unmount and unregister from the VFS. Files still open become invalid. */
esp_err_t floppy_fs_unmount(floppy_fs_handle_t fs);

/** Label, serial number, size and free space. Counting free clusters reads the FAT (cached after the first time). */
esp_err_t floppy_fs_get_info(floppy_fs_handle_t fs, floppy_fs_info_t *info);

/**
 * @brief Create an empty FAT12 file system on a (low-level formatted) disk: a DOS "quick format".
 *
 * Writes a DOS-compatible boot sector, the FATs and an empty root directory for the standard formats (1.44 MB,
 * 720 KB, 1.2 MB, 1.25 MB PC-98, 2.88 MB, 640 KB, 360 KB). The disk must not be mounted. To low-level format first,
 * use floppy_host_format().
 *
 * @param label Volume label (up to 11 characters, upper-cased), or NULL / "" for none
 * @return ESP_OK, ESP_ERR_NOT_SUPPORTED (capacity is not a standard floppy format), ESP_ERR_INVALID_STATE (mounted),
 *         ESP_ERR_FLOPPY_WRITE_PROTECTED, ESP_ERR_FLOPPY_NO_MEDIUM, or a drive error
 */
esp_err_t floppy_fs_mkfs(floppy_host_device_handle_t dev, const char *label);

#ifdef __cplusplus
}
#endif
