/*
 * USB Host driver for USB floppy disk drives (UFI)
 *
 * Talks to floppy drives that implement the USB Mass Storage UFI command set (subclass 0x04), over
 * Control/Bulk/Interrupt (CBI), Control/Bulk (CB) or Bulk-Only (BOT) transport. Gives sector access, the
 * formattable capacities, geometry, write protection, disk change detection and low-level formatting.
 * See floppy_fs.h for FAT file access.
 *
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Constants                                                                */
/* ------------------------------------------------------------------------ */

#define FLOPPY_MAX_FORMATS            16   /*!< Formattable capacities kept by floppy_host_read_format_capacities() */

/* Error codes (in addition to standard esp_err_t values) */
#define ESP_ERR_FLOPPY_BASE             0x7B000
#define ESP_ERR_FLOPPY_CHECK_CONDITION  (ESP_ERR_FLOPPY_BASE + 1) /*!< Command failed, sense data available */
#define ESP_ERR_FLOPPY_NO_MEDIUM        (ESP_ERR_FLOPPY_BASE + 2) /*!< No disk in the drive (sense 02/3A) */
#define ESP_ERR_FLOPPY_NOT_READY        (ESP_ERR_FLOPPY_BASE + 3) /*!< Drive did not become ready in time */
#define ESP_ERR_FLOPPY_TRANSPORT        (ESP_ERR_FLOPPY_BASE + 4) /*!< USB transport failure */
#define ESP_ERR_FLOPPY_STALL            (ESP_ERR_FLOPPY_BASE + 5) /*!< Endpoint stalled (internal) */
#define ESP_ERR_FLOPPY_WRITE_PROTECTED  (ESP_ERR_FLOPPY_BASE + 6) /*!< Write protected disk (sense 07/27) */
#define ESP_ERR_FLOPPY_MEDIUM_CHANGED   (ESP_ERR_FLOPPY_BASE + 7) /*!< Disk changed (sense 06/28): command not run */
#define ESP_ERR_FLOPPY_MEDIUM_ERROR     (ESP_ERR_FLOPPY_BASE + 8) /*!< Unreadable sector, CRC error, ... (sense key 03) */

/* Common sense keys */
#define FLOPPY_SENSE_NO_SENSE         0x00
#define FLOPPY_SENSE_RECOVERED_ERROR  0x01
#define FLOPPY_SENSE_NOT_READY        0x02
#define FLOPPY_SENSE_MEDIUM_ERROR     0x03
#define FLOPPY_SENSE_HARDWARE_ERROR   0x04
#define FLOPPY_SENSE_ILLEGAL_REQUEST  0x05
#define FLOPPY_SENSE_UNIT_ATTENTION   0x06
#define FLOPPY_SENSE_DATA_PROTECT     0x07

/* ------------------------------------------------------------------------ */
/* Types                                                                    */
/* ------------------------------------------------------------------------ */

typedef struct floppy_host_device *floppy_host_device_handle_t;

typedef enum {
    FLOPPY_DEVICE_CONNECTED,    /*!< A USB device with a floppy (UFI) interface was connected */
    FLOPPY_DEVICE_DISCONNECTED, /*!< An installed device was disconnected; call floppy_host_uninstall_device() */
} floppy_host_event_id_t;

typedef struct {
    floppy_host_event_id_t event;
    union {
        uint8_t address;                    /*!< USB address, for FLOPPY_DEVICE_CONNECTED */
        floppy_host_device_handle_t handle; /*!< Device handle, for FLOPPY_DEVICE_DISCONNECTED */
    } device;
} floppy_host_event_t;

typedef void (*floppy_host_event_cb_t)(const floppy_host_event_t *event, void *arg);

typedef struct {
    bool create_background_task; /*!< Create a task that handles USB client events */
    unsigned task_priority;      /*!< Priority of the background task */
    size_t stack_size;           /*!< Stack size of the background task */
    BaseType_t core_id;          /*!< Core of the background task (tskNO_AFFINITY allowed) */
    floppy_host_event_cb_t callback; /*!< Connection/disconnection callback (required) */
    void *callback_arg;
    bool accept_scsi_devices;    /*!< Also report Bulk-Only mass storage devices with the SCSI transparent (0x06) or
                                      SFF-8070i (0x05) subclass, as some floppy drives use. install_device() then
                                      accepts them if INQUIRY reports a removable direct access device. Off by
                                      default because that matches card readers and USB sticks too. */
} floppy_host_driver_config_t;

/** How commands travel to the drive */
typedef enum {
    FLOPPY_TRANSPORT_CBI = 0, /*!< Control/Bulk/Interrupt (protocol 0x00): status on the interrupt endpoint */
    FLOPPY_TRANSPORT_CB,      /*!< Control/Bulk (protocol 0x01): status by REQUEST SENSE */
    FLOPPY_TRANSPORT_BOT,     /*!< Bulk-Only (protocol 0x50) */
} floppy_transport_t;

typedef struct {
    uint8_t key;  /*!< Sense key */
    uint8_t asc;  /*!< Additional sense code */
    uint8_t ascq; /*!< Additional sense code qualifier */
} floppy_sense_t;

typedef struct {
    uint16_t vid;
    uint16_t pid;
    uint8_t usb_address;
    uint8_t interface_subclass; /*!< 0x04 = UFI, 0x05 = SFF-8070i, 0x06 = SCSI transparent */
    floppy_transport_t transport;
    uint8_t peripheral_type;    /*!< INQUIRY peripheral device type, 0x00 (direct access) for floppy drives */
    bool removable;
    char vendor[9];
    char product[17];
    char revision[5];
} floppy_host_device_info_t;

/** A capacity: number of blocks and block (sector) size */
typedef struct {
    uint32_t blocks;
    uint32_t block_size;
} floppy_capacity_t;

/** State of the disk, from the current/maximum capacity descriptor of READ FORMAT CAPACITIES */
typedef enum {
    FLOPPY_MEDIA_UNFORMATTED = 1, /*!< Unformatted (or unreadable) disk: capacity is the maximum formattable one */
    FLOPPY_MEDIA_FORMATTED = 2,   /*!< Formatted disk: capacity is the current one */
    FLOPPY_MEDIA_NONE = 3,        /*!< No disk: capacity is the maximum the drive can format */
} floppy_media_state_t;

typedef struct {
    floppy_media_state_t state;
    floppy_capacity_t current;    /*!< Current capacity (formatted) or maximum capacity (unformatted, no disk) */
    uint8_t num_formats;          /*!< Entries in formats[] */
    floppy_capacity_t formats[FLOPPY_MAX_FORMATS]; /*!< Capacities the drive can format the inserted disk to.
                                                        Empty with no disk. */
} floppy_format_capacities_t;

/** Disk geometry */
typedef struct {
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors_per_track;
} floppy_geometry_t;

typedef struct {
    floppy_capacity_t capacity;  /*!< READ CAPACITY */
    bool write_protected;        /*!< Write protect tab open (MODE SENSE header) */
    uint8_t medium_type;         /*!< UFI medium type code: 0x1E 720 KB, 0x93 1.25 MB, 0x94 1.44 MB (0 if unknown) */
    floppy_geometry_t geometry;  /*!< From the flexible disk page (05h), or the standard format with this capacity */
    uint16_t transfer_rate_kbps; /*!< kbit/s (500 for HD, 250 for DD), 0 if the drive didn't report it */
} floppy_media_info_t;

/**
 * @brief Called after each formatted track
 * @return false to abort the format
 */
typedef bool (*floppy_format_progress_cb_t)(uint16_t cylinder, uint8_t head, const floppy_geometry_t *geometry,
                                            void *arg);

/* ------------------------------------------------------------------------ */
/* Driver                                                                   */
/* ------------------------------------------------------------------------ */

/**
 * @brief Install the floppy driver (registers a USB Host client).
 *
 * The USB Host Library must already be installed.
 */
esp_err_t floppy_host_install(const floppy_host_driver_config_t *config);

/**
 * @brief Uninstall the driver. All devices must be uninstalled first.
 */
esp_err_t floppy_host_uninstall(void);

/**
 * @brief Handle USB client events. Only needed if create_background_task was false.
 */
esp_err_t floppy_host_handle_events(TickType_t timeout);

/**
 * @brief Open and initialize a floppy drive found at @p usb_address.
 *
 * Claims the interface and issues INQUIRY. Does not require a disk.
 *
 * @return ESP_ERR_NOT_SUPPORTED if the device is not a floppy drive
 */
esp_err_t floppy_host_install_device(uint8_t usb_address, floppy_host_device_handle_t *handle);

esp_err_t floppy_host_uninstall_device(floppy_host_device_handle_t handle);

esp_err_t floppy_host_get_device_info(floppy_host_device_handle_t handle, floppy_host_device_info_t *info);

esp_err_t floppy_host_print_descriptors(floppy_host_device_handle_t handle);

/* ------------------------------------------------------------------------ */
/* Commands                                                                 */
/*                                                                          */
/* All command functions are thread safe (serialized per device). When a    */
/* command returns ESP_ERR_FLOPPY_CHECK_CONDITION (or one of the specific   */
/* errors derived from the sense data), the sense data has been fetched     */
/* automatically and is available via floppy_host_get_last_sense().         */
/* ------------------------------------------------------------------------ */

/**
 * @brief Execute an arbitrary UFI/SCSI command.
 *
 * @param cdb        Command descriptor block (1..12 bytes, padded to 12 with zeros)
 * @param data       Data buffer, or NULL for no data phase
 * @param data_len   Length of data buffer
 * @param data_in    true: device-to-host, false: host-to-device
 * @param timeout_ms Timeout for the data and status phases (0: default)
 * @param[out] actual_len  Bytes actually transferred (optional)
 */
esp_err_t floppy_host_execute_cdb(floppy_host_device_handle_t handle,
                                  const uint8_t *cdb, size_t cdb_len,
                                  void *data, size_t data_len, bool data_in,
                                  uint32_t timeout_ms, size_t *actual_len);

/** Sense data of the last command that failed with CHECK CONDITION */
esp_err_t floppy_host_get_last_sense(floppy_host_device_handle_t handle, floppy_sense_t *sense);

/** Human readable description of common sense codes */
const char *floppy_sense_to_str(const floppy_sense_t *sense);

/** Like esp_err_to_name(), knowing the ESP_ERR_FLOPPY_* codes too */
const char *floppy_err_to_name(esp_err_t err);

/** TEST UNIT READY. ESP_OK when a disk is in the drive. */
esp_err_t floppy_host_test_unit_ready(floppy_host_device_handle_t handle);

/**
 * @brief Poll TEST UNIT READY until the drive is ready.
 *
 * Swallows UNIT ATTENTION (disk changed, power on) and "becoming ready" conditions.
 *
 * @return ESP_OK, ESP_ERR_FLOPPY_NO_MEDIUM, ESP_ERR_FLOPPY_NOT_READY (timeout) or another error
 */
esp_err_t floppy_host_wait_ready(floppy_host_device_handle_t handle, uint32_t timeout_ms);

/**
 * @brief Number of disk changes (UNIT ATTENTION 28h) seen since the device was installed.
 *
 * Compare with an earlier value to know that the disk was swapped, even when nobody polled the drive while
 * there was no disk.
 */
uint32_t floppy_host_get_media_change_count(floppy_host_device_handle_t handle);

/** READ FORMAT CAPACITIES: disk state, current capacity and the capacities the disk can be formatted to */
esp_err_t floppy_host_read_format_capacities(floppy_host_device_handle_t handle, floppy_format_capacities_t *caps);

/** READ CAPACITY: number of blocks and block size of the formatted disk */
esp_err_t floppy_host_read_capacity(floppy_host_device_handle_t handle, floppy_capacity_t *capacity);

/**
 * @brief Capacity, write protection and geometry of the inserted disk.
 *
 * READ CAPACITY, then MODE SENSE (flexible disk page 05h). Drives that don't answer MODE SENSE are assumed
 * not write protected (a write then fails with ESP_ERR_FLOPPY_WRITE_PROTECTED), with the geometry of the
 * standard format of that capacity.
 */
esp_err_t floppy_host_get_media_info(floppy_host_device_handle_t handle, floppy_media_info_t *info);

/**
 * @brief Read sectors (READ(10)). Large requests are split internally.
 * @param block_size Sector size of the disk (512 for PC formats, 1024 for 1.2 MB PC-98 / 1.23 MB disks)
 */
esp_err_t floppy_host_read(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count, uint32_t block_size,
                           void *buf);

/** Write sectors (WRITE(10)). @return ESP_ERR_FLOPPY_WRITE_PROTECTED if the disk is write protected */
esp_err_t floppy_host_write(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count, uint32_t block_size,
                            const void *buf);

/** VERIFY: check that the sectors can be read, without transferring them */
esp_err_t floppy_host_verify(floppy_host_device_handle_t handle, uint32_t lba, uint32_t count);

/**
 * @brief Geometry of the standard format with this capacity (1.44 MB, 720 KB, 1.2 MB, 1.25 MB, 2.88 MB, ...).
 * @return ESP_ERR_NOT_FOUND for a capacity that isn't a known floppy format
 */
esp_err_t floppy_geometry_from_capacity(const floppy_capacity_t *capacity, floppy_geometry_t *geometry);

/**
 * @brief Low-level format a single track (UFI FORMAT UNIT, single track).
 * @param capacity The format, one of floppy_format_capacities_t.formats
 */
esp_err_t floppy_host_format_track(floppy_host_device_handle_t handle, const floppy_capacity_t *capacity,
                                   uint16_t cylinder, uint8_t head);

/**
 * @brief Low-level format the whole disk, track by track. Erases everything; follow with floppy_fs_mkfs().
 *
 * Takes about a minute for a 1.44 MB disk. With @p verify, each track is checked after formatting (VERIFY).
 *
 * @param capacity  The format, one of floppy_format_capacities_t.formats (NULL: the largest one)
 * @param progress  Optional, called after each track
 * @return ESP_OK, ESP_ERR_NOT_SUPPORTED (capacity is not a known floppy format), ESP_ERR_INVALID_STATE (aborted by
 *         the callback), ESP_ERR_FLOPPY_WRITE_PROTECTED, ESP_ERR_FLOPPY_MEDIUM_ERROR (bad track), or another error
 */
esp_err_t floppy_host_format(floppy_host_device_handle_t handle, const floppy_capacity_t *capacity, bool verify,
                             floppy_format_progress_cb_t progress, void *arg);

#ifdef __cplusplus
}
#endif
