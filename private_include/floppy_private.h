/*
 * SPDX-FileCopyrightText: 2026 Andre Bernet
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sys/queue.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "usb/floppy_host.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mass storage interface subclasses and protocols */
#define MSC_SUBCLASS_UFI        0x04 /* USB Floppy Interface */
#define MSC_SUBCLASS_SFF8070I   0x05
#define MSC_SUBCLASS_SCSI       0x06
#define MSC_PROTOCOL_CBI        0x00 /* Control/Bulk/Interrupt, with command completion interrupt */
#define MSC_PROTOCOL_CB         0x01 /* Control/Bulk, no command completion interrupt */
#define MSC_PROTOCOL_BOT        0x50 /* Bulk-Only */

#define FLOPPY_CDB_LEN          12   /* UFI command blocks are always 12 bytes */

/* Default timeouts. Floppies are slow: spin-up ~0.5 s, a full seek ~0.3 s, and the drive retries bad sectors. */
#define FLOPPY_TIMEOUT_DEFAULT_MS  10000
#define FLOPPY_TIMEOUT_CBW_MS       5000
#define FLOPPY_TIMEOUT_LONG_MS     30000 /* reads, writes, format */

/* Maximum payload of a single data phase (larger reads are split): a 2.88 MB track (36 sectors of 512 bytes) */
#define FLOPPY_MAX_XFER_SIZE       (36 * 512)

typedef struct floppy_host_device {
    STAILQ_ENTRY(floppy_host_device) tailq_entry;
    SemaphoreHandle_t lock;          /* Serializes commands */
    SemaphoreHandle_t transfer_done; /* Signalled by transfer callbacks */
    usb_device_handle_t dev_hdl;
    usb_transfer_t *bulk_xfer;       /* Bulk transfers (CBW, data, CSW) */
    usb_transfer_t *ctrl_xfer;       /* Control transfers (CBI command block, reset, clear halt) */
    usb_transfer_t *intr_xfer;       /* CBI command completion interrupt */
    bool xfer_lost;                  /* A timed-out transfer could not be cancelled: the device is unusable */
    bool intr_seen;                  /* CBI: the drive has sent a command completion interrupt */
    bool info_intr_seen;             /* UFI CBI: ... for INQUIRY or REQUEST SENSE */
    bool no_info_intr;               /* UFI CBI: the drive sends none for INQUIRY and REQUEST SENSE */
    uint8_t usb_address;
    uint8_t iface_num;
    uint8_t subclass;
    uint8_t protocol;
    uint8_t alt_setting;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t ep_intr;
    uint16_t mps_in;
    uint16_t mps_intr;
    uint32_t tag;
    volatile uint32_t media_changes; /* UNIT ATTENTION 28h seen */
    floppy_sense_t last_sense;
    floppy_host_device_info_t info;
} floppy_device_t;

/* Result of a transport that doesn't report the command status (CB): the caller must ask with REQUEST SENSE */
#define FLOPPY_STATUS_UNKNOWN  (ESP_ERR_FLOPPY_BASE + 0x100)

/**
 * @brief Execute a command without taking the device lock and without auto sense.
 *
 * @return ESP_OK, ESP_ERR_FLOPPY_CHECK_CONDITION (command failed), FLOPPY_STATUS_UNKNOWN (CB transport) or a
 *         transport error
 */
esp_err_t floppy_transport_command(floppy_device_t *dev, const uint8_t *cdb, size_t cdb_len,
                                   void *data, size_t data_len, bool data_in,
                                   uint32_t timeout_ms, size_t *actual_len);

#ifdef __cplusplus
}
#endif
