# esp_usb_host_floppy

ESP-IDF USB Host class driver for **USB floppy disk drives**. It speaks the UFI command set (USB mass storage
subclass 0x04) over **Control/Bulk/Interrupt (CBI)**, **Control/Bulk (CB)** or **Bulk-Only** transport: the
transports of real USB floppy drives, which the stock `usb_host_msc` driver (Bulk-Only only) doesn't support.

On top of sector access it gives what's floppy-specific: the formattable capacities, geometry, write protection,
disk change detection, low-level formatting, and a FAT12 file system through ESP-IDF FatFs and the VFS, with a
cylinder cache that makes it reasonably fast.

It requires ESP-IDF >= 6.0 and the `espressif/usb` component. Tested on ESP32-P4 and ESP32-S3 with three
drives (see [Tested hardware](#tested-hardware)).

## Installation

```
idf.py add-dependency "kilrah/esp_usb_host_floppy^0.1.0"
```

The [floppy_info example](examples/floppy_info) prints the drive's identification, the disk's formats and
geometry, lists the files and measures the read speed; optionally it writes a test file and formats the disk.

**Power:** a floppy drive asks for 500 mA. A TEAC FD-05PUB ran fine from the VBUS of an ESP32-P4 board through a
bus-powered hub (motor, seeks, writes and formatting). With weaker supplies, a drive that enumerates and then drops
off the bus when the motor starts is a power problem.

**Hubs on high-speed ports:** USB floppy drives are full-speed devices. Behind a high-speed hub they need the
hub's Transaction Translator, which the ESP USB Host Library doesn't support: the drive isn't enumerated
(`Connected device is FS, transaction translator (TT) is not supported`). Plug the drive in directly, or run the
port at full speed so the hub does too. See `CONFIG_EXAMPLE_FORCE_FULL_SPEED` in the example: it sets
`usb_host_config_t.fsls_only` where the IDF HAL supports it, and otherwise sets HCFG.FSLSSupp itself before
powering the port.

## Files

| File | Content |
|---|---|
| [include/usb/floppy_host.h](include/usb/floppy_host.h) | Driver and command API, types, error codes |
| [include/usb/floppy_fs.h](include/usb/floppy_fs.h) | FAT file system API (FatFs + VFS, mkfs) |
| [private_include/floppy_private.h](private_include/floppy_private.h) | Device struct, timeouts, max transfer size |
| [src/floppy_host.c](src/floppy_host.c) | USB Host client, device install, descriptor parsing, CBI / CB / Bulk-Only transports, error recovery |
| [src/floppy_ufi.c](src/floppy_ufi.c) | UFI commands with auto REQUEST SENSE: INQUIRY, TUR, wait ready, READ FORMAT CAPACITIES, READ CAPACITY, MODE SENSE page 05h, READ/WRITE(10), VERIFY, FORMAT UNIT |
| [src/floppy_fs.c](src/floppy_fs.c) | FatFs disk I/O driver, cylinder cache (write-back), disk change handling, VFS mount, FAT12 mkfs |

## Usage

```c
// After usb_host_install():
const floppy_host_driver_config_t cfg = {
    .create_background_task = true, .task_priority = 5, .stack_size = 4096,
    .core_id = tskNO_AFFINITY, .callback = my_event_cb,
};
floppy_host_install(&cfg);

// In my_event_cb: FLOPPY_DEVICE_CONNECTED gives event->device.address -> forward to a task:
floppy_host_device_handle_t dev;
floppy_host_install_device(address, &dev);          // claims the interface, INQUIRY

if (floppy_host_wait_ready(dev, 3000) == ESP_OK) {
    floppy_media_info_t media;
    floppy_host_get_media_info(dev, &media);         // 2880 x 512, 80/2/18, write protected?

    floppy_fs_handle_t fs;
    floppy_fs_config_t fs_cfg = FLOPPY_FS_CONFIG_DEFAULT();   // "/floppy", 4 cylinders cached
    if (floppy_fs_mount(dev, &fs_cfg, &fs) == ESP_OK) {
        FILE *f = fopen("/floppy/README.TXT", "r");  // or opendir(), fwrite(), rename(), mkdir()...
        ...
        fclose(f);                                    // writes reach the disk at fclose() / fsync()
        floppy_fs_unmount(fs);
    }
}

// On FLOPPY_DEVICE_DISCONNECTED: floppy_host_uninstall_device(event->device.handle)
```

Sector access without a file system: `floppy_host_read()` / `floppy_host_write()` (with the disk's block size,
512, or 1024 for PC-98 disks). Formatting:

```c
floppy_host_format(dev, NULL, true, progress_cb, NULL); // low level, largest format, verify each track (~2 min)
floppy_fs_mkfs(dev, "MY DISK");                         // empty FAT12 file system (DOS quick format)
```

Every command function is thread-safe: calls are serialized per device with a mutex.

### Disk changes

A disk change is reported by the drive once, as UNIT ATTENTION 06/28 on the next command.
`floppy_host_wait_ready()` swallows it; `floppy_host_get_media_change_count()` counts every one seen by any
command, so an application polling the drive notices a disk swapped between two polls (see the example).

While mounted, the file system checks the drive at most once per second (TEST UNIT READY, on file system
access). When the disk is removed or changed, the cache is dropped, open files of the old disk become invalid,
and the new disk is mounted at the next access. Unwritten changes are lost if the disk is removed before
`fclose()` / `fsync()`: an error is logged.

### Errors

| Code | Meaning |
|---|---|
| `ESP_ERR_FLOPPY_CHECK_CONDITION` | The drive rejected the command. The sense data was fetched automatically: `floppy_host_get_last_sense()`, `floppy_sense_to_str()` |
| `ESP_ERR_FLOPPY_NO_MEDIUM` | No disk (sense 02/3A) |
| `ESP_ERR_FLOPPY_MEDIUM_CHANGED` | The disk was changed (06/28): the command was not executed |
| `ESP_ERR_FLOPPY_WRITE_PROTECTED` | Write protect tab open (07/27) |
| `ESP_ERR_FLOPPY_MEDIUM_ERROR` | Unreadable sector, CRC error... (sense key 03) |
| `ESP_ERR_FLOPPY_NOT_READY` | `floppy_host_wait_ready()` timed out |
| `ESP_ERR_FLOPPY_TRANSPORT` | USB transport failure. Recovery has already been attempted |
| `ESP_ERR_INVALID_STATE` | Device gone |

`floppy_err_to_name()` names them (`esp_err_to_name()` doesn't know component error codes).

## Design notes

- **Interface matching:** mass storage class, subclass 0x04 (UFI) with protocol 0x00 (CBI), 0x01 (CB) or 0x50
  (Bulk-Only); subclass 0x05 (SFF-8070i: LS-120, Zip) with CBI/CB. With
  `floppy_host_driver_config_t.accept_scsi_devices`, Bulk-Only devices with subclass 0x05/0x06 that report a
  removable direct access device are accepted too (off by default: it matches card readers and USB sticks).
  Command blocks are padded to 12 bytes.
- **The CBI interrupt transfer is exactly one packet** (2 bytes rounded up to the endpoint's MPS), not the size of
  the transfer buffer: the USB Host Library rounds buffers up to the cache line (64 bytes on the ESP32-P4), and the
  2-byte status is a full packet with MPS 2, so a 64-byte transfer never completes. Before this was fixed, all
  three drives looked like they sent no interrupt on the P4, and worked on the S3.
- **CBI without interrupts:** some drives declare CBI but don't send the command completion interrupt (Linux forces
  CB for some TEAC FD-05PUB revisions with an `unusual_devs` entry). UFI leaves the interrupt undefined for INQUIRY
  and REQUEST SENSE: if none comes for those, the driver stops waiting for it there only. If no interrupt comes for
  the first other command either, it falls back to CB (status by REQUEST SENSE). Each probe costs up to 1 s, once.
- **CB status is read too early for writes:** the REQUEST SENSE right after a WRITE's data stage arrives while the
  drive is still waiting for the sector to come around. In CB mode the TEAC answers 02/04/FF (02/04/04 for FORMAT
  UNIT); a next command then fails with 0B/4E (overlapped commands). The driver polls REQUEST SENSE every 10 ms
  while it says "in progress", up to the command timeout. With CBI the interrupt only comes at the end.
- **Drives stall the command block to reject a command** (e.g. TEST UNIT READY without a disk, or the first command
  after a disk change). The USB Host Library clears EP0 stalls itself, but logs each one from its ISR as
  `E USBH: Dev n EP 0 STALL`, which per-tag log levels don't filter. It's harmless: expect one per poll while the
  drive is empty.
- **Cylinder cache:** the drive caches nothing; every command waits for its first sector to come around (300 rpm:
  200 ms per revolution). Measured with the TEAC and a 1.44 MB disk:

  | Read | Time |
  |---|---|
  | 1 sector, re-read of the same | 199 ms |
  | 1 track (18 sectors) | 294 ms |
  | 1 cylinder (36 sectors) in one command | 599 ms |
  | 1 cylinder as 2 commands | 800 ms |
  | sequential sectors written one per command | 210 ms each |

  So the cache unit is a cylinder, read with one command, and FAT and directory accesses after the first are free.
  Reading a 348 KB file: 26 KB/s (the drive's own limit is ~30 KB/s); without the cache it doesn't finish in
  2 minutes, FatFs reading sector by sector. Writes are kept in the cache (write-back, dirty bitmap per cylinder)
  and written as one command per run of consecutive sectors when FatFs syncs, which it does at the end of every
  `fclose()`, `fsync()`, `mkdir()`, `rename()`, `unlink()`..., when a slot is reused and on unmount: a 20 KB file
  is written in 2 s instead of 9.6 s write-through. The cache (`cache_cylinders`, 18 KB per cylinder for 1.44 MB)
  goes to PSRAM when there is some.
- **Exact allocation lengths for variable-size replies** (READ FORMAT CAPACITIES, MODE SENSE): header first, then
  exactly the length it reports. A USB-IDE bridge used with optical drives didn't end the data phase with a short
  packet when the allocation length was larger than the reply.
- **Geometry:** from the flexible disk mode page (05h) if it adds up to the capacity, otherwise from a table of the
  standard formats (1.44 MB, 720 KB, 1.2 MB, 1.25 MB PC-98, 2.88 MB, DMF, 640 KB, 360 KB).
- **mkfs** writes a DOS 5-style boot sector (BPB of the standard format, media descriptor, serial number, label,
  "not bootable" boot code), two FATs and the root directory with a volume label entry. Only for the standard
  formats.
- **BOT error handling and data toggles:** same as
  [esp_usb_host_atapi](https://github.com/kilrah/esp_usb_host_atapi). After clearing a halt, the interface is
  released and re-claimed so that the host pipes restart at DATA0 (the ESP host doesn't reset its toggle on
  `usb_host_endpoint_clear()`). The interrupt endpoint is cleared on the device too, or its next interrupt would be
  dropped. CBI reset recovery is the Command Block Reset (SEND DIAGNOSTIC), then 2 s for the drive.
- **Timeouts:** command block 5 s, normal commands 10 s, reads, writes and format 30 s.

## Tested hardware

| Drive | Transport | Notes |
|---|---|---|
| TEAC FD-05PUB (0644:0000, rev 1026) | CBI | Formats 1.44 MB, 1.25 MB (1232 x 1024) and 1.2 MB. Also tested as CB (see above) |
| Y-E Data USB-FDU (057b:0000, bcdDevice 6.00, rev 6.01) | CBI | Same formats. Read 27 KB/s, write test OK. Reads a disk formatted by `floppy_fs_mkfs()` and then written on a PC |
| NEC USB UF000x (0409:0040, rev 1.50) | CBI | Bulk endpoints numbered differently (IN 0x82, OUT 0x01). Read 23 KB/s, write test OK |

The Y-E Data was also tested as CBI (ESP32-S3: read 26 KB/s, write test OK). The NEC was tested as CB only (before
the interrupt transfer fix); it declares CBI like the others.

Tested on:

- ESP32-P4 rev 1.0, through the board's WCH 1a86:8091 hub, high-speed port forced to full speed: reading, writing
  (files, directories, long names, rename, delete), write protection, disk removal and change, low-level format with
  verify (128 s for 1.44 MB) and mkfs, all as CB; then, after the interrupt fix, the TEAC as CBI: reading and
  writing, 10 boots and 4 power cycles.
- ESP32-P4 rev 3.2, TEAC plugged in directly, with and without the forcing: CBI with interrupts, reading and writing.
- ESP32-S3 (full-speed OTG), TEAC plugged in directly: CBI with interrupts, reading and writing.

## Limitations / TODO

- Single LUN (LUN 0) only.
- Bulk-Only UFI drives are implemented but untested (no such drive here).
- With CBI: formatting and disk changes were tested as CB only.
- A disk swapped while the file system stays mounted is handled (re-mount at the next access), but the example
  unmounts on removal, so that path was not exercised on hardware.
- mkfs only for the standard floppy formats; no FAT16/FAT32 creation.
- Formatting 720 KB disks: the TEAC listed no 720 KB format for an HD disk (as expected), so not tested.
- No support for suspend/resume events.

## License

Apache-2.0, see [LICENSE](LICENSE).

This driver was written with AI assistance (Claude by Anthropic), directed, reviewed and tested on hardware by the
author.
