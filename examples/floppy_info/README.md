# floppy_info

Waits for a USB floppy drive and, for each disk inserted:

- prints the disk state and the capacities the drive can format it to (READ FORMAT CAPACITIES),
- prints capacity, geometry and write protection,
- mounts the FAT file system at `/floppy`, prints label, serial number and free space, and lists the files,
- reads the largest file of the root directory and prints the speed.

Options (`idf.py menuconfig` → Floppy example):

| Option | Default | |
|---|---|---|
| `EXAMPLE_FORCE_FULL_SPEED` | y (high-speed targets) | Run the high-speed port at full speed, so a full-speed drive behind a high-speed hub works. Not needed when the drive is plugged in directly |
| `EXAMPLE_WRITE_TEST` | n | Write `ESPTEST.TXT`, read it back, move it into a new directory under a long name, delete both |
| `EXAMPLE_FORMAT` | n | **Erases the disk:** low-level format with verify, then an empty FAT12 file system labelled `ESP-FLOPPY` |

`sdkconfig.defaults` enables hub support and long file names.

```
idf.py set-target esp32p4
idf.py build flash monitor
```

ESP32-P4 boards with an early chip revision (v1.x) also need `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`.

With the USB-Serial-JTAG console, the last line printed before a quiet period can stay in its TX buffer until more
output comes; the example prints "Watching for disk changes..." after each disk for that reason.

Example output (TEAC FD-05PUB, 1.44 MB disk, ESP32-P4):

```
I (2335) floppy_info: Drive: TEAC FD-05PUB (rev 1026), VID 0x0644 PID 0x0000, CBI
W (4345) floppy_host: No command completion interrupt: using CB transport (status by REQUEST SENSE)
I (4355) floppy_info: Disk: formatted
I (4365) floppy_info:   current:   2880 x  512 =    1440 KB  (80 cyl, 2 heads, 18 sectors)
I (4365) floppy_info:   can format:   2880 x  512 =    1440 KB  (80 cyl, 2 heads, 18 sectors)
I (4365) floppy_info:   can format:   1232 x 1024 =    1232 KB  (77 cyl, 2 heads, 8 sectors)
I (4365) floppy_info:   can format:   2400 x  512 =    1200 KB  (80 cyl, 2 heads, 15 sectors)
I (4385) floppy_info: Media: 2880 blocks of 512 bytes, type 0x94, 500 kbit/s, 80/2/18, writable
I (4995) floppy_fs: Mounted FAT12 at /floppy
I (4995) floppy_info: FAT12 volume '', serial 004C-8ACF, 1423 KB, 614 KB free (mount + info 609 ms)
I (4995) floppy_info: Files:
I (4995) floppy_info:   MAVICA.HTM                                   8192  2002-01-01 00:08
I (4995) floppy_info:   MVC-104F.JPG                               330538  2002-01-01 00:07
...
I (18005) floppy_info: Read /floppy/MVC-105F.JPG: 348012 bytes in 13001 ms (26 KB/s), byte sum 0x02a6ae0e
I (21205) floppy_info: Write test: 20000 bytes written in 1996 ms, read back OK
I (23025) floppy_info: mkdir/rename/stat: OK, unlink/rmdir: OK
I (23025) floppy_info: Watching for disk changes...
```
