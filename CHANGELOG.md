# Changelog

## 0.1.1 - 2026-11-26

- `floppy_host_read_format_capacities()` no longer logs an error when the drive reports a disk change or no disk:
  the first command after a disk goes in always gets the disk change, so polling an empty drive with it logged an
  error every time a disk was inserted.
- `floppy_fs_mount()` documentation: the cache is write-back (writes reach the disk when FatFs syncs, e.g. at
  `fclose()`), not write-through.

## 0.1.0 - 2026-11-26

First version.

- USB Host client for UFI floppy drives: CBI, CB and Bulk-Only transports, with automatic fallback from CBI to
  CB for drives that send no command completion interrupt.
- UFI commands with auto REQUEST SENSE: INQUIRY, TEST UNIT READY, READ FORMAT CAPACITIES, READ CAPACITY,
  MODE SENSE (flexible disk page), READ(10), WRITE(10), VERIFY, FORMAT UNIT; disk change counter.
- Low-level format of the whole disk with optional verify and progress callback.
- FAT file system through ESP-IDF FatFs and the VFS, with a write-back cylinder cache and disk change handling.
- FAT12 mkfs for the standard floppy formats.
- Example `floppy_info`.
