# Changelog

## 0.1.0 - 2026-11-26

First version.

- USB Host client for UFI floppy drives: CBI, CB and Bulk-Only transports, with automatic fallback from CBI to
  CB for drives that send no command completion interrupt (TEAC FD-05PUB).
- UFI commands with auto REQUEST SENSE: INQUIRY, TEST UNIT READY, READ FORMAT CAPACITIES, READ CAPACITY,
  MODE SENSE (flexible disk page), READ(10), WRITE(10), VERIFY, FORMAT UNIT; disk change counter.
- Low-level format of the whole disk with optional verify and progress callback.
- FAT file system through ESP-IDF FatFs and the VFS, with a write-back cylinder cache and disk change handling.
- FAT12 mkfs for the standard floppy formats.
- Example `floppy_info`.
