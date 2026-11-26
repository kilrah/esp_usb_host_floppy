idf_component_register(SRCS "src/floppy_host.c"
                            "src/floppy_ufi.c"
                            "src/floppy_fs.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES usb
                       PRIV_REQUIRES fatfs vfs esp_hw_support
                       )
