idf_component_register(SRCS "floppy_info_main.c"
                       PRIV_REQUIRES esp_usb_host_floppy usb esp_hal_usb vfs esp_timer)
