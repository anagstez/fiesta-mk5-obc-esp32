idf_component_register(SRCS "main.c" "mcp2515.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd driver esp_driver_ledc esp_driver_spi esp_driver_gpio esp_driver_usb_serial_jtag esp_driver_twai nvs_flash)
