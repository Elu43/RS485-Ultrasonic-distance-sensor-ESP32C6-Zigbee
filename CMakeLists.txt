idf_component_register(
    SRCS "tank.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp-zboss-lib
        esp-zigbee-lib
        nvs_flash
        esp_driver_gpio
        esp_driver_uart
)

# make sure CONFIG_ZB_ENABLED is enabled in SDK configuration.
