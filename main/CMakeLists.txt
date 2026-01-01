idf_component_register(
    SRCS "tank.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp-zboss-lib
        esp-zigbee-lib
        nvs_flash
        driver
)

# make sure CONFIG_ZB_ENABLED is enabled in SDK configuration.
