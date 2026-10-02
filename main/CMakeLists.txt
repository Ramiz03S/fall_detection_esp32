idf_component_register(
    SRCS main.c i2c.c
    INCLUDE_DIRS .
    PRIV_INCLUDE_DIRS
    REQUIRES
    PRIV_REQUIRES esp_driver_i2c
)
