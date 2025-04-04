idf_component_register(
        SRCS          "src/bmp280.c"
                      "src/bmp280_hal_drivers/bmp280_hal_api.c"
                      "src/com.c"
                      "src/get_modes_names.c"
                      
        INCLUDE_DIRS  "include"
        REQUIRES esp_driver_i2c
        PRIV_REQUIRES "driver"
)
