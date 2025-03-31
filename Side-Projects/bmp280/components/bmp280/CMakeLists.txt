idf_component_register(
        SRCS          "src/bmp280.c"
                      "src/bmp280_hal_drivers/bmp280_hal_api.c"
                      "src/bmp280_ll_drivers/communications/com.c"
                      "src/bmp280_ll_drivers/measurements/compensation_words.c"
                      
        INCLUDE_DIRS  "include"
        PRIV_REQUIRES "driver"
)
