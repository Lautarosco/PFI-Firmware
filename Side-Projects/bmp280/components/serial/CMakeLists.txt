idf_component_register(SRCS "src/i2c_interface.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver"
                    PRIV_REQUIRES "esp_driver_i2c")
