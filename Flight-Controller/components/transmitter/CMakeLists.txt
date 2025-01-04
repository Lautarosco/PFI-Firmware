idf_component_register(SRCS "transmitter.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "ps3" "esp_system" "esp_wifi" "nvs_flash" "driver")