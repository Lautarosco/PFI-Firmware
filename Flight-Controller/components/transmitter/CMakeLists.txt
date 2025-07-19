file(GLOB_RECURSE SRCS "src/*.c")
idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS "include"
                    REQUIRES "ps3" "esp_system" "esp_wifi" "nvs_flash"
                             "driver" "esp_http_server" "json" "spiffs" "bt")

spiffs_create_partition_image(spiffs ../../spiffs FLASH_IN_PROJECT)
