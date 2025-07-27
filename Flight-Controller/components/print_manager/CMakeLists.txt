file(GLOB SRCS "source/*.c")

idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS "include"
                    REQUIRES "driver" "esp_system" "transmitter")