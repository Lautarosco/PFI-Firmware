file(GLOB SOURCES "src/*.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES "bmi160" "transmitter" "driver"
                             "pwm" "mma" "controllers" "spiffs"
                             "state_machine")
