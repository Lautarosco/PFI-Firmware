file(GLOB SOURCES "src/*.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES "bmi160" "bmp390" "gnss" "transmitter" "driver"
                             "pwm" "mma" "controllers" "spiffs" "lsm6dso"
                             "state_machine" "indicators" "print_manager")
