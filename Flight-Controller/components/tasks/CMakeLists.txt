file(GLOB SRC_FILES "src/*.c")
message(STATUS "Tasks component source files: ${SRC_FILES}")


idf_component_register(SRCS ${SRC_FILES}
                    INCLUDE_DIRS "include"
                    REQUIRES "drone" "state_machine" "esp_adc"
                             "indicators" "bmi160" "transmitter"
                             "driver" "pwm" "mma" "controllers"
                             "spiffs")
