idf_component_register(SRCS "src/drone.c" "src/drone_flash.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "bmi160" "transmitter" "driver"
                             "pwm" "mma" "controllers" "spiffs" )
