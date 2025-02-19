idf_component_register(SRCS "drone.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "bmi160" "transmitter" "driver"
                             "pwm" "mma" "controllers" "spiffs" )
