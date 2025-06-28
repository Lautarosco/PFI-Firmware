idf_component_register(SRCS "src/application_layer/bmp390_app_layer.c" "src/application_layer/compensation/bmp390_compensation.c"
                            "src/hardware_layer/bmp390_hw_layer.c" "src/hardware_layer/bmp390_modes_names.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "interface")
