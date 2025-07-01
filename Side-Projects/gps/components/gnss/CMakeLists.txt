idf_component_register(SRCS "src/hardware_layer/gnss_hwl_layer.c" "src/application_layer/gnss_app_layer.c"
                            "src/hardware_layer/protocols/ubx/classes/ids/ubx_nav_pvt.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver")
