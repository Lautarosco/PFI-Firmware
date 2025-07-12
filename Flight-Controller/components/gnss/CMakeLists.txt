# Search all .c files inside src/ and its subdirectories
idf_component_register(
        SRCS "src/application_layer/gnss_app_layer.c"
             "src/application_layer/neoxm/neo7m_app_layer.c"
             "src/hardware_layer/gnss_hwl_layer.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_cfg/ubx_cfg_msg.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_cfg/ubx_cfg_nav5.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_cfg/ubx_cfg_rate.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_nav/ubx_nav_dop.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_nav/ubx_nav_pvt.c"
             "src/hardware_layer/protocols/ubx/classes/ubx_nav/ubx_nav_svinfo.c"
        INCLUDE_DIRS "include"
        REQUIRES "driver" "gnss"
)
