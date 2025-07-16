# file(GLOB_RECURSE SOURCES "src/*.c")

idf_component_register(
    SRCS
        "src/application_layer/lsm6dso_app_layer.c"
        "src/hardware_layer/lsm6dso_hwl_layer.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver")
