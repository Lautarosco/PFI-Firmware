file(GLOB SOURCES "source/*.c")
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES "driver" "drone")