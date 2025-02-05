idf_component_register(SRCS "src/tasks.c" "src/uart_init.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "drone" "state_machine")
