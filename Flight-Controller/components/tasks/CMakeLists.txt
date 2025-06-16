idf_component_register(SRCS "src/tasks.c" "src/uart_init.c" "src/cmd_functions.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "drone" "state_machine")
