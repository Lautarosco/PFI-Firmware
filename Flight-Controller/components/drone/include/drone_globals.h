#ifndef DRONE_GLOBAL_VARIABLES_H
#define DRONE_GLOBAL_VARIABLES_H

/* Forward declaration to avoid including transmitter.h header file ( avoid circular dependency ) */
typedef struct tx_buttons tx_buttons_t;

/* Forward declaration to use SerialData_t type */
typedef struct SerialData SerialData_t;

/**
 * @brief Drone's global variables
 */
typedef struct drone_globals {
    /* Transmitter buttons */
    tx_buttons_t * tx_buttons;
    
    /* Serial data */
    SerialData_t * serial_data;

} drone_globals_t;

#endif
