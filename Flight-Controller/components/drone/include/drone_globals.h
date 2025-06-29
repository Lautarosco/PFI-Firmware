#ifndef DRONE_GLOBAL_VARIABLES_H
#define DRONE_GLOBAL_VARIABLES_H

#include "transmitter_structs.h"

/* Forward declaration to use pid_gain_t type */
typedef struct pid_gain pid_gain_t;

/**
 * @brief Drone's global variables
 */
typedef struct drone_globals {
    /* Transmitter buttons */
    tx_buttons_t tx_buttons;
    
    /* Transmitter buttons previous state*/
    tx_buttons_t tx_buttons_prev;
    
    /* Serial data */
    SerialData_t serial_data;

    /* Misc. Floats*/
    float misc_floats[10];

} drone_globals_t;

#endif
