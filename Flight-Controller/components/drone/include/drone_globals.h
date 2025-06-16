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

    /* IIR filter coefficient */
    float ema_filter_roll;
    float ema_filter_pitch;
    float ema_filter_yaw;

} drone_globals_t;

#endif
