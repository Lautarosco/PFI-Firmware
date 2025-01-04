#ifndef DRONE_GLOBAL_VARIABLES_H
#define DRONE_GLOBAL_VARIABLES_H

/* Forward declaration to avoid including transmitter.h header file ( avoid circular dependency ) */
typedef struct tx_buttons tx_buttons_t;

/* Forward declaration to use BluetoothData_t type */
typedef struct BluetoothData BluetoothData_t;

/**
 * @brief Drone's global variables
 */
typedef struct drone_globals {
    /* Transmitter buttons */
    tx_buttons_t * tx_buttons;
    
    /* Bluetooth data */
    BluetoothData_t * bt_data;

} drone_globals_t;

#endif
