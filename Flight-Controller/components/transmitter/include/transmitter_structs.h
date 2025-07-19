#ifndef STRUCT_TRANSMITTER_H
#define STRUCT_TRANSMITTER_H

#include <ps3.h>
#include "esp_spp_api.h"

#define PLAYSTATION_TX     0  /* Use playstation transmitter */
#define WEBSV_TX           1  /* Use web server transmitter */

#define MAC_ADDR_SIZE      6  /* Total bytes of MCU MAC Address */
#define MAX_BT_DATA_SIZE   1024 /*Max size of data sent via BT*/  

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Transmitter buttons
 */

typedef struct tx_analog_stick {
   
   int8_t x;  /* Analog stick X axis */
   int8_t y;  /* Analog stick Y axis */

} tx_analog_stick_t;

typedef struct tx_buttons {
   
   bool cross;
   bool square;
   bool triangle;
   bool circle;
   bool up;
   bool down;
   bool left;
   bool right;
   bool r1;
   bool r2;
   bool l1;
   bool l2;
   bool start;
   bool select;
   bool ps;

   tx_analog_stick_t left_stick;  /* Left analog stick */
   tx_analog_stick_t right_stick; /* Right analog stick */

} tx_buttons_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/* Forward declaration of drone_globals_t struct to avoid including drone_globals.h header file */
typedef struct drone_globals drone_globals_t;

/* Forward declaration of transmitter_t struct */
typedef struct transmitter transmitter_t;

typedef struct transmitter_methods {

   /** @brief [ M ] Initialize Joystick  @param tx: Address of Transmitter object @retval esp_err_t */
   esp_err_t ( * init )( transmitter_t * tx, const uint8_t mac_p[ MAC_ADDR_SIZE] );

   /** */ 
   esp_err_t ( * send_bt_data)(transmitter_t * tx, const uint8_t* data[MAX_BT_DATA_SIZE], uint16_t len);

} transmitter_methods_t;

typedef struct bluetooth_connection {

   /* SPP Handle*/
   uint32_t spp_handle;

   /* Connected flag */
   bool is_connected;

   /* Transmitter MAC Address */
   uint8_t mac_addr[ MAC_ADDR_SIZE ];

   /* Remote Bluetooth Device Address */
   uint8_t bda_mac_addr[ MAC_ADDR_SIZE ];


} bluetooth_connection_t;

typedef struct transmitter {

   /* Drone's global variables */
   drone_globals_t * global_variables;
   transmitter_methods_t methods;
   bluetooth_connection_t bluetooth_connection;

} transmitter_t;


#endif
