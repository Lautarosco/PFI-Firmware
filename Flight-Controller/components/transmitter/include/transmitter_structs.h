#ifndef STRUCT_TRANSMITTER_H
#define STRUCT_TRANSMITTER_H

#include <ps3.h>

#define MAC_ADDR_SIZE 6 /* Total bytes of MCU MAC Address */

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Transmitter buttons
 */
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

} tx_buttons_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/* Forward declaration of drone_globals_t struct to avoid including drone_globals.h header file */
typedef struct drone_globals drone_globals_t;

/* Forward declaration of transmitter_t struct */
typedef struct transmitter transmitter_t;

typedef struct transmitter {
   /* [ A ] Transmitter MAC Address */
   uint8_t mac_addr[ MAC_ADDR_SIZE ]; 

   /* [ A ] Drone's global variables */
   drone_globals_t * global_variables;

   /** @brief [ M ] Initialize Joystick  @param obj: Address of Transmitter object @param mac_p: Mac address @retval esp_err_t */
   esp_err_t ( *init )( transmitter_t * obj, const uint8_t mac_p[ MAC_ADDR_SIZE ] );

} transmitter_t;


#endif
