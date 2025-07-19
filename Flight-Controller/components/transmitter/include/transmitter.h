#ifndef MANDO_H
#define MANDO_H

#include <transmitter_structs.h>

// #define PLAYSTATION_TX
#define WEBSV_TX 1
#define PLAYSTATION_TX 0
#define ESP_SPP_INVALID_HANDLE 0

extern transmitter_t* active_transmitter;  // Declaration (definition in transmitter.c)
extern const char * TRANSMITTER_TAG;

esp_err_t ps3_transmitter_init(transmitter_t* Tx, const uint8_t mac_p[ MAC_ADDR_SIZE]);
esp_err_t web_transmitter_init(transmitter_t* Tx, const uint8_t mac_p[ MAC_ADDR_SIZE]);

/**
* @brief Make an instance of Transmitter class
* @param global_variables: Address of drone's global variables struct
* @retval Pointer to Transmitter object
*/
void Transmitter( transmitter_t * Tx, drone_globals_t * global_variables );

#endif
