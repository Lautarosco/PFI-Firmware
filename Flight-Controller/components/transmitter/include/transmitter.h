#ifndef MANDO_H
#define MANDO_H

#include <transmitter_structs.h>

/**
* @brief Make an instance of Transmitter class
* @param global_variables: Address of drone's global variables struct
* @retval Pointer to Transmitter object
*/
transmitter_t * Transmitter( drone_globals_t * global_variables );

#endif
