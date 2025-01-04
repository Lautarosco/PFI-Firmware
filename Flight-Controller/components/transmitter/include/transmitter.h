#ifndef MANDO
#define MANDO

#include <transmitter_structs.h>

/**
* @brief Make an instance of Joystick class
* @param global_variables: Address of drone's global variables struct
* @retval Pointer to Transmitter object
*/
transmitter_t * Transmitter( drone_globals_t * global_variables );

#endif
