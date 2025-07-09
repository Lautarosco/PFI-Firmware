#pragma once

#include <esp_err.h>
// #include <drone_structs.h>
#include <drone_configs.h>  /* This header file already includes drone_structs.h */


/**
 * @brief Make an instance of Drone Class
 * @param none
 * @retval Pointer to Drone object
 */
void Drone( drone_t * drone );

