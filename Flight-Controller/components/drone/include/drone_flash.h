#ifndef DRONE_FLASH_PARAMS_H
#define DRONE_FLASH_PARAMS_H

#include <drone_structs.h>
#include <nvs.h>

/* Function prototypes */

/**
 * @brief Get name of Drone parameter stored in flash memory
 * @param key: Drone param enum
 * @retval param name
 */
const char * GetKeyName( drone_flash_params_t key );

#endif
