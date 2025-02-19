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

/**
 * @brief Write data to NVS of MCU
 * @param namespace: Label of the data
 * @param key_param: Name of the parameter
 * @param value: Data to be written
 * @param size: Size of data
 * @retval ESP_OK if success - ESP_ERROR
 */
esp_err_t __write_to_flash( const char* namespace, drone_flash_params_t key_param, const void* value, size_t size );

/**
 * @brief Read data from NVS of MCU
 * @param namespace: Label of the data
 * @param key_param: Name of the parameter
 * @param value: Variable to stored read data
 * @param size: Size of data
 * @retval ESP_OK if success - ESP_ERROR
 */
esp_err_t __read_from_flash( const char* namespace, drone_flash_params_t key_param, void* value, size_t size );

#endif
