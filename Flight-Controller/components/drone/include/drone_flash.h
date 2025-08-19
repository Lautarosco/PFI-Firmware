#include <esp_err.h>
#include <stdlib.h>

/**
 * @brief Write data to NVS of MCU
 * @param namespace: Label of the data
 * @param key_param: Name of the parameter
 * @param value: Data to be written
 * @param size: Size of data
 * @retval ESP_OK if success - ESP_ERROR
 */
esp_err_t __write_to_flash( const char* namespace, const char* param_key, const void* value, size_t size );

/**
 * @brief Read data from NVS of MCU
 * @param namespace: Label of the data
 * @param key_param: Name of the parameter
 * @param value: Variable to stored read data
 * @param size: Size of data
 * @retval ESP_OK if success - ESP_ERROR
 */
esp_err_t __read_from_flash( const char* namespace, const char* param_key, void* value, size_t size );
