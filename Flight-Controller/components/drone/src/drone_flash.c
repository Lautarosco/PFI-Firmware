#include <drone_flash.h>
#include <string.h>
/* Function implementations */

esp_err_t __write_to_flash( const char* namespace, drone_flash_params_t key_param, const void* value, size_t size ) {

    const char * key = GetKeyName( key_param );
    nvs_handle_t handle;
    esp_err_t ret = nvs_open( namespace, NVS_READWRITE, &handle );
    if ( ret != ESP_OK ) {

        printf( "Error ( %s ) opening NVS handle!\n", esp_err_to_name( ret ) );
        return ret;
    }

    ret = nvs_set_blob( handle, key, value, size );
    if ( ret != ESP_OK ) {

        printf( "Error ( %s ) writing data to NVS!\n", esp_err_to_name( ret ) );
        nvs_close( handle );
        return ret;
    }

    ret = nvs_commit( handle );
    if ( ret != ESP_OK ) {
        
        printf( "Error ( %s ) committing data to NVS!\n", esp_err_to_name( ret ) );
    }

    nvs_close( handle );
    return ret;
}

esp_err_t __read_from_flash( const char* namespace, drone_flash_params_t key_param, void* value, size_t size ) {

    const char * key = GetKeyName( key_param );
    nvs_handle_t handle;
    esp_err_t ret = nvs_open( namespace, NVS_READWRITE, &handle );
    if ( ret != ESP_OK ) {

        printf( "Error ( %s ) opening NVS handle!\n", esp_err_to_name( ret ) );
        return ret;
    }

    size_t required_size = size;
    ret = nvs_get_blob( handle, key, value, &required_size );
    if ( ret == ESP_ERR_NVS_NOT_FOUND ) {
        printf( "Key '%s' not found in NVS. Initializing with default value.\n", key );
        memset(value, 0, size); // Set value to zero or provide your own default
    } else if ( ret != ESP_OK ) {
        printf( "Error ( %s ) reading data from NVS!\n", esp_err_to_name( ret ) );
    }

    nvs_close( handle );
    return ret;
}
/*
    * @brief Get the key name for a given drone_flash_params_t enum value
    * @param key: The drone_flash_params_t enum value
    * @retval The corresponding key name as a string, or "NOT FOUND" if the key is not recognized
*/

const char * GetKeyName( drone_flash_params_t key ) {

    int num_keys = sizeof(key_names) / sizeof(key_names[0]);
    if (key >= 0 && key < num_keys) {
        return key_names[key];
    }
    return "NOT FOUND";
}
