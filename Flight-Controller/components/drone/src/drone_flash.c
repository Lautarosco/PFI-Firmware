#include <drone_flash.h>


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
    if ( ret != ESP_OK ) {

        printf( "Error ( %s ) reading data from NVS!\n", esp_err_to_name( ret ) );
    }

    nvs_close( handle );
    return ret;
}

const char * GetKeyName( drone_flash_params_t key ) {

    switch ( key ) {

        case GYRO_OFFSET_X:
            return "gyro_offset_x";
            break;
        
        case GYRO_OFFSET_Y:
            return "gyro_offset_y";
            break;

        case GYRO_OFFSET_Z:
            return "gyro_offset_z";
            break;

        case PID_ROLL_KP:
            return "pid_roll_kp";
            break;

        case PID_ROLL_KI:
            return "pid_roll_ki";
            break;

        case PID_ROLL_KD:
            return "pid_roll_kd";
            break;

        case PID_ROLL_KB:
            return "pid_roll_kb";
            break;

        case PID_ROLL_D_KP:
            return "pid_roll_d_kp";
            break;

        case PID_ROLL_D_KI:
            return "pid_roll_d_ki";
            break;

        case PID_ROLL_D_KD:
            return "pid_roll_d_kd";
            break;

        case PID_ROLL_D_KB:
            return "pid_roll_d_kb";
            break;

        default:
            return "NOT FOUND";
            break;
    }
}
