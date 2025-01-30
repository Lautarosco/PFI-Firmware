#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "inttypes.h"

// Initialize NVS
void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// Save data to NVS
esp_err_t save_data_to_nvs(const char* namespace, const char* key, int32_t value) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(namespace, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_i32(handle, key, value);
    if (ret != ESP_OK) {
        printf("Error (%s) writing data to NVS!\n", esp_err_to_name(ret));
        nvs_close(handle);
        return ret;
    }

    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        printf("Error (%s) committing data to NVS!\n", esp_err_to_name(ret));
    }

    nvs_close(handle);
    return ret;
}

// Retrieve data from NVS
esp_err_t retrieve_data_from_nvs(const char* namespace, const char* key, int32_t* value) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(namespace, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_get_i32(handle, key, value);
    if (ret != ESP_OK) {
        printf("Error (%s) reading data from NVS!\n", esp_err_to_name(ret));
    }

    nvs_close(handle);
    return ret;
}

// Main application
void app_main(void) {
    // Initialize NVS
    initialize_nvs();

    // Save data to NVS
    const char* namespace = "storage";
    const char* key = "data_key";
    esp_err_t ret;
    
    int32_t data_to_save = 1234;
    ret = save_data_to_nvs(namespace, key, data_to_save);
    if (ret == ESP_OK) {
        printf("Data saved to NVS:  %"PRId32"\n", data_to_save);
    }
    

    // Retrieve data from NVS
    int32_t data_retrieved = 0;
    ret = retrieve_data_from_nvs(namespace, key, &data_retrieved);
    if (ret == ESP_OK) {
        printf("Data retrieved from NVS:  %"PRId32"\n", data_retrieved);
    }
}