#include <transmitter.h>
#include <stdio.h>
#include <esp_log.h>
#include <string.h>

const char *TRANSMITTER_TAG = "TRANSMITTER";

transmitter_t* active_transmitter = NULL;

esp_err_t transmitter_send_bt_data(transmitter_t *tx, const uint8_t *data, uint16_t len) {
    if (tx == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (tx->bluetooth_connection.spp_handle != ESP_SPP_INVALID_HANDLE) {
        esp_err_t ret = esp_spp_write(tx->bluetooth_connection.spp_handle, len, data);
        if (ret != ESP_OK) {
            ESP_LOGE(TRANSMITTER_TAG, "Failed to send BT data: %s", esp_err_to_name(ret));
            return ret;
        }
        return ESP_OK;
    }
    
    ESP_LOGW(TRANSMITTER_TAG, "No active BT connection to send data");
    return ESP_ERR_INVALID_STATE;
}

void Transmitter( transmitter_t* Tx, drone_globals_t * global_variables ) {
    memset(Tx, 0, sizeof(transmitter_t));

    Tx->global_variables = global_variables;
    Tx->methods.send_bt_data = transmitter_send_bt_data;
    Tx->bluetooth_connection.spp_handle = ESP_SPP_INVALID_HANDLE;
    active_transmitter = Tx;

    // mac is set in init to allow external logic to determine it.

    #ifdef WEBSV_TX
        ESP_LOGI( "TRANSMITTER", "Making an instance of WEB Transmitter Class..." );
        Tx->methods.init = web_transmitter_init;
    #elif PLAYSTATION_TX
        ESP_LOGI( "TRANSMITTER", "Making an instance of PS3 Transmitter Class..." );
        Tx->methods.init = ps3_transmitter_init;
    #endif

    ESP_LOGI( "TRANSMITTER", "Instance succesfully made" );
}
