#include "esp_err.h"

#include "print_manager.h"
#include "transmitter.h"


esp_err_t bluetooth_write(const char* data, size_t len, void* context) {
    transmitter_t* active_transmitter = (transmitter_t*) context;
    if (active_transmitter == NULL) {
        return ESP_FAIL;
    }
    if (active_transmitter->bluetooth_connection.is_connected) {
        return active_transmitter->methods.send_bt_data(active_transmitter, data, len);
    }
    return ESP_FAIL;
}

print_output_t bluetooth_output = {
    .name = "Bluetooth",
    .write = bluetooth_write,
    .context = NULL
};
