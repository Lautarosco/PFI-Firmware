#include <i2c/interface_i2c.h>      /* I2C custom driver */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

static const char * i2c_tag = "[I2C]";

/* ================= Master functions ================= */

esp_err_t i2c_init_master_bus(i2c_master_bus_config_t *i2c_master_bus_configs, i2c_master_bus_handle_t *i2c_master_handler) {
    if(i2c_new_master_bus(i2c_master_bus_configs, i2c_master_handler) != ESP_OK) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Initialize I2C bus --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(i2c_tag, "I2C bus successfully initialized");
    return ESP_OK;
}

esp_err_t i2c_add_new_device(i2c_master_bus_handle_t i2c_master_handler, i2c_device_config_t *dev_configs, i2c_master_dev_handle_t *dev_handler, digital_interfaces_t iface_type) {
    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Device selected interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    if(i2c_master_bus_add_device(i2c_master_handler, dev_configs, dev_handler) != ESP_OK) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Add <0x%X> to I2C bus --> FAILED", __func__, __LINE__, dev_configs->device_address);
        return ESP_FAIL;
    }

    ESP_LOGI(i2c_tag, "Add <0x%X> to I2C bus --> OK", dev_configs->device_address);
    return ESP_OK;
}

/* ================= Device functions ================= */

esp_err_t i2c_read_bytes(void *i2c_configs, uint8_t reg_addr, uint8_t *read_data, size_t n_bytes, digital_interfaces_t iface_type) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Device selected interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = i2c_master_transmit_receive(*(_i2c_configs->i2c_dev_handler), &reg_addr, 1, read_data, n_bytes, 1000 / portTICK_PERIOD_MS);

    switch (ret) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(i2c_tag, "{%s in line %d}: <i2c_master_transmit_receive> invalid parameter", __func__, __LINE__);
            break;

        case ESP_ERR_TIMEOUT:
            ESP_LOGE(i2c_tag, "{%s in line %d}: Operation timeout - Bus is busy or hardware crashed", __func__, __LINE__);
            break;

        default:
            break;
    }

    return ret;
}

esp_err_t i2c_write_byte(void *i2c_configs, uint8_t reg_addr, const uint8_t data, digital_interfaces_t iface_type) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "%s in line %d: Device serial interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t write_data[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_transmit(*(_i2c_configs->i2c_dev_handler), write_data, sizeof(write_data), 1000 / portTICK_PERIOD_MS);

    switch (ret) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(i2c_tag, "{%s in line %d}: <i2c_master_transmit_receive> invalid parameter", __func__, __LINE__);
            break;

        case ESP_ERR_TIMEOUT:
            ESP_LOGE(i2c_tag, "{%s in line %d}: Operation timeout - Bus is busy or hardware crashed", __func__, __LINE__);
            break;

        default:
            break;
    }

    return ret;
}
