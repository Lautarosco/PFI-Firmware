#include <interface_i2c.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

static const char * i2c_tag = "[I2C]";

/* ================= Master functions ================= */

esp_err_t i2c_init(void *i2c_configs) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(i2c_new_master_bus(&(_i2c_configs->i2c_master_configs), &(_i2c_configs->i2c_master_handler)) != ESP_OK) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Initialize I2C bus --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(i2c_tag, "I2C bus successfully initialized");
    return ESP_OK;
}

esp_err_t i2c_add_new_device(void *i2c_configs, digital_interfaces_t iface_type) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Device selected interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    if(i2c_master_bus_add_device(_i2c_configs->i2c_master_handler, &(_i2c_configs->i2c_dev_configs), &(_i2c_configs->i2c_dev_handler)) != ESP_OK) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Add <0x%X> to I2C bus --> FAILED", __func__, __LINE__, _i2c_configs->i2c_dev_configs.device_address);
        return ESP_FAIL;
    }

    ESP_LOGI(i2c_tag, "Add <0x%X> to I2C bus --> OK", _i2c_configs->i2c_dev_configs.device_address);
    return ESP_OK;
}

/* ================= Device functions ================= */

esp_err_t i2c_read_bytes(void *i2c_configs, uint8_t reg_addr, uint8_t *read_data, size_t n_bytes, digital_interfaces_t iface_type) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "{%s in line %d}: Device selected interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    return i2c_master_transmit_receive(_i2c_configs->i2c_dev_handler, &reg_addr, 1, read_data, n_bytes, 1000 / portTICK_PERIOD_MS);
}

esp_err_t i2c_write_byte(void *i2c_configs, uint8_t reg_addr, const uint8_t data, digital_interfaces_t iface_type) {
    i2c_custom_t *_i2c_configs = (i2c_custom_t *) i2c_configs;

    if(iface_type != I2C) {
        ESP_LOGE(i2c_tag, "%s in line %d: Device serial interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t write_data[2] = {reg_addr, data};
    i2c_master_transmit(_i2c_configs->i2c_dev_handler, write_data, sizeof(write_data), 1000 / portTICK_PERIOD_MS);
}
