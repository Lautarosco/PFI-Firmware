#include <i2c_interface.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char * i2c_tag = "[I2C]";

/* ================= Master functions ================= */

esp_err_t i2c_init(i2c_master_bus_handle_t * master_handler, int sda, int scl) {
    ESP_LOGI(i2c_tag, "Initializing I2C bus...");

    i2c_master_bus_config_t i2c_master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = scl,
        .sda_io_num                   = sda,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    if(i2c_new_master_bus(&i2c_master_cfg, master_handler) == ESP_OK) {
        ESP_LOGI(i2c_tag, "I2C bus successfully initialized");
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t i2c_add_new_device(i2c_master_bus_handle_t master_handler, i2c_device_config_t *dev_config, void *dev_handler, serial_iface_type_t serial_iface_type) {
    if(serial_iface_type != IFACE_I2C) {
        ESP_LOGE(i2c_tag, "%s in line %d: Device serial interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else {
        esp_err_t ret = i2c_master_bus_add_device(master_handler, dev_config, (i2c_master_dev_handle_t *) dev_handler);
        if(ret != ESP_OK) {
            ESP_LOGE(i2c_tag, "%s in line %d: Add <0x%X> to I2C bus --> FAILED", __func__, __LINE__, dev_config->device_address);
            return ESP_FAIL;
        } else {
            ESP_LOGI(i2c_tag, "Add <0x%X> to I2C bus --> OK", dev_config->device_address);
            return ESP_OK;
        }
    }
}

/*
bool i2c_scan(i2c_master_bus_handle_t bus_handler, uint8_t slave_addr) {
    ESP_LOGI(i2c_tag, "Scanning I2C bus...");

    for(int addr = 1; addr < (2 << (7 - 1)); addr++) {
        if(i2c_master_probe(bus_handler, addr, 1000) == ESP_OK) {
            if(addr == slave_addr) {
                ESP_LOGI(i2c_tag, "Device found at <0x%X>", slave_addr);
                return true;
            }
        }
    }

    ESP_LOGI(i2c_tag, "Device <0x%X> was not found", slave_addr);

    return false;
}
*/

/* ================= Device functions ================= */

esp_err_t i2c_read_bytes(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len, serial_iface_type_t serial_iface_type) {
    if(serial_iface_type != IFACE_I2C) {
        ESP_LOGE(i2c_tag, "%s in line %d: Device serial interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else {
        return i2c_master_transmit_receive(*(i2c_master_dev_handle_t *) dev_handler, &reg_addr, 1, buff, len, 1000 / portTICK_PERIOD_MS);
    }
}

esp_err_t i2c_write_bytes(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len, serial_iface_type_t serial_iface_type) {
    if(serial_iface_type != IFACE_I2C) {
        ESP_LOGE(i2c_tag, "%s in line %d: Device serial interface is not I2C", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else {
        uint8_t buff[2] = {reg_addr, data};
        return i2c_master_transmit(*(i2c_master_dev_handle_t *) dev_handler, buff, sizeof(buff), 1000 / portTICK_PERIOD_MS);
    }
}
