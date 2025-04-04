#include <com.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>


static const char * i2c_tag = "I2C";

esp_err_t i2c_init(i2c_master_bus_handle_t * bus_handler, int sda, int scl) {
    ESP_LOGI(i2c_tag, "Initializing I2C interface...");

    i2c_master_bus_config_t master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = scl,
        .sda_io_num                   = sda,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    if(i2c_new_master_bus(&master_cfg, bus_handler) == ESP_OK) {
        ESP_LOGI(i2c_tag, "I2C interface successfully initialized");
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}


esp_err_t i2c_add_device(i2c_master_bus_handle_t bus_handler, i2c_master_dev_handle_t * dev_handler, uint8_t addr, i2c_addr_bit_len_t addr_len, int scl_freq) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = addr_len,
        .device_address = addr,
        .scl_speed_hz = scl_freq
    };

    if(i2c_master_bus_add_device(bus_handler, &dev_cfg, dev_handler) != ESP_OK) {
        ESP_LOGE("BMP280", "Failed adding <0x%X> to I2C bus", addr);
        return ESP_FAIL;
    } else {
        ESP_LOGI("BMP280", "<0x%X> successfully added to I2C bus", addr);
        return ESP_OK;
    }
}


esp_err_t i2c_read_bytes(i2c_master_dev_handle_t dev_handler, uint8_t reg_addr, uint8_t * data, size_t len) {
    return i2c_master_transmit_receive(dev_handler, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}


esp_err_t i2c_write_bytes(i2c_master_dev_handle_t dev_handler, uint8_t reg_addr, uint8_t data) {
    uint8_t buff[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handler, buff, sizeof(buff), 1000 / portTICK_PERIOD_MS);
}


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
