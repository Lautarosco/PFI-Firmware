#include <i2c_interface.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char * i2c_tag = "[I2C]";

/* ================= Master functions ================= */

esp_err_t i2c_init(i2c_master_bus_handle_t * master_handler, int sda, int scl) {
    ESP_LOGI(i2c_tag, "Initializing I2C interface...");

    i2c_master_bus_config_t i2c_master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = scl,
        .sda_io_num                   = sda,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    if(i2c_new_master_bus(&i2c_master_cfg, master_handler) == ESP_OK) {
        ESP_LOGI(i2c_tag, "I2C interface successfully initialized");
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t i2c_add_device(i2c_master_bus_handle_t master_handler, i2c_master_dev_handle_t * dev_handler, uint8_t addr, i2c_addr_bit_len_t addr_len, int scl_freq) {
    i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = addr_len,
        .device_address = addr,
        .scl_speed_hz = scl_freq
    };

    if(i2c_master_bus_add_device(master_handler, &i2c_dev_cfg, dev_handler) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d: Failed adding <0x%X> to I2C bus", __func__, __LINE__, addr);
        return ESP_FAIL;
    } else {
        ESP_LOGI(i2c_tag, "<0x%X> device was successfully added to I2C bus", addr);
        return ESP_OK;
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

esp_err_t i2c_read_bytes(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len) {
    return i2c_master_transmit_receive(dev_handler, &reg_addr, 1, buff, len, 1000 / portTICK_PERIOD_MS);
}

esp_err_t i2c_write_bytes(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len) {
    uint8_t buff[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handler, buff, sizeof(buff), 1000 / portTICK_PERIOD_MS);
}
