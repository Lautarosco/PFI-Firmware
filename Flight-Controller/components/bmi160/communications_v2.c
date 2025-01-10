#include <communication.h>
#include "driver/i2c_master.h"
#include <esp_check.h>

const char * TAG = "I2C_MASTER";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;

esp_err_t i2c_init(int sda, int scl) {

    i2c_master_bus_config_t bus_config = {

        .i2c_port = I2C_NUM_0,          // Use port 0
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,         // Typical value for glitch filter
        .intr_priority = 0,             // Default priority
        .trans_queue_depth = 0,         // Synchronous operation (queue not used)
        .flags = { .enable_internal_pullup = true },
    };

    return i2c_new_master_bus(&bus_config, &i2c_bus_handle);
}


static i2c_master_dev_handle_t bmi160_handle = NULL;

esp_err_t bmi160_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data) {

    if (!bmi160_handle) {

        // Configure the BMI160 device handle on the I2C bus
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit address
            .device_address = sensor_addr,
            .scl_speed_hz = 100000,               // 100 kHz I2C speed
            .flags = { .disable_ack_check = false }
        };
        ESP_RETURN_ON_ERROR(
            i2c_master_bus_add_device(i2c_bus_handle, &device_config, &bmi160_handle),
            TAG, "Failed to add BMI160 device"
        );
    }

    uint8_t write_buffer[2] = { reg_addr, data };
    return i2c_master_transmit(bmi160_handle, write_buffer, sizeof(write_buffer), 1000); // 1 second timeout
}

esp_err_t bmi160_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {

    if (!bmi160_handle) {
        
        // Configure the BMI160 device handle on the I2C bus
        i2c_device_config_t device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit address
            .device_address = sensor_addr,
            .scl_speed_hz = 100000,               // 100 kHz I2C speed
            .flags = { .disable_ack_check = false }
        };
        ESP_RETURN_ON_ERROR(
            i2c_master_bus_add_device(i2c_bus_handle, &device_config, &bmi160_handle),
            TAG, "Failed to add BMI160 device"
        );
    }

    return i2c_master_transmit_receive(bmi160_handle, &reg_addr, 1, data, len, 1000); // 1 second timeout
}
