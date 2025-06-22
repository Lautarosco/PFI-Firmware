#ifndef INTERFACE_H
#define INTERFACE_H

#include <esp_err.h>
#include <stdint.h>
#include <interface_types.h>

typedef struct device_interface {
    /**
     * @brief Read n bytes of <reg_addr> register and store its content in <read_data> buffer
     * 
     * @param i2c_configs: I2C configs
     * @param reg_addr: Register address
     * @param read_data: Buffer to store read data
     * @param n_bytes: Total bytes to read
     * @param iface_type: Device selected interface
     * 
     * @retval
     *      - ESP_OK: Success
     *      - ESP_ERR_INVALID_ARG: Device selected interface is not I2C
     *      - ESP_FAIL: Failed to add new device to I2C bus
     */
    esp_err_t (*read_bytes)(void *i2c_configs, uint8_t reg_addr, uint8_t *read_data, size_t n_bytes, digital_interfaces_t iface_type);

    /**
     * @brief Write 1 byte (<data>) to <reg_addr> register
     * 
     * @param i2c_configs: I2C configs
     * @param reg_addr: Register address
     * @param data: Data to be written
     * @param iface_type: Device selected interface
     * 
     * @retval
     *      - ESP_OK: Success
     *      - ESP_ERR_INVALID_ARG: Device selected interface is not I2C
     *      - ESP_FAIL: Failed to add new device to I2C bus
     */
    esp_err_t (*write_bytes)(void *i2c_configs, uint8_t reg_addr, const uint8_t data, digital_interfaces_t iface_type);
    
    /* Settings of selected interface --> Here it should be added a struct with everything needed for the selected interface to work */
    void *iface_cfg;

    /* Type of interface */
    digital_interfaces_t iface_sel;
} device_interface_t;

#endif