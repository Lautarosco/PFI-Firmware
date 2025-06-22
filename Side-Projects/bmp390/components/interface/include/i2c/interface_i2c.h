#ifndef INTERFACE_I2C_H
#define INTERFACE_I2C_H

#include <interface_types.h>    /* Available types of interface */
#include <i2c/i2c_custom.h>     /* I2C custom struct */

/**
 * @brief Initialize I2C interface
 * 
 * @param i2c_master_bus_configs: I2C master bus configs
 * @param i2c_master_handler: I2C master bus handler
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_FAIL: Failed to initalize I2C master bus
 */
esp_err_t i2c_init_master_bus(i2c_master_bus_config_t *i2c_master_bus_configs, i2c_master_bus_handle_t *i2c_master_handler);

/**
 * @brief Add new device to I2C bus
 * 
 * @param i2c_master_handler: I2C master bus handler
 * @param dev_configs: I2C device configs
 * @param dev_handler: Device I2C bus handler
 * @param iface_type: Device selected interface
 * 
 * @retval
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Device selected interface is not I2C
 *      - ESP_FAIL: Failed to add new device to I2C bus
 */
esp_err_t i2c_add_new_device(i2c_master_bus_handle_t i2c_master_handler, i2c_device_config_t *dev_configs, i2c_master_dev_handle_t *dev_handler, digital_interfaces_t iface_type);

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
esp_err_t i2c_read_bytes(void *i2c_configs, uint8_t reg_addr, uint8_t *read_data, size_t n_bytes, digital_interfaces_t iface_type);

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
esp_err_t i2c_write_byte(void *i2c_configs, uint8_t reg_addr, const uint8_t data, digital_interfaces_t iface_type);

#endif
