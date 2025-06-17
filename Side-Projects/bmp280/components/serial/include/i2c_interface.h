#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <driver/i2c_master.h>  /* ESP-IDF I2C driver */
#include <serial_types.h>
#include <esp_err.h>

/* ================= Master functions ================= */

/**
 * @brief Initialize I2C interface
 * 
 * @param bus_handler: Pointer to master bus handler
 * @param sda: I2C data
 * @param scl: I2C clock
 * 
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG Parameter error
 *      - ESP_FAIL Driver installation error
 */
esp_err_t i2c_init(i2c_master_bus_handle_t * master_handler, int sda, int scl);

/**
 * @brief Add new device to I2C bus
 * 
 * @param master_handler:
 * @param dev_config:
 * @param dev_handler:
 * @param serial_iface_type: Device type of serial interface
 * 
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG if serial interface is not I2C
 *      - ESP_FAIL Failed to add device to I2C bus
 */
esp_err_t i2c_add_new_device(i2c_master_bus_handle_t master_handler, i2c_device_config_t *dev_config, i2c_master_dev_handle_t *dev_handler, serial_iface_type_t serial_iface_type);

/**
 * @brief Seek devices connected to I2C bus
 * 
 * @param bus_handler: Master bus handler
 * @param slave_addr: Address of sensor to be found on I2C bus
 * 
 * @retval
 *      - true if found
 *      - false if not found
 */
// bool i2c_scan(i2c_master_bus_handle_t bus_handler, uint8_t slave_addr);

/* ================= Device functions ================= */

/**
 * @brief Read bytes
 * 
 * @param dev_handler: Device bus handler
 * @param reg_addr: Register to read from
 * @param buff: Variable to store data
 * @param len: Total bytes to be read
 * @param serial_iface_type: Device type of serial interface
 * 
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t i2c_read_bytes(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len, serial_iface_type_t serial_iface_type);

/**
 * @brief Write bytes
 * 
 * @param dev_handler: Device bus handler
 * @param reg_addr: Register to be written
 * @param data: Data to be written
 * @param len: Total bytes to be read
 * @param serial_iface_type: Device type of serial interface
 * 
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t i2c_write_bytes(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len, serial_iface_type_t serial_iface_type);

#endif
