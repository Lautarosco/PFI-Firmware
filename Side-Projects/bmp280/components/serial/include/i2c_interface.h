#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <driver/i2c_master.h>
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
 * @brief Add slave to I2C bus
 * 
 * @param master_handler: Master bus handler
 * @param dev_handler: Slave bus handler
 * @param addr: Slave address
 * @param addr_len: Length of address
 * @param scl_freq: SCL clock frequency
 * 
 * @return
 *      - ESP_OK: Create I2C master device successfully.
 *      - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
 *      - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
 */
esp_err_t i2c_add_device(i2c_master_bus_handle_t master_handler, i2c_master_dev_handle_t * dev_handler, uint8_t addr, i2c_addr_bit_len_t addr_len, int scl_freq);

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
 * @param dev_handler: Slave bus handler
 * @param reg_addr: Register to read from
 * @param data: Variable to store data
 * @param len: Total bytes to be read
 * 
 * @return
 *      - ESP_OK: I2C master transmit-receive success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t i2c_read_bytes(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len);

/**
 * @brief Write bytes
 * 
 * @param dev_handler: Slave bus handler
 * @param reg_addr: Register to be written
 * @param data: Data to be written
 * 
 * @return
 *      - ESP_OK: I2C master transmit success
 *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid
 *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
 */
esp_err_t i2c_write_bytes(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len);

#endif
