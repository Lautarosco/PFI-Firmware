#ifndef COM_H
#define COM_H

#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>

typedef struct sensors_addr {
    const char * name;  /* Name of sensor */
    uint8_t addr;       /* Address of sensor */
    bool __found;       /** @private I2C flag */
} sensors_addr_t;

/**
 * @brief Initialize I2C interface
 * 
 * @param sda: I2C data
 * @param scl: I2C clock
 * 
 * @return ESP_OK Success - ESP_ERR_INVALID_ARG Parameter error - ESP_FAIL Driver installation error
 */
esp_err_t i2c_init(int sda, int scl);

/**
 * @brief Read serial data through I2C interface
 * 
 * @param sensor_addr: Address of sensor
 * @param reg_addr: Register address
 * @param buff: Buffer to store read data
 * @param len: Total bytes to be read
 * 
 * @return ESP_OK if success
 */
esp_err_t i2c_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t * buff, int len);

/**
 * @brief Write serial data through I2C interface
 * 
 * @param sensor_addr: Address of sensor
 * @param reg_addr: Register address
 * @param data: Data to be written
 * 
 * @return ESP_OK if success
 */
esp_err_t i2c_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Seek devices connected to I2C bus
 * 
 * @param sensors: Array of sensors connected to I2C bus
 * @param len: Length of array
 * 
 * @retval true if found - false
 */
void i2c_scan(sensors_addr_t * sensors, int len);

#endif
