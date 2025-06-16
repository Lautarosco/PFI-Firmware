#ifndef SERIAL_H
#define SERIAL_H

#include <esp_err.h>

/**
 * @brief [M] Read register address and store bytes
 * 
 * @param dev_handler: Device serial interface handler
 * @param reg_addr: Register address
 * @param buff: Buffer to store data
 * @param len: Length of data
 * 
 * @retval
 *      - ESP_OK
 *      - ESP_FAIL
 */
typedef esp_err_t (*serial_read_func)(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len);

/**
 * @brief [M] Write data to register address
 * 
 * @param dev_handler: Device serial interface handler
 * @param reg_addr: Register address
 * @param data: Data to be written
 * @param len: Length of data
 * 
 * @retval
 *      - ESP_OK
 *      - ESP_FAIL
 */
typedef esp_err_t (*serial_write_func)(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len);

typedef struct serial_interface {

    /**
     * @brief [M] Read register address and store bytes
     * 
     * @param dev_handler: Device serial interface handler
     * @param reg_addr: Register address
     * @param buff: Buffer to store data
     * @param len: Length of data
     * 
     * @retval
     *      - ESP_OK
     *      - ESP_FAIL
     */
    serial_read_func read;

    /**
     * @brief [M] Write data to register address
     * 
     * @param dev_handler: Device serial interface handler
     * @param reg_addr: Register address
     * @param data: Data to be written
     * @param len: Length of data
     * 
     * @retval
     *      - ESP_OK
     *      - ESP_FAIL
     */
    serial_write_func write;

    /* Device serial handler */
    void *handler;

    /* Device address */
    uint8_t addr;
} serial_interface_t;

#endif
