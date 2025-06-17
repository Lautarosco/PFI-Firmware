#ifndef SERIAL_H
#define SERIAL_H

#include <i2c_interface.h>
#include <serial_types.h>
#include <esp_err.h>

typedef struct dev_serial_iface {
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
    esp_err_t (*read_func)(void *dev_handler, uint8_t reg_addr, uint8_t *buff, uint16_t len, serial_iface_type_t serial_iface_type);

    /**
     * @brief Write bytes
     * 
     * @param dev_handler: Device bus handler
     * @param reg_addr: Register to be written
     * @param data: Data to be written reg_addr
     * @param len: Total bytes to be read
     * @param serial_iface_type: Device type of serial interface
     * 
     * @return
     *      - ESP_OK: I2C master transmit success
     *      - ESP_ERR_INVALID_ARG: I2C master transmit parameter invalid
     *      - ESP_ERR_TIMEOUT: Operation timeout(larger than xfer_timeout_ms) because the bus is busy or hardware crash
     */
    esp_err_t (*write_func)(void *dev_handler, uint8_t reg_addr, const uint8_t data, uint16_t len, serial_iface_type_t serial_iface_type);

    /* [A] Device serial handler */
    void *handler;

    /* Type of serial interface */
    serial_iface_type_t type;

    union
    {
        i2c_device_config_t i2c_cfg;
        // spi_struct_t spi; // TODO
    };
} dev_serial_iface_t;

#endif
