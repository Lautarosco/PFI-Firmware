#ifndef INTERFACE_H
#define INTERFACE_H

#include <esp_err.h>
#include <stdint.h>
#include <interface_types.h>

typedef struct device_interface {
    /**
     * @brief Pointer to generic read function
     * 
     * @param iface_cfg: Settings of selected interface
     * @param iface_sel: Selected interface
     * @param reg_addr: Address to be read
     * @param read_data: Stored data after reading <reg_addr> register
     * @param n_bytes: Total bytes to read
     * 
     * @retval
     *      -   ESP_OK: Read operation was successfully completed
     *      -   ESP_ERR_INVALID_ARG: Selected interface <iface_sel> does not match with the function provided. i. e, iface_sel = SPI and read function is for I2C
     *      -   ESP_ERR_TIMEOUT: Operation timeout because the bus is busy or hardware crash
     */
    esp_err_t (*read_bytes)(void *iface_cfg, digital_interfaces_t iface_sel, uint8_t reg_addr, uint8_t *read_data, int n_bytes);

    /**
     * @brief Pointer to generic write function
     * 
     * @param iface_cfg: Settings of selected interface
     * @param iface_sel: Selected interface
     * @param reg_addr: Address to be read
     * @param write_data: Data to be written to <reg_addr> address
     * @param n_bytes: Total bytes to read
     * 
     * @retval
     *      -   ESP_OK: Write operation was successfully completed
     *      -   ESP_ERR_INVALID_ARG: Selected interface <iface_sel> does not match with the function provided. i. e, iface_sel = SPI and read function is for I2C
     *      -   ESP_ERR_TIMEOUT: Operation timeout because the bus is busy or hardware crash
     */
    esp_err_t (*write_bytes)(void *iface_cfg, digital_interfaces_t iface_sel, uint8_t reg_addr, uint8_t *write_data, int n_bytes);
    
    /* Settings of selected interface --> Here it should be added a struct with everything needed for the selected interface to work */
    void *iface_cfg;

    /* Type of interface */
    digital_interfaces_t iface_sel;
} device_interface_t;

#endif