#ifndef BMP390_HAL_API_H
#define BMP390_HAL_API_H

#include <stdbool.h>
#include <esp_err.h>

typedef struct device_interface device_interface_t;

typedef struct if_conf {
    /* Configure SPI interface mode for primary interface. 0: SPI 4-wire mode, 1: SPI 3-wire mode */
    bool spi3;

    /* Enable I2C watchdog timer, backed by NVM. 0: Disable WDT, 1: Enable WDT */
    bool i2c_wdt_en;

    /* Select timer period for I2C watchdog, backed by NVM. 0: I2C watchdog timeout after 1.25 ms, 1: I2C watchdog timeout after 40 ms */
    bool i2c_wdt_sel;
} if_conf_t;

/**
 * @brief Read chip ID and store it in <read_data> buffer
 * 
 * @param dev_iface: Generic device interface settings
 * @param read_data: Buffer to store bytes after reading BMP290 CHIP_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: Selected interface <iface_sel> does not match with the function provided. i. e, iface_sel = SPI and read function is for I2C
 *      -   ESP_ERR_TIMEOUT: Operation timeout because the bus is busy or hardware crash
 */
esp_err_t bmp390_hal_get_chip_id(device_interface_t dev_iface, uint8_t *read_data);

/**
 * @brief Read mask revision of the ASIC and store it in <read_data> buffer
 * 
 * @param dev_iface: Generic device interface settings
 * @param read_data: Buffer to store bytes after reading BMP290 REV_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: Selected interface <iface_sel> does not match with the function provided. i. e, iface_sel = SPI and read function is for I2C
 *      -   ESP_ERR_TIMEOUT: Operation timeout because the bus is busy or hardware crash
 */
esp_err_t bmp390_hal_get_rev_id(device_interface_t dev_iface, uint8_t *read_data);

/**
 * @brief Check sensor error conditions
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: If no errors occurred
 *      -   ESP_FAIL: Errors exist
 */
esp_err_t bmp390_hal_err(device_interface_t dev_iface);


/**
 * @brief Check if command decoder is ready to accept a new command
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: Command decoder is ready
 *      -   ESP_ERR_NOT_FINISHED: Command in progress
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_cmd_rdy_status(device_interface_t dev_iface);

/**
 * @brief Check if pressure data is ready
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: Pressure data is ready
 *      -   ESP_ERR_NOT_FINISHED: Pressure data is not ready
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_drdy_press_status(device_interface_t dev_iface);

/**
 * @brief Check if temperature data is ready
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: Temperature data is ready
 *      -   ESP_ERR_NOT_FINISHED: Temperature data is not ready
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_drdy_temp_status(device_interface_t dev_iface);

/**
 * @brief Check if device was powered up or soft reseted (Cleared on read)
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   1: After device was powered up or soft reseted
 *      -   0: Shut down or still reseting
 *      -  -1: If read operation fails or errors exist
 */
int bmp390_hal_detect_soft_reset(device_interface_t dev_iface);

/**
 * @brief Enable SPI interface
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: SPI successfully enabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_spi_en(device_interface_t dev_iface, bmp390_if_conf_reg_spi_t spi_mode);

/**
 * @brief Enable I2C watchdog timeout
 * 
 * @param dev_iface: Generic device interface settings
 * @param i2c_wdt_tout: Watchdog timeout
 * 
 * @retval
 *      -   ESP_OK: I2C watchdog timeout successfully enabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_i2c_en_wdt(device_interface_t dev_iface, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout);

/**
 * @brief Disable I2C watchdog timeout
 * 
 * @param dev_iface: Generic device interface settings
 * @param i2c_wdt_tout: Watchdog timeout
 * 
 * @retval
 *      -   ESP_OK: I2C successfully disabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hal_i2c_dis_wdt(device_interface_t dev_iface);

#endif
