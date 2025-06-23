#ifndef BMP390_HW_LAYER_H
#define BMP390_HW_LAYER_H

#include <stdbool.h>
#include <esp_err.h>
#include <hardware_layer/bmp390_registers.h>

typedef struct device_interface device_interface_t;

/**
 * @brief Read chip ID and store it in <read_data> buffer
 * 
 * @param dev_iface: Generic device interface settings
 * @param read_data: Buffer to store bytes after reading BMP290 CHIP_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: <read_data> parameter is NULL
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_get_chip_id(device_interface_t dev_iface, uint8_t *read_data);

/**
 * @brief Read mask revision of the ASIC and store it in <read_data> buffer
 * 
 * @param dev_iface: Generic device interface settings
 * @param read_data: Buffer to store bytes after reading BMP290 REV_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: <read_data> parameter is NULL
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_get_rev_id(device_interface_t dev_iface, uint8_t *read_data);

/**
 * @brief Check sensor error conditions
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: If no errors occurred
 *      -   ESP_FAIL: Errors exist
 */
esp_err_t bmp390_hwl_err(device_interface_t dev_iface);


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
esp_err_t bmp390_hwl_cmd_rdy_status(device_interface_t dev_iface);

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
esp_err_t bmp390_hwl_drdy_press_status(device_interface_t dev_iface);

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
esp_err_t bmp390_hwl_drdy_temp_status(device_interface_t dev_iface);

/**
 * @brief Check if device was powered up or soft reseted (Cleared on read)
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   1: After device was powered up or soft reseted
 *      -   0: Shut down or still reseting
 *      -  (-1): If read operation fails or errors exist
 */
int bmp390_hwl_detect_soft_reset(device_interface_t dev_iface);

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
esp_err_t bmp390_hwl_spi_en(device_interface_t dev_iface, bmp390_if_conf_reg_spi_t spi_mode);

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
esp_err_t bmp390_hwl_i2c_en_wdt(device_interface_t dev_iface, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout);

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
esp_err_t bmp390_hwl_i2c_dis_wdt(device_interface_t dev_iface);

/**
 * @brief Set power mode
 * 
 * @param dev_iface: Generic device interface settings
 * @param pwr_mode: Power mode (Sleep/Normal/Forced)
 * 
 * @retval
 *      -   ESP_OK: Power mode setted successfully
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_set_pwr_mode(device_interface_t dev_iface, bmp390_pwr_ctrl_mode_t pwr_mode);

/**
 * @brief Enable pressure sensor
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: Pressure sensor successfully enabled
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_press_en(device_interface_t dev_iface);

/**
 * @brief Enable temperature sensor
 * 
 * @param dev_iface: Generic device interface settings
 * 
 * @retval
 *      -   ESP_OK: Temperature sensor successfully enabled
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_temp_en(device_interface_t dev_iface);

/**
 * @brief Set pressure resolution
 * 
 * @param dev_iface: Generic device interface settings
 * @param osr_press: Oversampling rate
 * 
 * @retval
 *      -   ESP_OK: Set pressure resolution success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <osr_press> parameter
 */
esp_err_t bmp390_hwl_set_osr_press(device_interface_t dev_iface, bmp390_osr_press_t osr_press);

/**
 * @brief Set temperature resolution
 * 
 * @param dev_iface: Generic device interface settings
 * @param osr_temp: Oversampling rate
 * 
 * @retval
 *      -   ESP_OK: Set temperature resolution success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <osr_temp> parameter
 */
esp_err_t bmp390_hwl_set_osr_temp(device_interface_t dev_iface, bmp390_osr_temp_t osr_temp);

/**
 * @brief Set output rate
 * 
 * @param dev_iface: Generic device interface settings
 * @param odr_sel: Output data rate
 * 
 * @retval
 *      -   ESP_OK: Set output dara rate success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <odr_sel> parameter
 */
esp_err_t bmp390_hwl_set_odr(device_interface_t dev_iface, bmp390_odr_sel_t odr_sel);

/**
 * @brief Set IIR filter coefficient
 * 
 * @param dev_iface: Generic device interface settings
 * @param iir_coef: Filter coefficient
 * 
 * @retval
 *      -   ESP_OK: Set IIR filter coefficient success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <iir_coef> parameter
 */
esp_err_t bmp390_hwl_set_iir_coef(device_interface_t dev_iface, bmp390_config_coef_t iir_coef);

/**
 * @brief Execute a command from the sensor available commands
 * 
 * @param dev_iface: Generic device interface settings
 * @param cmd_sel: Command to be executed
 * 
 * @retval
 *      -   ESP_OK: Set command success
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_exec_cmd(device_interface_t dev_iface, bmp390_cmd_t cmd_sel);

/**
 * @brief Perform a burst read from press_xlsb to temp_msb (6 bytes in a row)
 * 
 * @param dev_iface: Generic device interface settings
 * @param adc_temp: Pointer to store raw temperature measurement
 * @param adc_press: Pointer to store raw pressure measurement
 * 
 * @retval
 *      -   ESP_OK: Set command success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Pointers are NULL
 */
esp_err_t bmp390_hwl_read_raw_data(device_interface_t dev_iface, uint32_t *adc_temp, uint32_t *adc_press);

/**
 * @brief Read any BMP390 register and check a specific mode value
 * 
 * @note This function is for debbuging purposes to check if registers contents are consistent with setted configurations
 * 
 * @param dev_iface: Generic device interface settings
 * @param mode_name: Name of the register mode
 * @param reg_addr: Register
 * @param n_bits: Total bits used by the register mode
 * @param bit_start_pos: Starting bit position of the register mode
 * 
 * @retval
 *      -   ESP_OK: Check register mode value success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Bits count (<bit_start_pos> + <n_bits>) exceeds 8 bits length
 */
void bmp390_hwl_get_mode_val(device_interface_t dev_iface, uint8_t reg_addr, uint8_t starting_bit_pos);

#endif
