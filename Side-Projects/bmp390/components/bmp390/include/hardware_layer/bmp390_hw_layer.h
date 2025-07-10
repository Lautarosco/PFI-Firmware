#ifndef BMP390_HW_LAYER_H
#define BMP390_HW_LAYER_H

#include <stdbool.h>
#include <esp_err.h>
#include <hardware_layer/bmp390_registers.h>
#include <driver/i2c_master.h>

/* ========== Public defines ========== */

/**
 * @brief Concats 2 bytes and return a new 16-bit value
 * 
 * @param msb: Most significant byte
 * @param lsb: Less significant byte
 * 
 * @retval unsigned 16-bit new value
 */
inline uint16_t CONCAT_BYTES(uint8_t msb, uint8_t lsb) {return (((uint16_t) msb << 8) | (uint16_t) lsb);}

/**
 * @brief Set <mode> bits of <reg> register starting at <starting_bit_pos> bit position
 * 
 * @note
 *      i.e: <reg> = xxx? ?xxx --> We aim to set mode = 01 in ?? bits of reg. Then,
 *      <mask> = 11
 *      <mode> = 01
 *      <starting_bit_pos> = 3
 * 
 * @param reg: Register address
 * @param mask: Bits to be set
 * @param mode: New bit/s
 * @param starting_bit_pos: Starting bit position
 * 
 * @retval
 *      - Register with <mode> bit/s set
 */
inline uint8_t SET_BITS(uint8_t reg, uint8_t mask, uint8_t mode, uint8_t starting_bit_pos) {return ((reg & ~(mask << starting_bit_pos)) | (mode << starting_bit_pos));}

/* ========== Public structs ========== */

typedef struct device_interface device_interface_t;

typedef struct bmp390_calib_data {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
} bmp390_calib_data_t;

/* ========== Public functions ========== */

/**
 * @brief Read chip ID and store it in <read_data> buffer
 * 
 * @param i2c_bmp_handler: I2C bus BMP390 handler
 * @param read_data: Buffer to store bytes after reading BMP290 CHIP_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: <read_data> parameter is NULL
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_get_chip_id(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t *read_data);

/**
 * @brief Read mask revision of the ASIC and store it in <read_data> buffer
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param read_data: Buffer to store bytes after reading BMP290 REV_ID register
 * 
 * @retval
 *      -   ESP_OK: Read operation was successfully completed
 *      -   ESP_ERR_INVALID_ARG: <read_data> parameter is NULL
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_get_rev_id(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t *read_data);

/**
 * @brief Check sensor error conditions
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: If no errors occurred
 *      -   ESP_FAIL: Errors exist
 */
esp_err_t bmp390_hwl_err(i2c_master_dev_handle_t i2c_bmp_handler);


/**
 * @brief Check if command decoder is ready to accept a new command
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: Command decoder is ready
 *      -   ESP_ERR_NOT_FINISHED: Command in progress
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_cmd_rdy_status(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Check if pressure data is ready
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: Pressure data is ready
 *      -   ESP_ERR_NOT_FINISHED: Pressure data is not ready
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_drdy_press_status(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Check if temperature data is ready
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: Temperature data is ready
 *      -   ESP_ERR_NOT_FINISHED: Temperature data is not ready
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_drdy_temp_status(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Check if device was powered up or soft reseted (Cleared on read)
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   1: After device was powered up or soft reseted
 *      -   0: Shut down or still reseting
 *      -  (-1): If read operation fails or errors exist
 */
int bmp390_hwl_detect_soft_reset(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Enable SPI interface
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: SPI successfully enabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_spi_en(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_if_conf_reg_spi_t spi_mode);

/**
 * @brief Enable I2C watchdog timeout
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param i2c_wdt_tout: Watchdog timeout
 * 
 * @retval
 *      -   ESP_OK: I2C watchdog timeout successfully enabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_i2c_en_wdt(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout);

/**
 * @brief Disable I2C watchdog timeout
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param i2c_wdt_tout: Watchdog timeout
 * 
 * @retval
 *      -   ESP_OK: I2C successfully disabled
 *      -   ESP_ERR_INVALID_ARG: No interface selected
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_i2c_dis_wdt(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Set power mode
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param pwr_mode: Power mode (Sleep/Normal/Forced)
 * 
 * @retval
 *      -   ESP_OK: Power mode setted successfully
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_set_pwr_mode(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_pwr_ctrl_mode_t pwr_mode);

/**
 * @brief Enable pressure sensor
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: Pressure sensor successfully enabled
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_press_en(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Enable temperature sensor
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * 
 * @retval
 *      -   ESP_OK: Temperature sensor successfully enabled
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_temp_en(i2c_master_dev_handle_t i2c_bmp_handler);

/**
 * @brief Set pressure resolution
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param osr_press: Oversampling rate
 * 
 * @retval
 *      -   ESP_OK: Set pressure resolution success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <osr_press> parameter
 */
esp_err_t bmp390_hwl_set_osr_press(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_osr_press_t osr_press);

/**
 * @brief Set temperature resolution
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param osr_temp: Oversampling rate
 * 
 * @retval
 *      -   ESP_OK: Set temperature resolution success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <osr_temp> parameter
 */
esp_err_t bmp390_hwl_set_osr_temp(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_osr_temp_t osr_temp);

/**
 * @brief Set output rate
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param odr_sel: Output data rate
 * 
 * @retval
 *      -   ESP_OK: Set output dara rate success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <odr_sel> parameter
 */
esp_err_t bmp390_hwl_set_odr(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_odr_sel_t odr_sel);

/**
 * @brief Set IIR filter coefficient
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param iir_coef: Filter coefficient
 * 
 * @retval
 *      -   ESP_OK: Set IIR filter coefficient success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Invalid <iir_coef> parameter
 */
esp_err_t bmp390_hwl_set_iir_coef(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_config_coef_t iir_coef);

/**
 * @brief Execute a command from the sensor available commands
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param cmd_sel: Command to be executed
 * 
 * @retval
 *      -   ESP_OK: Set command success
 *      -   ESP_FAIL: If read operation fails or errors exist
 */
esp_err_t bmp390_hwl_exec_cmd(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_cmd_t cmd_sel);

/**
 * @brief Perform a burst read from press_xlsb to temp_msb (6 bytes in a row)
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param adc_temp: Pointer to store raw temperature measurement
 * @param adc_press: Pointer to store raw pressure measurement
 * 
 * @retval
 *      -   ESP_OK: Set command success
 *      -   ESP_FAIL: If read operation fails or errors exist
 *      -   ESP_ERR_INVALID_ARG: Pointers are NULL
 */
esp_err_t bmp390_hwl_read_raw_data(i2c_master_dev_handle_t i2c_bmp_handler, uint32_t *adc_temp, uint32_t *adc_press);

/**
 * @brief Read any operation mode of a given register and return its actual value
 * 
 * @note Given buffer is cleared before its used
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param reg_addr: Register to be read
 * @param mode: mode
 * @param msg: Buffer to store answer
 * @param msg_length: Length of buffer
 * 
 * @retval
 *      - Mode value
 *      - (-1) If mode was not found
 */
int bmp390_hwl_get_mode_val(i2c_master_dev_handle_t i2c_bmp_handler, uint8_t reg_addr, uint8_t mode, char *msg, size_t msg_length);

/**
 * @brief Read BMP390 CALIBRATION_DATA registers (0x31 to 0x45) and convert each compensation coefficient into a floating point number
 * 
 * @param i2c_bmp_handler: I2C BMP390 handler
 * @param calib_data: Pointer to calibration data structure
 * 
 * @retval
 *      - ESP_OK: success
 *      - ESP_FAIL
 */
esp_err_t bmp390_hwl_get_comp_coefs(i2c_master_dev_handle_t i2c_bmp_handler, bmp390_calib_data_t *calib_data);

#endif
