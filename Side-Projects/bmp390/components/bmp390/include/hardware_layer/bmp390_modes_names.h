#ifndef BMP390_MODES_NAMES_H
#define BMP390_MODES_NAMES_H

#include <hardware_layer/bmp390_registers.h>

/* ========== Public functions ========== */

/**
 * @brief Get IIR coefficient name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_iir_coef_name(unsigned int mode_value);

/**
 * @brief Get selected ODR name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_odr_sel_name(unsigned int mode_value);

/**
 * @brief Get pressure resolution name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_osr_press_name(unsigned int mode_value);

/**
 * @brief Get temperature resolution name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_osr_temp_name(unsigned int mode_value);

/**
 * @brief Get power mode name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_pwr_mode_name(unsigned int mode_value);

/**
 * @brief Get SPI mode name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_spi_mode_name(unsigned int mode_value);

/**
 * @brief Get I2C watchdog timeout name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_i2c_wdt_tout_name(unsigned int mode_value);

/**
 * @brief Get CMD name
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_cmd_name(unsigned int mode_value);

/**
 * @brief Get pressure status (enable or disable)
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_press_status(unsigned int mode_value);

/**
 * @brief Get temperature status (enable or disable)
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_temp_status(unsigned int mode_value);

/**
 * @brief Get I2C watchdog timeout status (enable or disable)
 * 
 * @param mode_value: Actual value of a register mode
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_i2c_wdt_en_status(unsigned int mode_value);

#endif
