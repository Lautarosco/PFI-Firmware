#ifndef BMP390_MODES_NAMES_H
#define BMP390_MODES_NAMES_H

#include <hardware_layer/bmp390_registers.h>

/**
 * @brief Get IIR coefficient name
 * 
 * @param iir_coef: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_iir_coef_name(bmp390_config_coef_t iir_coef);

/**
 * @brief Get selected ODR name
 * 
 * @param odr_sel: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_odr_sel_name(bmp390_odr_sel_t odr_sel);

/**
 * @brief Get pressure resolution name
 * 
 * @param osr_press: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_osr_press_name(bmp390_osr_press_t osr_press);

/**
 * @brief Get temperature resolution name
 * 
 * @param osr_temp: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_osr_temp_name(bmp390_osr_temp_t osr_temp);

/**
 * @brief Get power mode name
 * 
 * @param pwr_mode: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_pwr_mode_name(bmp390_pwr_ctrl_mode_t pwr_mode);

/**
 * @brief Get SPI mode name
 * 
 * @param spi_mode: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_spi_mode_name(bmp390_if_conf_reg_spi_t spi_mode);

/**
 * @brief Get I2C watchdog timeout name
 * 
 * @param i2c_wdt_tout: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_i2c_wdt_tout_name(bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout);

/**
 * @brief Get CMD name
 * 
 * @param i2c_wdt_tout: _
 * 
 * @retval Name of given BMP390 mode
 */
const char *bmp390_get_cmd_name(bmp390_cmd_t cmd);

#endif
