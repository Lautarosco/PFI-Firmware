#ifndef BMP280_HAL_MODE_H
#define BMP280_HAL_MODE_H

#include <registers_map/bmp280_registers.h>

#include <bmp280_structs.h>

#include <esp_err.h>


/**
 * @brief Reset device in order to clean device registers
 * 
 * @param bmp: Bmp280 object
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_Reset(bmp280_t bmp);

/**
 * @brief Set power mode. Available options are Sleep, Forced and Normal mode
 * 
 * @param bmp: Bmp280 object
 * @param power_mode: Power mode
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetPowerMode(bmp280_t bmp, bmp280_PowerMode_t power_mode);

/**
 * @brief Set temperature oversampling. More oversampling means more resolution
 * 
 * @param bmp: Bmp280 object
 * @param temp_os: Temperature oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetOsT(bmp280_t bmp, bmp280_OsT_t temp_os );

/**
 * @brief Set pressure oversampling. More oversampling means more resolution
 * 
 * @param bmp: Bmp280 object
 * @param press_os: Pressure oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetOsP(bmp280_t bmp, bmp280_OsP_t press_os );

/**
 * @brief Set sampling time (use only when power mode is set to 'Normal mode')
 * 
 * @param bmp: Bmp280 object
 * @param t_sb: Standby time
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetTsb(bmp280_t bmp, bmp280_TStandby_t t_sb );

/**
 * @brief Set IIR filter coefficient
 * 
 * @param bmp: Bmp280 object
 * @param coeff: IIR filter coefficient
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetIIR(bmp280_t bmp, bmp280_IIRCoeff_t coeff );

/**
 * @brief Set serial interface
 * 
 * @param bmp: Bmp280 object
 * @param serial: Serial interface
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetSerial(bmp280_t bmp, bmp280_SerialInterface_t serial );

/**
 * @brief Read raw pressure and temperature data from registers
 * 
 * @param bmp: Bmp280 object
 * @param adc_t: Variable to store raw temperature
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadRawTP(bmp280_t bmp, bmp280_s32_t * adc_t);

#endif
