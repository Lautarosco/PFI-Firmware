#ifndef BMP280_HAL_API_H
#define BMP280_HAL_API_H

#include <bmp280_registers.h>
#include <bmp280_data_types.h>

#include <bmp280_structs.h>

#include <esp_err.h>
#include <stdint.h>

typedef struct comp_words {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} comp_words_t;


/**
 * @brief Reset device in order to clean its registers
 * 
 * @param dev_iface: bmp280 serial interface
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_Reset(dev_serial_iface_t *dev_iface);

/**
 * @brief Get chip ID and store it into given buffer
 * 
 * @param dev_iface: bmp280 serial interface
 * @param buff: buffer to store chip ID
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_GetChipID(dev_serial_iface_t *dev_iface, uint8_t *buff);

/**
 * @brief Set power mode. Available options are Sleep, Forced and Normal mode
 * 
 * @param dev_iface: bmp280 serial interface
 * @param power_mode: Power mode
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetPowerMode(dev_serial_iface_t *dev_iface, bmp280_PowerMode_t power_mode);

/**
 * @brief Set temperature oversampling. More oversampling means more resolution
 * 
 * @param dev_iface: bmp280 serial interface
 * @param temp_os: Temperature oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetOsT(dev_serial_iface_t *dev_iface, bmp280_OsT_t temp_os);

/**
 * @brief Set pressure oversampling. More oversampling means more resolution
 * 
 * @param dev_iface: bmp280 serial interface
 * @param press_os: Pressure oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetOsP(dev_serial_iface_t *dev_iface, bmp280_OsP_t press_os);

/**
 * @brief Set sampling time (use only when power mode is set to 'Normal mode')
 * 
 * @param dev_iface: bmp280 serial interface
 * @param t_sb: Standby time
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetTsb(dev_serial_iface_t *dev_iface, bmp280_TStandby_t t_sb);

/**
 * @brief Set IIR filter coefficient
 * 
 * @param dev_iface: bmp280 serial interface
 * @param coeff: IIR filter coefficient
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetIIR(dev_serial_iface_t *dev_iface, bmp280_IIRCoeff_t coeff);

/**
 * @brief Set serial interface
 * 
 * @param dev_iface: bmp280 serial interface
 * @param serial: Serial interface
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetSerial(dev_serial_iface_t *dev_iface, bmp280_SerialInterface_t serial);

/**
 * @brief Read raw pressure and temperature data from registers
 * 
 * @param dev_iface: bmp280 serial interface
 * @param adc_t: Pointer to adc_t variable to store raw temperature
 * @param adc_p: Pointer to adc_p variable to store raw pressure
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadRawTP(dev_serial_iface_t *dev_iface, bmp280_s32_t * adc_t, bmp280_s32_t * adc_p);

/**
 * @brief Read data from registers
 * 
 * @param dev_iface: bmp280 serial interface
 * @param reg_addr: Register address
 * @param buff: Buffer to store read data
 * @param n_bytes: Bytes to read
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadSerial(dev_serial_iface_t *dev_iface, uint8_t reg_addr, uint8_t * buff, int n_bytes);

/**
 * @brief Read data from registers
 * 
 * @param dev_iface: bmp280 serial interface
 * @param reg_addr: Register address
 * @param data: Data to be written in given register
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_WriteSerial(dev_serial_iface_t *dev_iface, uint8_t reg_addr, uint8_t data);

/**
 * @brief Read compensation words stored in chip registers
 * 
 * @param dev_iface: bmp280 serial interface
 * @param comp_words: Pointer to compensation words
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadCompWords(dev_serial_iface_t *dev_iface, comp_words_t * comp_words);

#endif
