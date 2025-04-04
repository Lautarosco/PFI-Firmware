#ifndef BMP280_HAL_MODE_H
#define BMP280_HAL_MODE_H

#include <com.h>
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
 * @brief Reset device in order to clean device registers
 * 
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_Reset(i2c_master_dev_handle_t bmp280_i2c_bus_handler);

/**
 * @brief Get chip ID
 * 
 * @param bmp: Bmp280 object
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_GetChipID(bmp280_t * bmp);

/**
 * @brief Set power mode. Available options are Sleep, Forced and Normal mode
 * 
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param power_mode: Power mode
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetPowerMode(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_PowerMode_t power_mode);

/**
 * @brief Set temperature oversampling. More oversampling means more resolution
 * 
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param temp_os: Temperature oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL 
 */
esp_err_t bmp280_hal_SetOsT(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_OsT_t temp_os);

/**
 * @brief Set pressure oversampling. More oversampling means more resolution
 * 
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param press_os: Pressure oversampling
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetOsP(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_OsP_t press_os);

/**
 * @brief Set sampling time (use only when power mode is set to 'Normal mode')
 * 
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param t_sb: Standby time
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetTsb(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_TStandby_t t_sb);

/**
 * @brief Set IIR filter coefficient
 * 
 * @param bmp280_i2c_handler: bmp280 I2C bus handler
 * @param coeff: IIR filter coefficient
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetIIR(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_IIRCoeff_t coeff);

/**
 * @brief Set serial interface
 * 
 * @param bmp280_i2c_handler: bmp280 I2C bus handler
 * @param serial: Serial interface
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_SetSerial(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_SerialInterface_t serial);

/**
 * @brief Read raw pressure and temperature data from registers
 * 
 * @param bmp280_i2c_handler: bmp280 I2C bus handler
 * @param adc_t: Pointer to adc_t variable to store raw temperature
 * @param adc_p: Pointer to adc_p variable to store raw pressure
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadRawTP(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_s32_t * adc_t, bmp280_s32_t * adc_p);

/**
 * @brief Initialize I2C interface and add bmp280 to bus
 * 
 * @param i2c_master_bus_handler: Pointer to master I2C bus handler
 * @param bmp280_i2c_bus_handler: Pointer to bmp280 I2C bus handler
 * @param slave_addr: bmp280 address
 * @param sda: SDA (data) GPIO
 * @param scl: SCL (clock) GPIO
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_i2cInit(i2c_master_bus_handle_t * i2c_master_bus_handler, i2c_master_dev_handle_t * bmp280_i2c_bus_handler, uint8_t slave_addr, int sda, int scl);

/**
 * @brief Read data from registers
 * 
 * @param reg_addr: Register address
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param buff: Buffer to store read data
 * @param n_bytes: Bytes to read
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadSerial(uint8_t reg_addr, i2c_master_dev_handle_t bmp280_i2c_bus_handler, uint8_t * buff, int n_bytes);

/**
 * @brief Read data from registers
 * 
 * @param reg_addr: Register address
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * @param data: Data to be written in given register
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_WriteSerial(uint8_t reg_addr, i2c_master_dev_handle_t bmp280_i2c_bus_handler, uint8_t data);

/**
 * @brief Read compensation words stored in chip registers
 * 
 * @param comp_words: Pointer to compensation words
 * @param bmp280_i2c_bus_handler: bmp280 I2C bus handler
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
esp_err_t bmp280_hal_ReadCompWords(comp_words_t * comp_words, i2c_master_dev_handle_t bmp280_i2c_bus_handler);

/**
 * @brief Calculate pressure 'n' times and get average value. Should be used as an alternative to sea level pressure
 * 
 * @param bmp: Bmp280 object
 * @param n: Total samples
 * 
 * @return 64-bit calculated average pressure
 */
double bmp280_hal_GetRelativeP(bmp280_t bmp, int n);

#endif
