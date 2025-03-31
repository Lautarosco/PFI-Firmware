#ifndef BMP280_STRUCTS_H
#define BMP280_STRUCTS_H

#include <stdint.h>
#include <esp_err.h>
#include <data_types/bmp280_data_types.h>
#include <bmp280_ll_drivers/measurements/compensation_words.h>


typedef struct __i2c_cfg {
    int sda;
    int scl;
} __i2c_cfg_t;

typedef struct bmp280 bmp280_t;

typedef struct bmp280 {
    uint8_t addr;       /* [A] Address - 0x76 if SDO = 0 or 0x77 if SDO = 1 */
    __i2c_cfg_t i2c;    /* [A] I2C interface pinout */
    uint8_t id;         /* [A] Chip ID - should be 0x58 */
    
    /**
     * @brief [M] Initialize Bmp280 Class
     * 
     * @param bmp: Pointer to Bmp280 object
     * @param addr: Sensor address
     * @param sda: SDA data pin
     * @param scl: SCL clock pin
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (* init)(bmp280_t * bmp, int addr, int sda, int scl);

    /**
     * @brief [M] Compensate raw temperature values stored in registers and return temperature
     * in degrees (°C) with a resolution of .01
     * 
     * @param bmp: Bmp280 object
     * @param adc_t: Raw data stored in registers
     * @param comp_words: Compensation words
     * 
     * @return 32-bit signed int measured temperature
     */
    bmp280_s32_t (* get_temp)(bmp280_t * bmp, bmp280_s32_t adc_t, compensation_words_t comp_words);

    /**
     * @brief Measure pressure and temperature
     * 
     * @param bmp: Bmp280 object
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (* measure)(bmp280_t * bmp);
} bmp280_t;

#endif
