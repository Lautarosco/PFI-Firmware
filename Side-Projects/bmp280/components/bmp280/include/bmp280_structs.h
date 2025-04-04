#ifndef BMP280_STRUCTS_H
#define BMP280_STRUCTS_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>


typedef struct __i2c_cfg {
    int sda;                                        /* I2C SDA (data) GPIO */
    int scl;                                        /* I2C SCL (clock) GPIO */
    i2c_master_dev_handle_t bmp280_i2c_bus_handler;     /* bmp280 I2C bus handler */
} __i2c_cfg_t;

typedef struct bmp280 bmp280_t;

typedef struct bmp280 {
    uint8_t addr;       /* [A] Address - 0x76 if SDO = 0 or 0x77 if SDO = 1 */
    __i2c_cfg_t i2c;    /* [A] I2C interface settings */
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
    esp_err_t (* init)(bmp280_t * bmp, i2c_master_bus_handle_t * master_i2c_bus_handler, int addr, int sda, int scl);

    /**
     * @brief [M] Compensate raw temperature values stored in registers and return temperature
     * in degrees (°C) with a resolution of .01
     * 
     * @return 32-bit signed int measured temperature
     */
    double (* get_temp)(void);

    /**
     * @brief [M] Compensate raw pressure values stored in registers and return
     * actual pressure in hecto pascals (hPa)
     * 
     * @return 32-bit signed int measured pressure
     */
    double (* get_press)(void);

    /**
     * @brief [M] Measure pressure and temperature
     * 
     * @param bmp: Bmp280 object
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (* measure)(i2c_master_dev_handle_t bmp280_i2c_bus_handler);

    /**
     * @brief [M] Calculate altitude based on measured pressure 'p' and relative pressure 'p0'. The latter should be calculated with
     * 'bmp280_GetRelativeP' function or use sea level value, ~1013.25 hPa (value taken from https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf, p. 16, 3.6)
     * 
     * @param p: Measured pressure
     * @param p0: Relative pressure
     * 
     * @return 64-bit estimated altitude
     */
    double (* get_altitude)(double p, double p0);
} bmp280_t;

#endif
