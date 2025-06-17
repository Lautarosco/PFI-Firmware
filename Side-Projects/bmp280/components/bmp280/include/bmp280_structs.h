#ifndef BMP280_STRUCTS_H
#define BMP280_STRUCTS_H

/* Gereral headers */
#include <stdint.h>
#include <esp_err.h>
#include <stdbool.h>

/* Components headers */
#include <serial.h>         /* Serial interfaces driver */

typedef struct bmp280 bmp280_t;

typedef struct bmp280 {
    /* [A] Chip ID - should be 0x58 */
    uint8_t id;

    /* [A] Flag to check if Bmp280 object was already initialized */
    bool init_ok;

    /* [A] bmp serial interface */
    dev_serial_iface_t serial_iface;
    
    /**
     * @brief [M] Initialize Bmp280 Class
     * 
     * @param bmp: Pointer to Bmp280 object
     * @param bmp_iface: bmp serial interface
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (*init)(bmp280_t *bmp, dev_serial_iface_t *bmp_iface);

    /**
     * @brief [M] Compensate raw temperature values stored in registers and return temperature
     * in degrees (°C) with a resolution of .01
     * 
     * @return 32-bit signed int measured temperature
     */
    double (*get_temperature)(void);

    /**
     * @brief [M] Compensate raw pressure values stored in registers and return
     * actual pressure in hecto pascals (hPa)
     * 
     * @return 32-bit signed int measured pressure
     */
    double (*get_pressure)(void);

    /**
     * @brief [M] Measure pressure and temperature
     * 
     * @param bmp: Bmp280 object
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (*measure)(i2c_master_dev_handle_t bmp280_i2c_bus_handler);

    /**
     * @brief [M] Calculate altitude based on measured pressure 'p' and relative pressure 'p0'. The latter should be calculated with
     * 'bmp280_GetRelativeP' function or use sea level value, ~1013.25 hPa (value taken from https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf, p. 16, Sec. 3.6)
     * 
     * @param p: Measured pressure
     * @param p0: Relative pressure
     * 
     * @return 64-bit estimated altitude
     */
    double (*get_altitude)(double p, double p0);

    /**
     * @brief [M] Calculate pressure 'n' times and get average value. Should be used as an alternative to sea level pressure
     * 
     * @param bmp: Bmp280 object
     * @param n: Total samples
     * 
     * @return 64-bit calculated average pressure
     */
    double (*get_avg_pressure)(bmp280_t bmp, int n);

    /**
     * @brief [M] Calculate altitude 'n' times and get average value
     * 
     * @param bmp: Bmp280 object
     * @param p0: Relative pressure. It could be sea level pressure or average pressure obtained with get_avg_pressure method
     * @param n: Total samples
     * 
     * @return 64-bit calculated average altitude
     */
    double (*get_avg_altitude)(bmp280_t bmp, double p0, int n);
} bmp280_t;

#endif
