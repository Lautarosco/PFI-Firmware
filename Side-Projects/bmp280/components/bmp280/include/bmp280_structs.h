#ifndef BMP280_STRUCTS_H
#define BMP280_STRUCTS_H

#include <stdint.h>
#include <esp_err.h>

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
     * @brief Initialize Bmp280 Class
     * 
     * @param bmp: Pointer to Bmp280 object
     * @param addr: Sensor address
     * @param sda: SDA data pin
     * @param scl: SCL clock pin
     * 
     * @return ESP_OK if success - ESP_FAIL
     */
    esp_err_t (* init)(bmp280_t * bmp, int addr, int sda, int scl);
} bmp280_t;

#endif
