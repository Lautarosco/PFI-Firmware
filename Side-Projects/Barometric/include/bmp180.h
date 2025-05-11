#include <stdint.h>
#include "esp_system.h"
#include "registers.h"

typedef struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2;
    int16_t  MB, MC, MD;
} bmp180_calib_t;

typedef struct {
    // Calibration
    bmp180_calib_t calib;
    
    // I2C Config
    uint8_t i2c_addr;
    int sda_pin;
    int scl_pin;
    
    // Measurement settings
    uint8_t oss;  // Oversampling setting
} bmp180_t;

// Initialization
esp_err_t bmp180_init(bmp180_t* dev, uint8_t i2c_addr, int sda, int scl, uint8_t oss);

// Measurement functions
esp_err_t bmp180_read_temperature(bmp180_t* dev, float* temperature);
esp_err_t bmp180_read_pressure(bmp180_t* dev, int32_t* pressure);
esp_err_t bmp180_calculate_altitude(float* altitude, int32_t pressure, float sea_level_pa);