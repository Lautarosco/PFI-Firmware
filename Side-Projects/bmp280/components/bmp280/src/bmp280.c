#include <bmp280.h>
#include <stdio.h>

#include <bmp280_hal_drivers/bmp280_hal_api.h>
#include <com.h>
#include <bmp280_registers.h>
#include <bmp280_data_types.h>

#include <esp_log.h>
#include <string.h>
#include <math.h>


/* #################### CONSTANTS #################### */

static const char * bmp280_tag = "BMP280";

static comp_words_t comp_words; /* Compensation words used for calibration */

static bmp280_s32_t adc_t = 0;  /* 20-bit raw temperature */
static bmp280_s32_t adc_p = 0;  /* 20-bit raw pressure */
static bmp280_s32_t t_fine = 0; /* Used to calculate actual pressure */


/* #################### INLINES #################### */

/**
 * @brief Computes base^(n - 1)
 * 
 * @param base: Base
 * @param n: Exponent
 * 
 * @return float
 */
static inline float pow_float(int base, int n) {return (float) (base << (n - 1));}


/* #################### PROTOTYPES #################### */

/**
 * @brief Initialize an Bmp280 object
 * 
 * @param bmp: Pointer to Bmp280 object
 * @param master_i2c_bus_handler: master (MCU) I2C bus handler
 * @param addr: Address of bmp280 sensor
 * @param sda: I2C SDA data
 * @param scl: I2C SCL clock
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
static esp_err_t bmp280_Init(bmp280_t * bmp, i2c_master_bus_handle_t * master_i2c_bus_handler, int addr, int sda, int scl);


/**
 * @brief Compensate raw temperature values stored in registers and return
 * actual temperature in degrees (°C) with a resolution of .01
 * 
 * @return 32-bit signed int measured temperature
 */
static double bmp280_GetTemp(void);


/**
 * @brief Compensate raw pressure values stored in registers and return
 * actual pressure in hecto-Pascals (hPa)
 * 
 * @return 32-bit signed int measured pressure
 */
static double bmp280_GetPress(void);


/**
 * @brief Get raw pressure and temperature data
 * 
 * @param bmp280_i2c_bus_handler: Bmp280 I2C bus handler
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
static esp_err_t bmp280_Measure(i2c_master_dev_handle_t bmp280_i2c_bus_handler);


/**
 * @brief Calculate altitude based on measured pressure 'p' and relative pressure 'p0'. The latter should be calculated with
 * 'bmp280_GetRelativeP' function or use sea level value, ~1013.25 hPa (value taken from https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf, p. 16, 3.6)
 * 
 * @param p: Measured pressure
 * @param p0: Relative pressure
 * 
 * @return 64-bit estimated altitude
 */
static double bmp280_GetAltitude(double p, double p0);


/* #################### DEFINITIONS #################### */

bmp280_t * Bmp280(void) {
    bmp280_t * bmp = (bmp280_t *) malloc(sizeof(bmp280_t));

    memset(bmp, 0, sizeof(bmp280_t));   /* Initialize all attributes to 0 */

    /* Function pointers assignment */
    bmp->init         = bmp280_Init;
    bmp->get_temp     = bmp280_GetTemp;
    bmp->get_press    = bmp280_GetPress;
    bmp->measure      = bmp280_Measure;
    bmp->get_altitude = bmp280_GetAltitude;

    return bmp;
}


static esp_err_t bmp280_Init(bmp280_t * bmp, i2c_master_bus_handle_t * master_i2c_bus_handler, int addr, int sda, int scl) {
    bmp->addr = addr;
    bmp->i2c.sda = sda;
    bmp->i2c.scl = scl;

    bmp280_hal_i2cInit(master_i2c_bus_handler, &(bmp->i2c.bmp280_i2c_bus_handler), bmp->addr, bmp->i2c.sda, bmp->i2c.scl);
  
    /* Reset device in order to clean all registers */
    if(bmp280_hal_Reset(bmp->i2c.bmp280_i2c_bus_handler) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Retrieve chip ID from registers */
    bmp280_hal_GetChipID(bmp);


    /**
     * Configure register <config> 0xF5
     * 
     * set t_sb bits (7, 6, 5) to 0b000 (default value), but in forced mode => hence it has no action
     * set filter bits (4, 3, 2) to 0b100
     * set spi3w_en to 0b0 (default value => disabled)
     */

    if(bmp280_hal_SetSerial(bmp->i2c.bmp280_i2c_bus_handler, I2C) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetIIR(bmp->i2c.bmp280_i2c_bus_handler, IIR_16) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetTsb(bmp->i2c.bmp280_i2c_bus_handler, TS_MANUAL) != ESP_OK) {
        return ESP_FAIL;
    }

    /**
     * Configure register <ctrl_meas> 0xF4
     * 
     * set osrs_t bits (7, 6, 5) to 0b010 (x2 just to improve a little pressure measurements)
     * set osrs_p bits (4, 3, 2) to 0b101 (Ultra high resolution x16)
     * set mode bits (1, 0) to 0b01 (Forced mode) => @attention always set force mode again before taking a new measurement
     */

    if(bmp280_hal_SetPowerMode(bmp->i2c.bmp280_i2c_bus_handler, FORCED_MODE) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsP(bmp->i2c.bmp280_i2c_bus_handler, P_OS_X16) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsT(bmp->i2c.bmp280_i2c_bus_handler, T_OS_X2) != ESP_OK) {
        return ESP_FAIL;
    }

    bmp280_hal_ReadCompWords(&comp_words, bmp->i2c.bmp280_i2c_bus_handler);  /* Get compensation words stored in chip registers */

    bmp280_Measure(bmp->i2c.bmp280_i2c_bus_handler);    /* Measure once to update registers and avoid reading wrong values */

    ESP_LOGI(bmp280_tag, "Initialize Bmp280 object --> OK");

    return ESP_OK;
}


/**
 * @brief Measure raw pressure and temperature
 * 
 * @param bmp: Bmp280 object
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
static esp_err_t bmp280_Measure(i2c_master_dev_handle_t bmp280_i2c_bus_handler) {

    /* Enable Forced mode */
    if(bmp280_hal_SetPowerMode(bmp280_i2c_bus_handler, FORCED_MODE) != ESP_OK) {
        return ESP_FAIL;
    }

    bmp280_hal_ReadRawTP(bmp280_i2c_bus_handler, &adc_t, &adc_p);

    return ESP_OK;
}


static double bmp280_GetTemp(void) {
    double var1, var2;

    var1 = ((((double) adc_t) / pow_float(2, 14)) - (((double) comp_words.dig_T1) / pow_float(2, 10))) * ((double) comp_words.dig_T2);
    var2 = (((((double) adc_t) / pow_float(2, 17)) - (((double) comp_words.dig_T1) / pow_float(2, 13))) * (((double) adc_t / pow_float(2, 17)) - (((double) comp_words.dig_T1) / pow_float(2, 13)))) * ((double) comp_words.dig_T3);

    t_fine = (bmp280_s32_t) (var1 + var2);

    return (var1 + var2) / (pow_float(2, 9) * 10);
}


static double bmp280_GetPress(void) {
    /**
     * Pressure compensation uses 't_fine' variable which gets updated by running
     * 'bmp280_GetTemp' function. The latter may not be running, therefore we must ensure it does
     */

    bmp280_GetTemp();
/*
    double var1, var2, p;

    var1 = ((double) t_fine / pow_float(2, 1)) - 64000.0f;
    var2 = ((var1 * var1) * ((double) comp_words.dig_P6)) / pow_float(2, 15);
    var2 = var2 + (var1 * ((double) comp_words.dig_P5) * pow_float(2, 1));
    var2 = (var2 / pow_float(2, 2)) + (((double) comp_words.dig_P4) * pow_float(2, 16));
    var1 = ((((double) comp_words.dig_P3) * var1 * var1) / pow_float(2, 19)) + ((((double) comp_words.dig_P2) * var1) / pow_float(2, 19));
    var1 = (1.0f + (var1 / pow_float(2, 15))) * ((double) comp_words.dig_P1);
    p = pow_float(2, 20) - ((double) adc_p);
    p = ((p - (var2 / pow_float(2, 12))) * 6250.0f) / var1;
    var1 = (((double) comp_words.dig_P9) * p * p) / pow_float(2, 31);
    var2 = (p * ((double) comp_words.dig_P8)) / pow_float(2, 15);
    p += var1 + var2 + (((double) comp_words.dig_P7) / pow_float(2, 4));

    return p;
*/

    bmp280_s64_t var1, var2, p;

    var1 = ((bmp280_s64_t) t_fine) - 128000;
    var2 = var1 * var1 * (bmp280_s64_t) comp_words.dig_P6;
    var2 = var2 + ((var1 * (bmp280_s64_t) comp_words.dig_P5) << 17);
    var2 = var2 + (((bmp280_s64_t) comp_words.dig_P4) << 35);
    var1 = ((var1 * var1 * (bmp280_s64_t) comp_words.dig_P3) >> 8) + ((var1 * (bmp280_s64_t) comp_words.dig_P2) << 12);
    var1 = (((bmp280_s64_t) 1 << 47) + var1) * ((bmp280_s64_t) comp_words.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((bmp280_s64_t) comp_words.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((bmp280_s64_t) comp_words.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((bmp280_s64_t) comp_words.dig_P7) << 4);

    return (p / 256.0) / 100.0;

}


double bmp280_GetRelativeP(bmp280_t bmp, int n) {
    return bmp280_hal_GetRelativeP(bmp, n);
}


static double bmp280_GetAltitude(double p, double p0) {    
    return 44330.0 * (1 - pow(p / p0, 1 / 5.255));
}
