#include <bmp280.h>
#include <stdio.h>

#include <bmp280_hal_drivers/bmp280_hal_api.h>
#include <bmp280_registers.h>
#include <bmp280_data_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <string.h>
#include <math.h>


/* #################### CONSTANTS #################### */

static const char * bmp280_tag = "[BMP280]";

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
 * @brief [M] Initialize Bmp280 Class
 * 
 * @param bmp: Pointer to Bmp280 object
 * @param bmp_iface: bmp serial interface
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
static esp_err_t bmp280_Init(bmp280_t *bmp, dev_serial_iface_t *bmp_iface);


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
 * @param bmp_iface: bmp280 serial interface
 * 
 * @return
 *      - ESP_OK if success
 *      - ESP_FAIL
 */
static esp_err_t bmp280_Measure(dev_serial_iface_t bmp_iface);


/**
 * @brief Calculate altitude based on measured pressure 'p' and relative pressure 'p0'. The latter should be calculated with
 * 'bmp280_GetRelativeP' function or use sea level value, ~1013.25 hPa (value taken from https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf, p. 16, 3.6)
 * 
 * @param p: Measured pressure in hPa
 * @param p0: Relative pressure in hPA
 * 
 * @return 64-bit estimated altitude
 */
static double bmp280_GetAltitude(double p, double p0);


/**
 * @brief Calculate pressure 'n' times and get average value in hPA. Should be used as an alternative to sea level pressure
 * 
 * @param bmp: Bmp280 object
 * @param n: Total samples
 * 
 * @return 64-bit calculated average pressure
 */
static double bmp280_GetAvgPressure(bmp280_t bmp, int n);


/**
 * @brief Calculate altitude 'n' times and get average value in meters
 * 
 * @param bmp: Bmp280 object
 * @param p0: Relative pressure in hPa. It could be sea level pressure or average pressure obtained with get_avg_pressure method
 * @param n: Total samples
 * 
 * @return 64-bit calculated average altitude
 */
static double bmp280_GetAvgAltitude(bmp280_t bmp, double p0, int n);


/* #################### DEFINITIONS #################### */

void Bmp280(bmp280_t *bmp) {
    memset(bmp, 0, sizeof(bmp280_t));   /* Initialize all attributes to 0 */

    /* Function pointers assignment */
    bmp->init             = bmp280_Init;
    bmp->measure          = bmp280_Measure;
    bmp->get_temperature  = bmp280_GetTemp;
    bmp->get_pressure     = bmp280_GetPress;
    bmp->get_altitude     = bmp280_GetAltitude;
    bmp->get_avg_pressure = bmp280_GetAvgPressure;
    bmp->get_avg_altitude = bmp280_GetAvgAltitude;
}


static esp_err_t bmp280_Init(bmp280_t *bmp, dev_serial_iface_t *bmp_iface) {
    /* Copy serial interface parameter into's bmp serial_iface attribute */
    memcpy(&(bmp->serial_iface), bmp_iface, sizeof(dev_serial_iface_t));

    /* Check if any serial protocol was specified => If none then default value will be IFACE_NONE = 0 */
    if(bmp->serial_iface.type == IFACE_NONE) {
        ESP_LOGE(bmp280_tag, "%s in line %d: No serial protocol was specified", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
  
    /* Reset device in order to clean all registers */
    if(bmp280_hal_Reset(bmp->serial_iface) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Retrieve chip ID from registers */
    bmp280_hal_GetChipID(bmp->serial_iface, &(bmp->id));


    /**
     * Configure register <config> 0xF5
     * 
     * set t_sb bits (7, 6, 5) to 0b000 (default value), but in forced mode => hence it has no action
     * set filter bits (4, 3, 2) to 0b100
     * set spi3w_en to 0b0 (default value => disabled)
     */

    if(bmp280_hal_SetSerial(bmp->serial_iface, I2C) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetIIR(bmp->serial_iface, IIR_16) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetTsb(bmp->serial_iface, TS_MANUAL) != ESP_OK) {
        return ESP_FAIL;
    }

    /**
     * Configure register <ctrl_meas> 0xF4
     * 
     * set osrs_t bits (7, 6, 5) to 0b010 (x2 just to improve a little pressure measurements)
     * set osrs_p bits (4, 3, 2) to 0b101 (Ultra high resolution x16)
     * set mode bits (1, 0) to 0b01 (Forced mode) => @attention always set force mode again before taking a new measurement
     */

    if(bmp280_hal_SetPowerMode(bmp->serial_iface, FORCED_MODE) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsP(bmp->serial_iface, P_OS_X16) != ESP_OK) {
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsT(bmp->serial_iface, T_OS_X2) != ESP_OK) {
        return ESP_FAIL;
    }

    bmp280_hal_ReadCompWords(bmp->serial_iface, &comp_words);  /* Get compensation words stored in chip registers */

    bmp280_Measure(bmp->serial_iface);    /* Measure once to update registers and avoid reading wrong values */

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
static esp_err_t bmp280_Measure(dev_serial_iface_t bmp_iface) {
    /* Enable Forced mode */
    if(bmp280_hal_SetPowerMode(bmp_iface, FORCED_MODE) != ESP_OK) {
        return ESP_FAIL;
    }

    bmp280_hal_ReadRawTP(bmp_iface, &adc_t, &adc_p);

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


static double bmp280_GetAvgPressure(bmp280_t bmp, int n) {
    ESP_LOGI(bmp280_tag, "Computing average pressure. Estimated time: %.2f minutes", n / 1500.0f);

    double p0 = 0.0;
    for(int i = 0; i < n; i++) {
        bmp.measure(bmp.serial_iface);
        p0 += bmp.get_pressure();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    p0 /= n;

    ESP_LOGI(bmp280_tag, "Average pressure: %lf hPa", p0);

    return p0;
}

static double bmp280_GetAvgAltitude(bmp280_t bmp, double p0, int n) {
    ESP_LOGI(bmp280_tag, "Computing average altitude. Estimated time: %.2f minutes", n / 1500.0f);

    double z = 0.0;
    for(int i = 0; i < n; i++) {
        bmp.measure(bmp.serial_iface);
        z += bmp.get_altitude(bmp.get_pressure(), p0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    z /= n;

    ESP_LOGI(bmp280_tag, "Average altitude: %lf m", z);

    return z;
}

static double bmp280_GetAltitude(double p, double p0) {    
    return 44330.0 * (1 - pow(p / p0, 1 / 5.255));
}
