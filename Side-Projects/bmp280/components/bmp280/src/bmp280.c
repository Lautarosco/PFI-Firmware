#include <bmp280.h>
#include <stdio.h>

#include <bmp280_ll_drivers/communications/com.h>
#include <bmp280_hal_drivers/bmp280_hal_api.h>
#include <registers_map/bmp280_registers.h>

#include <esp_log.h>
#include <string.h>


/* #################### CONSTANTS #################### */

static const char * bmp280_tag = "BMP280";

static compensation_words_t comp_words;
static bmp280_s32_t t_fine = 0;

static bmp280_u16_t press_msb;  /* MSB raw pressure */
static bmp280_u16_t press_lsb;  /* LSB raw pressure */
static bmp280_u16_t press_xlsb; /* XLSB raw pressure */

static bmp280_u16_t temp_msb;   /* MSB raw temperature */
static bmp280_u16_t temp_lsb;   /* LSB raw temperature */
static bmp280_u16_t temp_xlsb;  /* XLSB raw temperature */


/* #################### INLINES #################### */

/**
 * @brief Computes base^(n - 1)
 * 
 * @param base: Base of power
 * @param n: Exponent of power
 * 
 * @return float
 */
static inline float pow_float(int base, int n) {return (float) (base << (n - 1));}


/* #################### PROTOTYPES #################### */

/**
 * @brief Initialize an Bmp280 object
 * 
 * @param bmp: Pointer to Bmp280 object
 * @param addr: Address of bmp280 sensor
 * @param sda: I2C SDA data
 * @param scl: I2C SCL clock
 * 
 * @return none
 */
static esp_err_t bmp280_Init(bmp280_t * bmp, int addr, int sda, int scl);


/**
 * @brief Compensate raw temperature values stored in registers and return temperature
 * in degrees (°C) with a resolution of .01
 * 
 * @param bmp: Bmp280 object
 * @param adc_t: Raw data stored in registers
 * @param comp_words: Compensation words
 * 
 * @return 32-bit signed int measured temperature
 */
static bmp280_s32_t bmp280_MeasureTemp(bmp280_t * bmp, bmp280_s32_t adc_t, compensation_words_t comp_words);


/**
 * @brief Measure pressure and temperature
 * 
 * @param bmp: Bmp280 object
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
static esp_err_t bmp280_Measure(bmp280_t * bmp);


/* #################### DEFINITIONS #################### */

bmp280_t * Bmp280(void) {
    bmp280_t * bmp = (bmp280_t *) malloc(sizeof(bmp280_t));

    memset(bmp, 0, sizeof(bmp280_t));   /* Initialize all attributes to 0 */

    /* Function pointers assignment */
    bmp->init     = bmp280_Init;
    bmp->get_temp = bmp280_MeasureTemp;
    bmp->measure  = bmp280_Measure;

    return bmp;
}


static esp_err_t bmp280_Init(bmp280_t * bmp, int addr, int sda, int scl) {
    ESP_LOGI(bmp280_tag, "Initializing Bmp280 object...");

    bmp->addr = addr;
    bmp->i2c.sda = sda;
    bmp->i2c.scl = scl;

    /* Initialize I2C interface */
    if(i2c_init(bmp->i2c.sda, bmp->i2c.scl) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to initialized I2C interface");
        return ESP_FAIL;
    }

    sensors_addr_t sensors[] = {
        {
            .name = "bmp280",
            .addr = bmp->addr,
            .__found = false
        }
    };

    /* Ensure bmp sensor is connected to I2C bus */
    i2c_scan(sensors, sizeof(sensors) / (sizeof(sensors[0])));
    for (int i = 0; i < ((sizeof(sensors)) / (sizeof(sensors[0]))); i++) {
        if(!sensors[i].__found) {
            ESP_LOGE(bmp280_tag, "%s in line %d --> ValueError: %s not found in I2C bus", __func__, __LINE__, sensors[i].name);
        }
    }
  
    /* Reset device in order to clean all registers */
    if(bmp280_hal_Reset(*bmp) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Retrieve chip ID */
    if(i2c_read_byte(bmp->addr, BMP280_ID_REG, &(bmp->id), 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to read bytes from <0x%X>", BMP280_ID_REG);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp280_tag, "Chip id is <0x%X>", bmp->id);


    /**
     * Configure register <config> 0xF5
     * 
     * set t_sb bits (7, 6, 5) to 0b000 (default value), but in forced mode => hence it has no action
     * set filter bits (4, 3, 2) to 0b100
     * set spi3w_en to 0b0 (default value => disabled)
     */

    if(bmp280_hal_SetSerial(*bmp, I2C ) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure serial interface", __func__, __LINE__);
        return ESP_FAIL;
    }
    if(bmp280_hal_SetIIR(*bmp, IIR_16) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure IIR filter", __func__, __LINE__);
        return ESP_FAIL;
    }
    if(bmp280_hal_SetTsb(*bmp, TS_MANUAL) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure standby time", __func__, __LINE__);
        return ESP_FAIL;
    }

    /**
     * Configure register <ctrl_meas> 0xF4
     * 
     * set osrs_t bits (7, 6, 5) to 0b010 (x2 just to improve a little pressure measurements)
     * set osrs_p bits (4, 3, 2) to 0b101 (Ultra high resolution x16)
     * set mode bits (1, 0) to 0b01 (Forced mode) => @attention always set force mode again before taking a new measurement
     */

    if(bmp280_hal_SetPowerMode(*bmp, FORCED_MODE ) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure pwoer mode", __func__, __LINE__);
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsP(*bmp, P_OS_X16) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure pressure oversampling", __func__, __LINE__);
        return ESP_FAIL;
    }
    if(bmp280_hal_SetOsT(*bmp, T_OS_X2) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure temperature oversampling", __func__, __LINE__);
        return ESP_FAIL;
    }



    /* Get compensation paremeters */

    if(bmp280_InitCompWords(bmp->addr, &comp_words) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to retrieve compensation parameters from chip");
        return ESP_FAIL;
    }

    ESP_LOGI(bmp280_tag, "dig_T1: %u", comp_words.dig_T1);
    ESP_LOGI(bmp280_tag, "dig_T2: %d", comp_words.dig_T2);
    ESP_LOGI(bmp280_tag, "dig_T3: %d", comp_words.dig_T3);
    ESP_LOGI(bmp280_tag, "dig_P1: %u", comp_words.dig_P1);
    ESP_LOGI(bmp280_tag, "dig_P2: %d", comp_words.dig_P2);
    ESP_LOGI(bmp280_tag, "dig_P3: %d", comp_words.dig_P3);
    ESP_LOGI(bmp280_tag, "dig_P4: %d", comp_words.dig_P4);
    ESP_LOGI(bmp280_tag, "dig_P5: %d", comp_words.dig_P5);
    ESP_LOGI(bmp280_tag, "dig_P6: %d", comp_words.dig_P6);
    ESP_LOGI(bmp280_tag, "dig_P7: %d", comp_words.dig_P7);
    ESP_LOGI(bmp280_tag, "dig_P8: %d", comp_words.dig_P8);
    ESP_LOGI(bmp280_tag, "dig_P9: %d", comp_words.dig_P9);


    ESP_LOGI(bmp280_tag, "Bmp280 object successfully initialized.");

    return ESP_OK;
}


/**
 * @brief Measure pressure and temperature
 * 
 * @param bmp: Bmp280 object
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
static esp_err_t bmp280_Measure(bmp280_t * bmp) {
    /* Enable Forced mode */
    if(bmp280_hal_SetPowerMode(*bmp, FORCED_MODE ) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to configure pwoer mode", __func__, __LINE__);
        return ESP_FAIL;
    }

    bmp280_s32_t adc_t;
    bmp280_hal_ReadRawTP(*bmp, &adc_t);

    return ESP_OK;
}


static bmp280_s32_t bmp280_MeasureTemp(bmp280_t * bmp, bmp280_s32_t adc_t, compensation_words_t comp_words) {
    bmp280_s32_t var1, var2;

    var1 = ((((double) adc_t) / pow_float(2, 14)) - (((double) comp_words.dig_T1) / pow_float(2, 10))) * ((double) comp_words.dig_T2);
    var2 = (((((double) adc_t) / pow_float(2, 17)) - (((double) comp_words.dig_T1) / pow_float(2, 13))) * (((double) adc_t / pow_float(2, 17)) - (((double) comp_words.dig_T1) / pow_float(2, 13)))) * ((double) comp_words.dig_T3);

    t_fine = (bmp280_s32_t) (var1 + var2);

    return (var1 + var2) / (pow_float(2, 9) * 10);
}
