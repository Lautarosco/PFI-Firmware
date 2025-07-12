#ifndef BMP390_APP_LAYER_H
#define BMP390_APP_LAYER_H

#define BMP390_ADDR 0x76

#include <hardware_layer/bmp390_registers.h>
#include <driver/i2c_master.h>
#include <stdbool.h>


/* =========== Public structs =========== */

typedef enum bmp390_press_units {
    PA,         /* Compensate raw pressure measurements and return new value in Pa */
    HPA         /* Compensate raw pressure measurements and return new value in hPa */
} bmp390_press_units_t;

typedef enum bmp390_temp_units {
    C,          /* Compensate raw temperature measurements and return new value in °C (Celsius) */
    K           /* Compensate raw temperature measurements and return new value in K (Kelvin) */
} bmp390_temp_units_t;

typedef struct bmp390_configs {
    bmp390_if_conf_reg_spi_t spi_mode;                      /* SPI mode (cannot select other interface) */
    bmp390_if_conf_reg_i2c_wdt_en_t i2c_wdt_en;             /* Enable I2C watchdog timeout (cannot select other interface) */
    bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout;         /* I2C watchdog timeout */
    i2c_master_dev_handle_t *i2c_handler;                   /* Pointer to BMP390 I2C bus handler */
    bmp390_pwr_ctrl_mode_t pwr_mode;                        /* Power mode */
    bool press_en;                                          /* Enable pressure measurements */
    bool temp_en;                                           /* Enable temperature measurements */
    bmp390_osr_press_t osr_press;                           /* Pressure resolution */
    bmp390_osr_temp_t osr_temp;                             /* Temperature resolution */
    bmp390_odr_sel_t odr_sel;                               /* Sampling frequency */
    bmp390_config_coef_t iir_coef;                          /* Internal IIR filter coefficient */
} bmp390_configs_t;

typedef struct bmp390 bmp390_t;

/* =========== Public functions =========== */

typedef struct bmp390 {
    /**
     * @brief Initialize Bmp390 object with given operation modes and selected interface
     * 
     * @param bmp: Pointer to bmp390_t struct
     * @param bmp_settings: Registers operation modes
     * @param temp_unit: Temperature measurements unit
     * @param press_unit: Pressure measurements unit
     * @param press0_samples: Total samples to compute relative pressure
     * 
     * @retval
     *      - ESP_OK: success
     *      - ESP_FAIL
     */
    esp_err_t (*init)(bmp390_t *bmp, bmp390_configs_t bmp_settings, bmp390_temp_units_t temp_unit, bmp390_press_units_t press_unit, unsigned int press0_samples);

    /**
     * @brief Read raw pressure and temperature data and compensate them to obtain actual values
     * 
     * @param bmp: Pointer to bmp390_t struct
     * 
     * @retval
     *      - ESP_OK
     */
    esp_err_t (*measure)(bmp390_t *bmp);

    /**
     * @brief Read any mode of a given register and return its actual value
     * 
     * @note Buffer is cleared before its used
     * 
     * @param bmp: Pointer to bmp390_t struct
     * @param reg_addr: Register address
     * @param mode: Operation mode
     * @param msg: Buffer to store answer
     * @param msg_length: Length of buffer
     * 
     * @retval
     *      - Mode value
     *      - (-1) If mode was not found
     */
    void (*check_reg_mode_value)(bmp390_t *bmp, uint8_t reg_addr, uint8_t mode, char *msg, size_t msg_length);

    /**
     * @brief Measure pressure and temperature <n_samples> time and compute relative pressure
     * 
     * @param bmp: Pointer to bmp390_t struct
     * @param n_samples: Total samples to be taken
     * @param t_ms: Delay between samples in milliseconds (ms)
     * 
     * @retval
     *      - ESP_OK
     *      - ESP_ERR_INVALID_ARG
     */
    esp_err_t (*get_relative_press)(bmp390_t *bmp, unsigned int n_samples, unsigned int t_ms);

    i2c_master_dev_handle_t i2c_bmp_handler;        /* I2C bus BMP390 handler */
    double press;                                   /* Last pressure measurement */
    double press0;                                  /* Relative pressure */
    double temp;                                    /* Last temperature measurement */
    bmp390_temp_units_t temp_unit;                  /* Temperature measurements unit */
    bmp390_press_units_t press_unit;                /* Pressure measurements unit */
} bmp390_t;

/**
 * @brief Make an instance of Bmp390 Class
 * 
 * @param bmp: Pointer to bmp390_t variable
 * 
 * @retval none
 */
void Bmp390(bmp390_t *bmp);

#endif
