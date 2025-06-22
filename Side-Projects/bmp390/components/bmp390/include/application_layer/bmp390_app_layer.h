#ifndef BMP390_APP_LAYER_H
#define BMP390_APP_LAYER_H

#include <hardware_layer/bmp390_registers.h>
#include <interface.h>
#include <stdbool.h>

typedef struct bmp390_configs {
    bmp390_if_conf_reg_spi_t spi_mode;                      /* SPI mode (cannot select other interface) */
    bmp390_if_conf_reg_i2c_wdt_en_t i2c_wdt_en;             /* Enable I2C watchdog timeout (cannot select other interface) */
    bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout;         /* I2C watchdog timeout */
    bmp390_pwr_ctrl_mode_t pwr_mode;                        /* Power mode */
    bool press_en;                                          /* Enable pressure measurements */
    bool temp_en;                                           /* Enable temperature measurements */
    bmp390_osr_press_t osr_press;                           /* Pressure resolution */
    bmp390_osr_temp_t osr_temp;                             /* Temperature resolution */
    bmp390_odr_sel_t odr_sel;                               /* Sampling frequency */
    bmp390_config_coef_t iir_coef;                          /* Internal IIR filter coefficient */
} bmp390_configs_t;

typedef struct bmp390 bmp390_t;

typedef struct bmp390 {
    esp_err_t (*init)(bmp390_t *bmp, device_interface_t *dev_iface, bmp390_configs_t bmp_settings);

    device_interface_t iface;       /* Sensor interface */
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
