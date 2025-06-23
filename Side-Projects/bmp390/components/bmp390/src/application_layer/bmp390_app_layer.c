#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <hardware_layer/bmp390_hw_layer.h>

#include <string.h>
#include <esp_log.h>

const char *bmp390_hal_tag = "[BMP390_APP_LAYER]";

/* =========== Private functions =========== */

static esp_err_t bmp390_init(bmp390_t *bmp, device_interface_t *dev_iface, bmp390_configs_t bmp_settings) {
    /* 1. Reset device to default settings */
    if(bmp390_hwl_exec_cmd(*dev_iface, BMP390_CMD_SOFTRESET) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 2. Wait until device is successfully reseted */
    while(!bmp390_hwl_detect_soft_reset(*dev_iface));

    /* 3. Configure interface */
    if(bmp_settings.i2c_wdt_en) {
        if(bmp390_hwl_i2c_en_wdt(*dev_iface, bmp_settings.i2c_wdt_tout) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    /* 4. Set power mode */
    if(bmp390_hwl_set_pwr_mode(*dev_iface, bmp_settings.pwr_mode)) {
        return ESP_FAIL;
    }

    /* . Enable pressure sensor and set its resolution */
    if(bmp_settings.press_en) {
        if((bmp390_hwl_press_en(*dev_iface) != ESP_OK) || (bmp390_hwl_set_osr_press(*dev_iface, bmp_settings.osr_press) != ESP_OK)) {
            return ESP_FAIL;
        }
    }

    /* . Enable pressure sensor and set its resolution */
    if(bmp_settings.temp_en) {
        if((bmp390_hwl_temp_en(*dev_iface) != ESP_OK) || (bmp390_hwl_set_osr_temp(*dev_iface, bmp_settings.osr_temp) != ESP_OK)) {
            return ESP_FAIL;
        }
    }

    /* 7. Set BMP390 internal IIR filter coefficient */
    if(bmp390_hwl_set_iir_coef(*dev_iface, bmp_settings.iir_coef) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 8. Set sampling frequency in Hz */
    if(bmp390_hwl_set_odr(*dev_iface, bmp_settings.odr_sel) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_hal_tag, "Initialize Bmp390 object --> OK");

    // bmp390_hwl_get_mode_value(*dev_iface, "I2C_WDT", BMP390_IF_CONF_RW_REG, 1, BMP390_IF_CONF_I2C_WDT_SEL_BIT);
    // bmp390_hwl_get_mode_value(*dev_iface, "POWER_MODE", BMP390_PWR_CTRL_RW_REG, 2, BMP390_PWR_CTRL_MODE_BITS);
    // bmp390_hwl_get_mode_value(*dev_iface, "PRESS_EN", BMP390_PWR_CTRL_RW_REG, 1, BMP390_PWR_CTRL_PRESS_EN_BIT);
    // bmp390_hwl_get_mode_value(*dev_iface, "PRESS_RES", BMP390_OSR_RW_REG, 3, BMP390_OSR_P_BITS);
    // bmp390_hwl_get_mode_value(*dev_iface, "TEMP_EN", BMP390_PWR_CTRL_RW_REG, 1, BMP390_PWR_CTRL_TEMP_EN_BIT);
    // bmp390_hwl_get_mode_value(*dev_iface, "TEMP_RES", BMP390_OSR_RW_REG, 3, BMP390_OSR_T_BITS);
    // bmp390_hwl_get_mode_value(*dev_iface, "IIR", BMP390_CONFIG_RW_REG, 3, BMP390_CONFIG_IIR_BITS);
    // bmp390_hwl_get_mode_value(*dev_iface, "ODR", BMP390_ODR_RW_REG, 5, BMP390_ODR_ODR_SEL_BITS);

    return ESP_OK;
}

/* =========== Public functions =========== */

void Bmp390(bmp390_t *bmp) {
    /* 1. Initialize all attributes to 0 */
    memset(bmp, 0, sizeof(bmp390_t));

    /* 2. Assign pointer to functions (methods of the Bmp390 Class) */
    bmp->init = bmp390_init;
}
