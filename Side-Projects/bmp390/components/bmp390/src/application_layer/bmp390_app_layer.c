#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <hardware_layer/bmp390_hw_layer.h>

#include <string.h>
#include <esp_log.h>

/* ========== Private variables ========== */

const char *bmp390_app_tag = "[BMP390_APP_LAYER]";      /* Application layer TAG */

static bmp390_calib_data_t calib_data;

/* =========== Private functions =========== */

static esp_err_t bmp390_init(bmp390_t *bmp, device_interface_t *dev_iface, bmp390_configs_t bmp_settings) {
    /* 1. Copy <dev_iface> parameter into bmp's <iface> attribute */
    memcpy(&(bmp->iface), dev_iface, sizeof(device_interface_t));

    /* 2. Reset device to default settings */
    if(bmp390_hwl_exec_cmd(*dev_iface, BMP390_CMD_SOFTRESET) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 3. Wait until device is successfully reseted */
    while(!bmp390_hwl_detect_soft_reset(*dev_iface));

    /* 4. Configure interface */
    if(bmp_settings.i2c_wdt_en) {
        if(bmp390_hwl_i2c_en_wdt(*dev_iface, bmp_settings.i2c_wdt_tout) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    /* 5. Enable pressure sensor and set its resolution */
    if(bmp_settings.press_en) {
        if((bmp390_hwl_press_en(*dev_iface) != ESP_OK) || (bmp390_hwl_set_osr_press(*dev_iface, bmp_settings.osr_press) != ESP_OK)) {
            return ESP_FAIL;
        }
    }

    /* 6. Enable temperature sensor and set its resolution */
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

    /* 9. Set power mode */
    if(bmp390_hwl_set_pwr_mode(*dev_iface, bmp_settings.pwr_mode)) {
        return ESP_FAIL;
    }

    /* 10. Read and update compensation coefficients */
    if(bmp390_hwl_get_comp_coefs(*dev_iface, &calib_data) != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_app_tag, "Initialize Bmp390 object --> OK");

    return ESP_OK;
}

static void bmp390_get_mode_value(bmp390_t *bmp, uint8_t reg_addr, uint8_t mode) {
    bmp390_hwl_get_mode_val(bmp->iface, reg_addr, mode);
}

static double bmp390_compensate_temp(uint32_t adc_temp) {
    double partial_data1 = 0.0f;
    double partial_data2 = 0.0f;

    partial_data1 = (double) adc_temp - calib_data.par_t1;
    partial_data2 = partial_data1 * calib_data.par_t2;

    return partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;
}

static esp_err_t bmp390_measure(bmp390_t *bmp) {
    uint32_t adc_press = 0;
    uint32_t adc_temp  = 0;

    if(bmp390_hwl_read_raw_data(bmp->iface, &adc_temp, &adc_press) != ESP_OK) {
        return ESP_FAIL;
    }

    double temp = bmp390_compensate_temp(adc_temp);

    ESP_LOGW(bmp390_app_tag, "Temperature: %f", temp);

    return ESP_OK;
}

/* =========== Public functions =========== */

void Bmp390(bmp390_t *bmp) {
    /* 1. Initialize all attributes to 0 */
    memset(bmp, 0, sizeof(bmp390_t));

    /* 2. Assign pointer to functions (methods of the Bmp390 Class) */
    bmp->init                 = bmp390_init;
    bmp->measure              = bmp390_measure;
    bmp->check_reg_mode_value = bmp390_get_mode_value;
}
