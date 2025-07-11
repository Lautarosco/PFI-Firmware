#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <hardware_layer/bmp390_hw_layer.h>
#include <application_layer/compensation/bmp390_compensation.h>

#include <string.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

typedef struct bmp390_sampling_times {
    bmp390_odr_sel_t odr_sel;           /* Selected sampling frequency Hz */
    unsigned int ts;                    /* Sampling time in ms based on selected odr */
} bmp390_sampling_times_t;

/* ========== Private variables ========== */

const char *bmp390_app_tag = "[BMP390_APP_LAYER]";      /* Application layer TAG */

static bmp390_calib_data_t calib_data;                  /* Compensation coefficients */

static bmp390_sampling_times_t ts_arr[] = {
    {.odr_sel = BMP390_ODR_SEL_200_HZ,      .ts = 5},
    {.odr_sel = BMP390_ODR_SEL_100_HZ,      .ts = 10},
    {.odr_sel = BMP390_ODR_SEL_50_HZ,       .ts = 20},
    {.odr_sel = BMP390_ODR_SEL_25_HZ,       .ts = 40},
    {.odr_sel = BMP390_ODR_SEL_12P5_HZ,     .ts = 80},
    {.odr_sel = BMP390_ODR_SEL_6P25_HZ,     .ts = 160},
    {.odr_sel = BMP390_ODR_SEL_3P1_HZ,      .ts = 320},
    {.odr_sel = BMP390_ODR_SEL_1P5_HZ,      .ts = 640},
    {.odr_sel = BMP390_ODR_SEL_0P78_HZ,     .ts = 1280},
    {.odr_sel = BMP390_ODR_SEL_0P39_HZ,     .ts = 2560},
    {.odr_sel = BMP390_ODR_SEL_0P2_HZ,      .ts = 5120},
    {.odr_sel = BMP390_ODR_SEL_0P1_HZ,      .ts = 10240},
    {.odr_sel = BMP390_ODR_SEL_0P05_HZ,     .ts = 20480},
    {.odr_sel = BMP390_ODR_SEL_0P02_HZ,     .ts = 40960},
    {.odr_sel = BMP390_ODR_SEL_0P01_HZ,     .ts = 81920},
    {.odr_sel = BMP390_ODR_SEL_0P006_HZ,    .ts = 163840},
    {.odr_sel = BMP390_ODR_SEL_0P003_HZ,    .ts = 327680},
    {.odr_sel = BMP390_ODR_SEL_0P0015_HZ,   .ts = 655360}
};

/* =========== Private functions =========== */

static esp_err_t bmp390_init(bmp390_t *bmp, bmp390_configs_t bmp_settings, bmp390_temp_units_t temp_unit, bmp390_press_units_t press_unit, unsigned int press0_samples) {
    /* 1. Copy temperature and pressure units to bmp's <temp_unit> and <press_unit> attributes */
    bmp->temp_unit       = temp_unit;
    bmp->press_unit      = press_unit;
    bmp->i2c_bmp_handler = *(bmp_settings.i2c_handler);

    /* 2. Reset device to default settings */
    if(bmp390_hwl_exec_cmd(bmp->i2c_bmp_handler, BMP390_CMD_SOFTRESET) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 3. Wait until device is successfully reseted */
    while(!bmp390_hwl_detect_soft_reset(bmp->i2c_bmp_handler));

    /* 4. Configure interface */
    if(bmp_settings.i2c_wdt_en) {
        if(bmp390_hwl_i2c_en_wdt(bmp->i2c_bmp_handler, bmp_settings.i2c_wdt_tout) != ESP_OK) {
            return ESP_FAIL;
        }
    }

    /* 5. Enable pressure sensor and set its resolution */
    if(bmp_settings.press_en) {
        if((bmp390_hwl_press_en(bmp->i2c_bmp_handler) != ESP_OK) || (bmp390_hwl_set_osr_press(bmp->i2c_bmp_handler, bmp_settings.osr_press) != ESP_OK)) {
            return ESP_FAIL;
        }
    }

    /* 6. Enable temperature sensor and set its resolution */
    if(bmp_settings.temp_en) {
        if((bmp390_hwl_temp_en(bmp->i2c_bmp_handler) != ESP_OK) || (bmp390_hwl_set_osr_temp(bmp->i2c_bmp_handler, bmp_settings.osr_temp) != ESP_OK)) {
            return ESP_FAIL;
        }
    }
    
    /* 7. Set BMP390 internal IIR filter coefficient */
    if(bmp390_hwl_set_iir_coef(bmp->i2c_bmp_handler, bmp_settings.iir_coef) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 8. Set sampling frequency in Hz */
    if(bmp390_hwl_set_odr(bmp->i2c_bmp_handler, bmp_settings.odr_sel) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 9. Set power mode */
    if(bmp390_hwl_set_pwr_mode(bmp->i2c_bmp_handler, bmp_settings.pwr_mode)) {
        return ESP_FAIL;
    }

    /* 10. Read and update compensation coefficients */
    if(bmp390_hwl_get_comp_coefs(bmp->i2c_bmp_handler, &calib_data) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 11. Compute relative pressure */
    unsigned int ts = 0;
    for (int i = 0; i < ((sizeof(ts_arr)) / (sizeof(ts_arr[0]))); i++) {
        if(ts_arr[i].odr_sel == bmp_settings.odr_sel) {
            ts = ts_arr[i].ts;
            break;
        }
    }
    if(!ts) {
        ESP_LOGE(bmp390_app_tag, "{Function <%s> in line %d}: Sampling frequency not found. Compute relative pressure --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    ESP_LOGI(bmp390_app_tag, "Computing relative pressure, do not move the sensor");
    bmp->get_relative_press(bmp, press0_samples, ts);
    ESP_LOGI(bmp390_app_tag, "Relative pressure: %lf hPa", bmp->press0);

    ESP_LOGI(bmp390_app_tag, "Initialize Bmp390 object --> OK");

    return ESP_OK;
}

static void bmp390_get_mode_value(bmp390_t *bmp, uint8_t reg_addr, uint8_t mode, char *msg, size_t msg_length) {
    bmp390_hwl_get_mode_val(bmp->i2c_bmp_handler, reg_addr, mode, msg, msg_length);
}

static esp_err_t bmp390_measure(bmp390_t *bmp) {
    uint32_t adc_press = 0;
    uint32_t adc_temp  = 0;

    if(bmp390_hwl_read_raw_data(bmp->i2c_bmp_handler, &adc_temp, &adc_press) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Compensate temperature */
    double temp = 0.0;
    switch (bmp->temp_unit) {
        case C:
            temp = bmp390_compensate_temp_celsius(adc_temp, calib_data);
            break;
        case K:
            break;
        default:
            ESP_LOGW(bmp390_app_tag, "{Function <%s> in line %d}: Temperature unit not found", __func__, __LINE__);
            break;
    }

    /* If temperature sensor is enabled, then update bmp's temperature attribute */
    char msg[256];
    if(bmp390_hwl_get_mode_val(bmp->i2c_bmp_handler, BMP390_PWR_CTRL_RW_REG, BMP390_PWR_CTRL_TEMP_EN, msg, sizeof(msg)) == BMP390_PWR_CTRL_TEMP_ON) {
        bmp->temp = temp;
    }

    /* Compensate temperature */
    switch (bmp->press_unit) {
        case PA:
            bmp->press = bmp390_compensate_press_pascal(adc_press, temp, calib_data);
            break;
        case HPA:
            bmp->press = bmp390_compensate_press_hectopascal(adc_press, temp, calib_data);
            break;
        default:
            ESP_LOGW(bmp390_app_tag, "{Function <%s> in line %d}: Pressure unit not found", __func__, __LINE__);
            break;
    }

    return ESP_OK;
}

/**
 * @brief Measure pressure and temperature <n_samples> time and compute relative pressure
 * 
 * @param bmp: Pointer to bmp390_t struct
 * @param n_samples: Total samples to be taken
 * @param t_ms: Delay between samples in milliseconds (ms)
 * 
 * @retval
 *      - Relative pressure if success
 *      - (-1) If total samples is less or equal to 0
 */
static esp_err_t bmp390_get_relative_press(bmp390_t *bmp, unsigned int n_samples, unsigned int t_ms) {
    /* Check if total samples is a valid number */
    if((n_samples <= 0) || (t_ms <= 0)) {
        ESP_LOGE(bmp390_app_tag, "{Function <%s> in line %d}: Total samples <n_samples> nor sampling time <t_ms> must be greater than 0. Compute relative pressure --> FAILED", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    for(int i = 0; i < 3; i++) {
        /* Update measurements */
        bmp->measure(bmp);
        printf("P%d: %lf hPa\n", i, bmp->press);

        vTaskDelay(pdMS_TO_TICKS(t_ms * 5));
    }

    double sum = 0.0;
    for(int i = 0; i < n_samples; i++) {
        /* Update measurements */
        bmp->measure(bmp);
        sum += bmp->press;

        vTaskDelay(pdMS_TO_TICKS(t_ms));
    }
    bmp->press0 = sum / ((double) n_samples);

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
    bmp->get_relative_press   = bmp390_get_relative_press;
}
