#include <application_layer/lsm6dso_app_layer.h>
#include <esp_log.h>
#include <string.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LSM6DSO_TEMP0_SAMPLES           100                 /* Total samples used to calculate initial value */
#define LSM6DSO_TEMP_TYPICAL_FS         52                  /* Typical refresh rate of temperature sensor */

static const char *lsm6dso_app_layer_tag = "[LSM6DSO_APP_LAYER]";

bool lsm6dso_check_params(lsm6dso_params_t lsm_params) {
    if((lsm_params.acc.fs < LSM6DSO_ACC_FS_2G) || (lsm_params.acc.fs > LSM6DSO_ACC_FS_8G)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid accelerometer Full-scale selection",
            __func__, __LINE__
        );

        return false;
    }

    if((lsm_params.acc.odr < LSM6DSO_ACC_ODR_POWER_DOWN) || (lsm_params.acc.odr > LSM6DSO_ACC_ODR_1P6_HZ_HM_MODE_1)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid accelerometer ODR selection",
            __func__, __LINE__
        );
    }

    if((lsm_params.gyro.odr < LSM6DSO_ODR_GYRO_POWER_DOWN) || (lsm_params.gyro.odr > LSM6DSO_ODR_GYRO_6660_HZ)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid gyroscope ODR selection",
            __func__, __LINE__
        );
    }

    if((lsm_params.gyro.fs < LSM6DSO_FS_GYRO_250_DPS) || (lsm_params.gyro.fs > LSM6DSO_FS_GYRO_125_DPS)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid gyroscope Full-scale selection",
            __func__, __LINE__
        );

        return false;
    }

    if((lsm_params.gyro.lpf1_mode < LSM6DSO_GYRO_LPF1_OFF) || (lsm_params.gyro.lpf1_mode > LSM6DSO_GYRO_LPF1_7)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid gyroscope LPF1 mode",
            __func__, __LINE__
        );

        return false;
    }

    if((lsm_params.gyro.hpf_mode < LSM6DSO_GYRO_HPF_OFF) || (lsm_params.gyro.hpf_mode > LSM6DSO_GYRO_HPF_1P04_Hz)) {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Invalid gyroscope HPF mode",
            __func__, __LINE__
        );

        return false;
    }

    return true;
}

esp_err_t lsm6dso_init(lsm6dso_t *lsm, lsm6dso_params_t lsm_params) {
    if(!lsm6dso_check_params(lsm_params)) {
        return ESP_FAIL;
    }

    lsm->i2c_lsm_handler = *(lsm_params.i2c_lsm_handler);

    char msg[256];
    
    /* Reset sensor */
    esp_err_t ret = lsm6dso_hwl_sw_reset(lsm->i2c_lsm_handler, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "%s", msg);
        return ESP_FAIL;
    }

    /* Get sensor ID */
    uint8_t id;
    ret = lsm6dso_hwl_get_chip_id(lsm->i2c_lsm_handler, &id, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "%s", msg);
        return ESP_FAIL;
    }
    ESP_LOGI(lsm6dso_app_layer_tag, "{Function %s in line %d}: Chip ID: 0x%X", __func__, __LINE__, id);

    /* Set Accelerometer ODR */
    ret = lsm6dso_hwl_set_odr_acc(lsm->i2c_lsm_handler, LSM6DSO_ACC_ODR_12P5_HZ, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    lsm->_private.acc_odr = lsm_params.acc.odr;

    /* Set Accelerometer Full-scale */
    ret = lsm6dso_hwl_set_fs_acc(lsm->i2c_lsm_handler, LSM6DSO_ACC_FS_4G, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    switch(lsm_params.acc.fs) {
        case LSM6DSO_ACC_FS_2G:
            lsm->_private.acc_so = LSM6DSO_ACC_SO_2G;
            break;
        case LSM6DSO_ACC_FS_4G:
            lsm->_private.acc_so = LSM6DSO_ACC_SO_4G;
            break;
        case LSM6DSO_ACC_FS_8G:
            lsm->_private.acc_so = LSM6DSO_ACC_SO_8G;
            break;
        case LSM6DSO_ACC_FS_16G:
            lsm->_private.acc_so = LSM6DSO_ACC_SO_16G;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* Enable Accelerometer LPF2 (additionally to LPF1 -> LPF1 cut off frequency is ODR/2) */
    if(lsm_params.acc.lpf2_en == LSM6DSO_ACC_LPF2_ENABLE) {
        ret = lsm6dso_hwl_en_lpf2_acc(lsm->i2c_lsm_handler, msg, sizeof(msg));
        if(ret != ESP_OK) {
            ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }
    }

    /* Set Gyroscope ODR */
    ret = lsm6dso_hwl_set_odr_gyro(lsm->i2c_lsm_handler, lsm_params.gyro.odr, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    lsm->_private.gyro_odr = lsm_params.gyro.odr;

    /* Set Gyroscope Full-scale */
    ret = lsm6dso_hwl_set_fs_gyro(lsm->i2c_lsm_handler, lsm_params.gyro.fs, msg, sizeof(msg));
    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    switch(lsm_params.gyro.fs) {
        case LSM6DSO_FS_GYRO_125_DPS:
            lsm->_private.gyro_so = LSM6DSO_GYRO_SO_125_DPS;
            break;
        case LSM6DSO_FS_GYRO_250_DPS:
            lsm->_private.gyro_so = LSM6DSO_GYRO_SO_250_DPS;
            break;
        case LSM6DSO_FS_GYRO_500_DPS:
            lsm->_private.gyro_so = LSM6DSO_GYRO_SO_500_DPS;
            break;
        case LSM6DSO_FS_GYRO_1000_DPS:
            lsm->_private.gyro_so = LSM6DSO_GYRO_SO_1000_DPS;
            break;
        case LSM6DSO_FS_GYRO_2000_DPS:
            lsm->_private.gyro_so = LSM6DSO_GYRO_SO_2000_DPS;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* Enable Gyroscope LPF1 */
    if(lsm_params.gyro.lpf1_en == LSM6DSO_GYRO_LPF1_ENABLE) {
        ret = lsm6dso_hwl_en_lpf1_gyro(lsm->i2c_lsm_handler, lsm_params.gyro.lpf1_mode, msg, sizeof(msg));
        if(ret != ESP_OK) {
            ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }
    }

    /* Enable Gyroscope HPF */
    if(lsm_params.gyro.hpf_en == LSM6DSO_GYRO_HPF_ENABLE) {
        ret = lsm6dso_hwl_en_lpf1_gyro(lsm->i2c_lsm_handler, lsm_params.gyro.hpf_mode, msg, sizeof(msg));
        if(ret != ESP_OK) {
            ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: Initialize Lsm6dso object --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t lsm6dso_measure(lsm6dso_t *lsm) {
    char msg[256];
    esp_err_t ret = ESP_OK;
    lsm6dso_data_ready_t data_ready = {.acc_ready = false, .gyro_ready = false, .temp_ready = false};
    
    /* Check if accelerometer and gyroscope have the same sampling frequency */
    if(acc_freq_hz[lsm->_private.acc_odr] == gyro_freq_hz[lsm->_private.gyro_odr]) {
        lsm6dso_hwl_data_ready(lsm->i2c_lsm_handler, &data_ready, msg, sizeof(msg));

        if((data_ready.acc_ready) && (data_ready.gyro_ready)) {
            ret = lsm6dso_read_gyro_and_acc(lsm->i2c_lsm_handler, &(lsm->temp), &(lsm->gyro), lsm->_private.gyro_so, lsm->_private.acc_unit, &(lsm->acc), lsm->_private.acc_so, lsm->_private.acc_unit, msg, sizeof(msg));
        }
    } else {
        ESP_LOGE(
            lsm6dso_app_layer_tag,
            "{Function %s in line %d}: Wrong function call. If Gyroscope and Accelerometer doesn't share the same ODR then they should be read separately",
            __func__, __LINE__
        );
    }

    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: %s", __func__, __LINE__, msg);
    }

    return ret;
}

esp_err_t lsm6dso_measure_acc(lsm6dso_t *lsm) {
    char msg[256];
    esp_err_t ret = ESP_OK;
    lsm6dso_data_ready_t data_ready = {.acc_ready = false, .gyro_ready = false, .temp_ready = false};
    
    lsm6dso_hwl_data_ready(lsm->i2c_lsm_handler, &data_ready, msg, sizeof(msg));
    if(data_ready.acc_ready) {
        ret = lsm6dso_read_acc(lsm->i2c_lsm_handler, &(lsm->temp), &(lsm->acc), lsm->_private.acc_so, lsm->_private.acc_unit, msg, sizeof(msg));
    }

    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: %s", __func__, __LINE__, msg);
    }

    return ret;
}

esp_err_t lsm6dso_measure_gyro(lsm6dso_t *lsm) {
    char msg[256];
    esp_err_t ret = ESP_OK;
    lsm6dso_data_ready_t data_ready = {.acc_ready = false, .gyro_ready = false, .temp_ready = false};
    
    lsm6dso_hwl_data_ready(lsm->i2c_lsm_handler, &data_ready, msg, sizeof(msg));
    if(data_ready.gyro_ready) {
        ret = lsm6dso_read_gyro(lsm->i2c_lsm_handler, &(lsm->temp), &(lsm->gyro), lsm->_private.gyro_so, lsm->_private.gyro_unit, msg, sizeof(msg));
    }

    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: %s", __func__, __LINE__, msg);
    }

    return ret;
}

esp_err_t lsm6dso_measure_temp(lsm6dso_t *lsm) {
    char msg[256];
    esp_err_t ret = ESP_OK;
    lsm6dso_data_ready_t data_ready = {.acc_ready = false, .gyro_ready = false, .temp_ready = false};
    
    lsm6dso_hwl_data_ready(lsm->i2c_lsm_handler, &data_ready, msg, sizeof(msg));
    if(data_ready.temp_ready) {
        ret = lsm6dso_read_temp(lsm->i2c_lsm_handler, &(lsm->temp), msg, sizeof(msg));
    }

    if(ret != ESP_OK) {
        ESP_LOGE(lsm6dso_app_layer_tag, "{Function %s in line %d}: %s", __func__, __LINE__, msg);
    }

    return ret;
}


void Lsm6dso(lsm6dso_t *lsm) {
    memset(lsm, 0, sizeof(lsm6dso_t));

    lsm->init = lsm6dso_init;
    lsm ->measure = lsm6dso_measure;
}
