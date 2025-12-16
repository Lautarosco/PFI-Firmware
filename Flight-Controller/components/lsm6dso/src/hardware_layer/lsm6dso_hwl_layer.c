#include <hardware_layer/lsm6dso_hwl_layer.h>
#include <esp_log.h>

#define I2C_TIMEOUT_MS              50                              /* I2C Read/Write operations timeout in milliseconds */
#define PI_RAD                      3.14159265358979323846f         /* Approximation to PI number */
#define GRAVITY                     9.80665f                        /* Approximation to force of gravity */
#define LSM6DSO_TSEN_C              256.0f                          /* Typical temperature sensitivity */
#define LSM6DSO_TEMP_OFFSET         25.0f                           /* Sensor outputs 0 LSB at 25 °C */

const char *lsm6dso_hwl_tag = "[LSM6DSO_HW_LAYER]";

/* The order of frequency elements MUST coincide with } lsm6dso_odr_gyro_t enumeration */
float gyro_freq_hz[11] = {
    0.0f,
    12.5f,
    26.0f,
    52.0f,
    104.0f,
    208.0f,
    416.0f,
    833.0f,
    1666.0f,
    3330.0f,
    6660.0f
};

/* The order of frequency elements MUST coincide with lsm6dso_odr_acc_t enumeration */
float acc_freq_hz[12] = {
    0.0f,
    12.5f,
    26.0f,
    52.0f,
    104.0f,
    208.0f,
    416.0f,
    833.0f,
    1666.0f,
    3330.0f,
    6660.0f,
    1.6f
};

esp_err_t lsm6dso_hwl_read_reg(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t *reg_addr, unsigned int reg_len, uint8_t *reg_value, unsigned int bytes, const char *func_caller) {
    if((reg_addr == NULL) || (reg_value == NULL) || (func_caller == NULL)) {
        ESP_LOGE(
            lsm6dso_hwl_tag,
            "{Function %s in line %d}: [Caller: %s] Invalid (NULL) parameters",
            __func__, __LINE__, func_caller
        );
        return ESP_FAIL;
    }

    *reg_value = 0;
    
    esp_err_t ret = i2c_master_transmit_receive(i2c_lsm_handler, reg_addr, reg_len, reg_value, bytes, I2C_TIMEOUT_MS);
    if(ret != ESP_OK) {
        ESP_LOGE(
            lsm6dso_hwl_tag,
            "{Function %s in line %d}: [Caller: %s] Read content of register 0x%X --> FAILED",
            __func__, __LINE__, func_caller, *reg_addr
        );
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_write_reg(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t reg_addr, uint8_t reg_value, const char *func_caller) {
    uint8_t write_data[2] = {reg_addr, reg_value};
    esp_err_t ret = i2c_master_transmit(i2c_lsm_handler, write_data, sizeof(write_data), I2C_TIMEOUT_MS);
    if(ret != ESP_OK) {
        ESP_LOGE(
            lsm6dso_hwl_tag,
            "{Function %s in line %d}: [Caller: %s] Write 0x%x to 0x%X register --> FAILED",
            __func__, __LINE__, func_caller,
            reg_value, reg_addr
        );
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_get_chip_id(i2c_master_dev_handle_t i2c_lsm_handler, uint8_t *id, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_WHO_AM_I_REG;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), id, sizeof(uint8_t), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: Get chip ID --> FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_set_odr_acc(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_odr_acc_t odr, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL1_XL_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(4, 4, reg_value, odr), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Accelerometer ODR --> 0x%X", __func__, __LINE__, reg_value >> 4);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_set_fs_acc(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_fs_acc_t fs, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL1_XL_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(2, 2, reg_value, fs), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Accelerometer FS --> 0x%X", __func__, __LINE__, (reg_value & (0b11 << 2)) >> 2);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_en_lpf2_acc(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_lpf2_acc_t filter_mode, char *msg, unsigned int msg_len) {

    uint8_t reg_addr = LSM6DSO_CTRL1_XL_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 1, reg_value, LSM6DSO_ACC_LPF2_ENABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Accelerometer LPF2 status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 1)) >> 1);
    
    reg_addr = LSM6DSO_CTRL8_XL_REG;

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(3, 5, reg_value, filter_mode), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Accelerometer LPF2 Mode --> 0x%X", __func__, __LINE__, (reg_value & 0b11100000)>>5);


    return ESP_OK;
}

esp_err_t lsm6dso_hwl_dis_lpf2_acc(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL1_XL_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 1, reg_value, LSM6DSO_ACC_LPF2_DISABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Accelerometer LPF2 status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 1)) >> 1);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_set_odr_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_odr_gyro_t odr, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL2_G_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(4, 4, reg_value, odr), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope ODR --> 0x%X", __func__, __LINE__, (reg_value & (0b1111 << 4)) >> 4);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_set_fs_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_fs_gyro_t fs, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL2_G_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    if(fs == LSM6DSO_FS_GYRO_125_DPS) {
        ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 1, reg_value, 1), __func__);
    } else {
        ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(2, 2, reg_value, fs), __func__);    
    }

    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    if(fs == LSM6DSO_FS_GYRO_125_DPS) {
        ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope FS --> +-125 dps", __func__, __LINE__);
    } else {
        ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope FS --> 0x%X", __func__, __LINE__, (reg_value & (0b11 << 2)) >> 2);
    }

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_sw_reset(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL3_C_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: Reset LSM6DSO device --> FAILED",
            __func__, __LINE__
        );
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 0, reg_value, 1), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: Reset LSM6DSO device --> FAILED",
            __func__, __LINE__
        );
        return ESP_FAIL;
    }

    do
    {
        lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    } while (reg_value & 0b1);
    

    ESP_LOGI(lsm6dso_hwl_tag, "Reset LSM6DSO device --> OK");

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_en_lpf1_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_lpf1_gyro_t lpf_mode, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL4_C_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 1, reg_value, LSM6DSO_GYRO_LPF1_ENABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope LPF1 status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 1)) >> 1);

    reg_addr = LSM6DSO_CTRL6_C_REG;

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(3, 0, reg_value, lpf_mode), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope LPF1 --> 0x%X", __func__, __LINE__, reg_value & 0b111);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_dis_lpf1_gyro(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL6_C_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 1, reg_value, LSM6DSO_GYRO_LPF1_DISABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope LPF1 status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 1)) >> 1);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_en_hpf_gyro(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_hpf_gyro_t hpf_mode, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL7_G_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 6, reg_value, LSM6DSO_GYRO_HPF_ENABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope HPF status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 6)) >> 6);

    reg_addr = LSM6DSO_CTRL6_C_REG;

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(2, 4, reg_value, hpf_mode), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope HPF --> 0x%X", __func__, __LINE__, (reg_value & (0b11 << 4)) >> 4);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_dis_hpf_gyro(i2c_master_dev_handle_t i2c_lsm_handler, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_CTRL7_G_REG;
    uint8_t reg_value = 0;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_write_reg(i2c_lsm_handler, reg_addr, LSM6DSO_SET_BITS(1, 6, reg_value, LSM6DSO_GYRO_HPF_DISABLE), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(lsm6dso_hwl_tag, "{Function %s in line %d}: Gyroscope HPF status --> 0x%X", __func__, __LINE__, (reg_value & (1U << 6)) >> 6);

    return ESP_OK;
}

esp_err_t lsm6dso_hwl_data_ready(i2c_master_dev_handle_t i2c_lsm_handler, lsm6dso_data_ready_t *data_ready, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_STATUS_REG_STATUS_SPIAux_REG;
    uint8_t reg_value = 0;

    data_ready->temp_ready = false;
    data_ready->acc_ready = false;
    data_ready->gyro_ready = false;

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), &reg_value, sizeof(reg_value), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    data_ready->temp_ready = (reg_value & (1U << 2)) != 0;
    data_ready->acc_ready  = (reg_value & (1U << 0)) != 0;
    data_ready->gyro_ready = (reg_value & (1U << 1)) != 0;

    return ESP_OK;
}

esp_err_t lsm6dso_read_gyro_and_acc(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_gyro_t *gyro, float gyro_so,
    lsm6dso_gyro_unit_t gyro_unit, lsm6dso_acc_t *acc, float acc_so, lsm6dso_acc_unit_t acc_unit,
    float acc_x_off, float acc_y_off, float acc_z_off, float gyro_x_off, float gyro_y_off, float gyro_z_off,
    char *msg, unsigned int msg_len
) {
    uint8_t reg_addr = LSM6DSO_OUT_TEMP_L_REG;
    uint8_t out[14];

    /* Perform a burst read of gyroscope x/y/z and accelerometer x/y/z */
    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), out, sizeof(out), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    /* Update temperature value */
    *temp = LSM6DSO_TEMP_OFFSET + ((float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) / LSM6DSO_TSEN_C));

    /* Update gyroscope values */
    gyro->x = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[3], out[2])) * gyro_so) - gyro_x_off);
    gyro->y = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[5], out[4])) * gyro_so) - gyro_y_off);
    gyro->z = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[7], out[6])) * gyro_so) - gyro_z_off);
    if(gyro_unit == LSM6DSO_GYRO_UNIT_RAD) {
        gyro->x *= (PI_RAD / 180.0f);
        gyro->y *= (PI_RAD / 180.0f);
        gyro->z *= (PI_RAD / 180.0f);
    }

    /* Update accelerometer values */
    acc->x = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[9], out[8])) * acc_so) - acc_x_off);
    acc->y = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[11], out[10])) * acc_so) - acc_y_off);
    acc->z = (float) ((((int16_t) LSM6DSO_CONCAT_BYTES(out[13], out[12])) * acc_so) - acc_z_off);
    if(acc_unit == LSM6DSO_ACC_UNIT_MS2) {
        acc->x *= GRAVITY;
        acc->y *= GRAVITY;
        acc->z *= GRAVITY;
    }

    return ESP_OK;
}

esp_err_t lsm6dso_read_gyro(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_gyro_t *gyro, float gyro_so,
    lsm6dso_gyro_unit_t gyro_unit, char *msg, unsigned int msg_len
) {
    uint8_t reg_addr = LSM6DSO_OUTX_L_G_REG;
    uint8_t out[6];

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), out, sizeof(out), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    if(gyro_unit == LSM6DSO_GYRO_UNIT_DEG) {
        gyro->x = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) * gyro_so);
        gyro->y = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[3], out[2])) * gyro_so);
        gyro->z = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[5], out[4])) * gyro_so);
    } else {
        gyro->x = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) * gyro_so * (PI_RAD / 180.0f));
        gyro->y = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[3], out[2])) * gyro_so * (PI_RAD / 180.0f));
        gyro->z = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[5], out[4])) * gyro_so * (PI_RAD / 180.0f));
    }

    return ESP_OK;
}

esp_err_t lsm6dso_read_acc(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, lsm6dso_acc_t *acc, float acc_so,
    lsm6dso_acc_unit_t acc_unit, char *msg, unsigned int msg_len
) {
    uint8_t reg_addr = LSM6DSO_OUTX_L_A_REG;
    uint8_t out[6];

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), out, sizeof(out), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    if(acc_unit == LSM6DSO_ACC_UNIT_G) {
        acc->x = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) * acc_so);
        acc->y = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[3], out[2])) * acc_so);
        acc->z = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[5], out[4])) * acc_so);
    } else {
        acc->x = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) * acc_so * GRAVITY);
        acc->y = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[4], out[2])) * acc_so * GRAVITY);
        acc->z = (float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[5], out[4])) * acc_so * GRAVITY);
    }

    return ESP_OK;
}

esp_err_t lsm6dso_read_temp(i2c_master_dev_handle_t i2c_lsm_handler, float *temp, char *msg, unsigned int msg_len) {
    uint8_t reg_addr = LSM6DSO_OUT_TEMP_L_REG;
    uint8_t out[2];

    esp_err_t ret = lsm6dso_hwl_read_reg(i2c_lsm_handler, &reg_addr, sizeof(reg_addr), out, sizeof(out), __func__);
    if(ret != ESP_OK) {
        snprintf(
            msg, msg_len,
            "{Function %s in line %d}: FAILED",
            __func__, __LINE__
        );

        return ESP_FAIL;
    }

    *temp = LSM6DSO_TEMP_OFFSET + ((float) (((int16_t) LSM6DSO_CONCAT_BYTES(out[1], out[0])) / LSM6DSO_TSEN_C));

    return ESP_OK;
}
