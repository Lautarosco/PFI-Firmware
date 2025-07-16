#pragma once

#include <hardware_layer/lsm6dso_hwl_layer.h>

#define LSM6DSO_ODR_ACC_DEFAULT                 LSM6DSO_XL_ODR_12P5_HZ
#define LSM6DSO_FS_ACC_DEFAULT                  LSM6DSO_XL_FS_4G
#define LSM6DSO_LPF2_EN_ACC_DEFAULT             LSM6DSO_XL_LPF2_DISABLE
#define LSM6DSO_ODR_GYRO_DEFAULT                LSM6DSO_ODR_G_12P5_HZ
#define LSM6DSO_FS_GYRO_DEFAULT                 LSM6DSO_FS_GYRO_250_DPS
#define LSM6DSO_FS_GYRO_UNIT_DEFAULT            LSM6DSO_GYRO_UNIT_DEG

typedef struct lsm6dso_gyro_params {
    lsm6dso_odr_gyro_t odr;
    lsm6dso_fs_gyro_t fs;
    lsm6dso_lpf1_en_gyro_t lpf1_en;
    lsm6dso_lpf1_gyro_t lpf1_mode;
    lsm6dso_hpf_en_gyro_t hpf_en;
    lsm6dso_hpf_gyro_t hpf_mode;
    lsm6dso_gyro_unit_t unit;
} lsm6dso_gyro_params_t;

typedef struct lsm6dso_acc_params {
    lsm6dso_odr_acc_t odr;
    lsm6dso_fs_acc_t fs;
    lsm6dso_lpf2_en_acc_t lpf2_en;
    lsm6dso_acc_unit_t unit;
} lsm6dso_acc_params_t;

typedef struct lsm6dso_params {
    lsm6dso_acc_params_t acc;
    lsm6dso_gyro_params_t gyro;
    i2c_master_dev_handle_t *i2c_lsm_handler;           /* Pointer to I2C LSM6DSO bus handler */
} lsm6dso_params_t;

typedef struct lsm6dso_private {
    lsm6dso_gyro_unit_t gyro_unit;                      /* Gyroscope unit */
    lsm6dso_acc_unit_t acc_unit;                        /* Accelerometer unit */
    float gyro_so;                                      /* Gyroscope sensitivity [dps/LSB] */
    float acc_so;                                       /* Accelerometer sensitivity [g/LSB] */
    lsm6dso_odr_acc_t acc_odr;                          /* Accelerometer sampling frequency */
    lsm6dso_odr_gyro_t gyro_odr;                        /* Gyroscope sampling frequency */
} lsm6dso_private_t;

typedef struct lsm6dso lsm6dso_t;
typedef struct lsm6dso {
    i2c_master_dev_handle_t i2c_lsm_handler;            /* I2C LSM6DSO bus handler */
    lsm6dso_private_t _private;                         /* Private attributes (SHOULD NOT BE MODIFIED) */
    lsm6dso_gyro_t gyro;
    lsm6dso_acc_t acc;
    float temp;

    esp_err_t (*init)(lsm6dso_t *lsm, lsm6dso_params_t lsm_params);
    esp_err_t (*measure)(lsm6dso_t *lsm);
} lsm6dso_t;

bool lsm6dso_check_params(lsm6dso_params_t lsm_params);
esp_err_t lsm6dso_init(lsm6dso_t *lsm, lsm6dso_params_t lsm_params);
void Lsm6dso(lsm6dso_t *lsm);
esp_err_t lsm6dso_measure_acc(lsm6dso_t *lsm);
esp_err_t lsm6dso_measure_gyro(lsm6dso_t *lsm);
esp_err_t lsm6dso_measure_temp(lsm6dso_t *lsm);
