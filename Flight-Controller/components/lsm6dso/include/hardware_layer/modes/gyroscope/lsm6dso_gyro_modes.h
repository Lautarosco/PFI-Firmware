#pragma once

#define LSM6DSO_GYRO_SO_125_DPS             0.004375f               /* dps/LSB */
#define LSM6DSO_GYRO_SO_250_DPS             0.00875f                /* dps/LSB */
#define LSM6DSO_GYRO_SO_500_DPS             0.0175f                 /* dps/LSB */
#define LSM6DSO_GYRO_SO_1000_DPS            0.035f                  /* dps/LSB */
#define LSM6DSO_GYRO_SO_2000_DPS            0.07f                   /* dps/LSB */

typedef enum lsm6dso_odr_gyro {
    LSM6DSO_ODR_GYRO_POWER_DOWN,
    LSM6DSO_ODR_GYRO_12P5_HZ,
    LSM6DSO_ODR_GYRO_26_HZ,
    LSM6DSO_ODR_GYRO_52_HZ,
    LSM6DSO_ODR_GYRO_104_HZ,
    LSM6DSO_ODR_GYRO_208_HZ,
    LSM6DSO_ODR_GYRO_416_HZ,
    LSM6DSO_ODR_GYRO_833_HZ,
    LSM6DSO_ODR_GYRO_1666_HZ,
    LSM6DSO_ODR_GYRO_3330_HZ,
    LSM6DSO_ODR_GYRO_6660_HZ,
} lsm6dso_odr_gyro_t;

/* The order of frequency elements MUST coincide with lsm6dso_odr_gyro_t enumeration */
extern float gyro_freq_hz[11];

typedef enum lsm6dso_fs_gyro {
    LSM6DSO_FS_GYRO_250_DPS,
    LSM6DSO_FS_GYRO_500_DPS,
    LSM6DSO_FS_GYRO_1000_DPS,
    LSM6DSO_FS_GYRO_2000_DPS,
    LSM6DSO_FS_GYRO_125_DPS
} lsm6dso_fs_gyro_t;

typedef enum lsm6dso_lpf1_en_gyro {
    LSM6DSO_GYRO_LPF1_DISABLE,
    LSM6DSO_GYRO_LPF1_ENABLE
} lsm6dso_lpf1_en_gyro_t;


typedef enum lsm6dso_lpf1_gyro {
    LSM6DSO_GYRO_LPF1_OFF = -1,
    LSM6DSO_GYRO_LPF1_0,
    LSM6DSO_GYRO_LPF1_1,
    LSM6DSO_GYRO_LPF1_2,
    LSM6DSO_GYRO_LPF1_3,
    LSM6DSO_GYRO_LPF1_4,
    LSM6DSO_GYRO_LPF1_5,
    LSM6DSO_GYRO_LPF1_6,
    LSM6DSO_GYRO_LPF1_7
} lsm6dso_lpf1_gyro_t;

typedef enum lsm6dso_hpf_en_gyro {
    LSM6DSO_GYRO_HPF_DISABLE,
    LSM6DSO_GYRO_HPF_ENABLE
} lsm6dso_hpf_en_gyro_t;

typedef enum lsm6dso_hpf_gyro {
    LSM6DSO_GYRO_HPF_OFF = -1,
    LSM6DSO_GYRO_HPF_16_mHz,
    LSM6DSO_GYRO_HPF_65_mHz,
    LSM6DSO_GYRO_HPF_260_mHz,
    LSM6DSO_GYRO_HPF_1P04_Hz
} lsm6dso_hpf_gyro_t;

typedef enum lsm6dso_gyro_unit {
    LSM6DSO_GYRO_UNIT_DEG,               /* Gyroscope unit is °/s = dps */
    LSM6DSO_GYRO_UNIT_RAD                /* Gyroscope unit is rad/s */
} lsm6dso_gyro_unit_t;

typedef struct lsm6dso_gyro {
    float x;                        /* Pitch [dps or rad/s] */
    float y;                        /* Roll [dps or rad/s] */
    float z;                        /* Yaw [dps or rad/s] */
} lsm6dso_gyro_t;
