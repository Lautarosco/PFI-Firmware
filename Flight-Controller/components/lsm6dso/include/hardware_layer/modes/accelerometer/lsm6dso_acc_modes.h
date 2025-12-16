#pragma once

#define LSM6DSO_ACC_SO_2G             0.000061f                 /* g/LSB */
#define LSM6DSO_ACC_SO_4G             0.000122f                 /* g/LSB */
#define LSM6DSO_ACC_SO_8G             0.000244f                 /* g/LSB */
#define LSM6DSO_ACC_SO_16G            0.000488f                 /* g/LSB */

typedef enum lsm6dso_odr_acc {
    LSM6DSO_ACC_ODR_POWER_DOWN,
    LSM6DSO_ACC_ODR_12P5_HZ,
    LSM6DSO_ACC_ODR_26_HZ,
    LSM6DSO_ACC_ODR_52_HZ,
    LSM6DSO_ACC_ODR_104_HZ,
    LSM6DSO_ACC_ODR_208_HZ,
    LSM6DSO_ACC_ODR_416_HZ,
    LSM6DSO_ACC_ODR_833_HZ,
    LSM6DSO_ACC_ODR_1666_HZ,
    LSM6DSO_ACC_ODR_3330_HZ,
    LSM6DSO_ACC_ODR_6660_HZ,
    LSM6DSO_ACC_ODR_1P6_HZ_HM_MODE_1
} lsm6dso_odr_acc_t;

/* The order of frequency elements MUST coincide with lsm6dso_odr_acc_t enumeration */
extern float acc_freq_hz[12];

typedef enum lsm6dso_fs_acc {
    LSM6DSO_ACC_FS_2G,
    LSM6DSO_ACC_FS_16G,
    LSM6DSO_ACC_FS_4G,
    LSM6DSO_ACC_FS_8G
} lsm6dso_fs_acc_t;

typedef enum lsm6dso_lpf2_en_acc {
    LSM6DSO_ACC_LPF2_DISABLE,
    LSM6DSO_ACC_LPF2_ENABLE
} lsm6dso_lpf2_en_acc_t;

typedef enum lsm6dso_lpf2_acc {
    LSM6DSO_ACC_LPF2_OFF = -1,
    LSM6DSO_ACC_LPF2_0,  // ODR/4
    LSM6DSO_ACC_LPF2_1,  // ODR/10
    LSM6DSO_ACC_LPF2_2,  // ODR/20
    LSM6DSO_ACC_LPF2_3,  // ODR/45
    LSM6DSO_ACC_LPF2_4,  // ODR/100
    LSM6DSO_ACC_LPF2_5,  // ODR/200
    LSM6DSO_ACC_LPF2_6,  // ODR/400
    LSM6DSO_ACC_LPF2_7  // ODR/800
} lsm6dso_lpf2_acc_t;

typedef struct lsm6dso_acc {
    float x;                        /* x-axis [m/s^2] */
    float y;                        /* y-axis [m/s^2] */
    float z;                        /* z-axis [m/s^2] */
} lsm6dso_acc_t;

typedef enum lsm6dso_acc_unit {
    LSM6DSO_ACC_UNIT_G,                 /* Accelerometer unit is g */
    LSM6DSO_ACC_UNIT_MS2                /* Accelerometer unit is m/s^2 */
} lsm6dso_acc_unit_t;
