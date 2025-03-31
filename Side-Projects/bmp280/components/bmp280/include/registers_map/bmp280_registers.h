#ifndef BMP280_REGISTERS_H
#define BMP280_REGISTERS_H

/* #################### ADDRESS #################### */

#define BMP280_CONFIG_REG       0xF5    /* Settings register */
#define BMP280_CTRL_MEAS_REG    0xF4    /* Measurements register */

/* #################### ID #################### */

#define BMP280_ID_REG   0xD0    /* Chip ID */


/* #################### RESET #################### */

#define BMP280_RESET_REG   0xE0    /* Reset register */


/* #################### STATUS #################### */

#define BMP280_STATUS_REG   0xF3    /* Status register */


/* #################### RAW MEASUREMENTS #################### */

#define BMP280_PRESS_MSB_REG       0xF7    /* <7:0> Raw pressure MSB */
#define BMP280_PRESS_LSB_REG       0xF8    /* <7:0> Raw pressure LSB */
#define BMP280_PRESS_XLSB_REG      0xF9    /* <7:4> Raw pressure LXSB */
#define BMP280_TEMP_MSB_REG        0xFA    /* <7:0> Raw temperature MSB */
#define BMP280_TEMP_LSB_REG        0xFB    /* <7:0> Raw temperature LSB */
#define BMP280_TEMP_XLSB_REG       0xFC    /* <7:4> Raw temperature LXSB */


/* #################### CALIBRATION #################### */

#define BMP280_DIG_T1_LSB_REG   0x88    /* LSB part of dig_T1; unsigned short */
#define BMP280_DIG_T1_MSB_REG   0x89    /* MSB part of dig_T1; unsigned short */

#define BMP280_DIG_T2_LSB_REG   0x8A    /* LSB part of dig_T2; signed short */
#define BMP280_DIG_T2_MSB_REG   0x8B    /* MSB part of dig_T2; signed short */

#define BMP280_DIG_T3_LSB_REG   0x8C    /* LSB part of dig_T3; unsigned short */
#define BMP280_DIG_T3_MSB_REG   0x8D    /* MSB part of dig_T3; signed short */

#define BMP280_DIG_P1_LSB_REG   0x8E    /* LSB part of dig_P1; unsigned short */
#define BMP280_DIG_P1_MSB_REG   0x8F    /* MSB part of dig_P1; unsigned short */

#define BMP280_DIG_P2_LSB_REG   0x90    /* LSB part of dig_P2; signed short */
#define BMP280_DIG_P2_MSB_REG   0x91    /* MSB part of dig_P2; signed short */

#define BMP280_DIG_P3_LSB_REG   0x92    /* LSB part of dig_P3; signed short */
#define BMP280_DIG_P3_MSB_REG   0x93    /* MSB part of dig_P3; signed short */

#define BMP280_DIG_P4_LSB_REG   0x94    /* LSB part of dig_P4; signed short */
#define BMP280_DIG_P4_MSB_REG   0x95    /* MSB part of dig_P4; signed short */

#define BMP280_DIG_P5_LSB_REG   0x96    /* LSB part of dig_P5; signed short */
#define BMP280_DIG_P5_MSB_REG   0x97    /* MSB part of dig_P5; signed short */

#define BMP280_DIG_P6_LSB_REG   0x98    /* LSB part of dig_P6; signed short */
#define BMP280_DIG_P6_MSB_REG   0x99    /* MSB part of dig_P6; signed short */

#define BMP280_DIG_P7_LSB_REG   0x9A    /* LSB part of dig_P7; signed short */
#define BMP280_DIG_P7_MSB_REG   0x9B    /* MSB part of dig_P7; signed short */

#define BMP280_DIG_P8_LSB_REG   0x9C    /* LSB part of dig_P8; signed short */
#define BMP280_DIG_P8_MSB_REG   0x9D    /* MSB part of dig_P8; signed short */

#define BMP280_DIG_P9_LSB_REG   0x9E    /* LSB part of dig_P9; signed short */
#define BMP280_DIG_P9_MSB_REG   0x9F    /* MSB part of dig_P9; signed short */


/* #################### CONFIG #################### */

typedef enum bmp280_TStandby {
    TS_MANUAL = 0,      /* Use if when Forced mode is enabled */
    TS_0p5_MS = 0,      /* Time between samples of 0.5 ms */
    TS_62p5_MS,         /* Time between samples of 62.5 ms */
    TS_125_MS,          /* Time between samples of 125 ms */
    TS_250_MS,          /* Time between samples of 250 ms */
    TS_500_MS,          /* Time between samples of 500 ms */
    TS_1000_MS,         /* Time between samples of 1000 ms */
    TS_2000_MS,         /* Time between samples of 2000 ms */
    TS_4000_MS          /* Time between samples of 4000 ms */
} bmp280_TStandby_t;


typedef enum bmp280_IIRCoeff {
    IIR_NO_FILTER,      /* Don't use IIR filter */
    IIR_2,             /* Filter coefficient of 2 */
    IIR_4,             /* Filter coefficient of 4 */
    IIR_8,             /* Filter coefficient of 8 */
    IIR_16             /* Filter coefficient of 16 */
} bmp280_IIRCoeff_t;


typedef enum bmp280_SerialInterface {
    I2C,                /* Enable I2C interface for serial communication */
    SPI_4_WIRE = 0,     /* Enable 4-wire SPI interface for serial communication */
    SPI_3_WIRE          /* Enable 3-wire SPI interface for serial communication */
} bmp280_SerialInterface_t;


/* #################### OVERSAMPLING #################### */

typedef enum bmp280_OsT {
    T_NO_MEASURE,     /* Don't measure temperature */
    T_OS_X1,          /* Oversampling x1; Resolution: 16 bit / .005 °C */
    T_OS_X2,          /* Oversampling x2; Resolution: 17 bit / .0025 °C */
    T_OS_X4,          /* Oversampling x4; Resolution: 18 bit / .0012 °C */
    T_OS_X8,          /* Oversampling x8; Resolution: 19 bit / .0006 °C */
    T_OS_X16          /* Oversampling x16; Resolution: 20 bit / .0003 °C */
} bmp280_OsT_t;

typedef enum bmp280_OsP {
    P_NO_MEASURE,     /* Don't measure pressure */
    P_OS_X1,          /* Ultra low power; oversampling x1; Resolution: 16 bit / 2.62 Pa */
    P_OS_X2,          /* Low power; oversampling x2; Resolution: 17 bit / 1.31 Pa */
    P_OS_X4,          /* Standard resolution; oversampling x4; Resolution: 18 bit / .66 Pa */
    P_OS_X8,          /* High resolution; oversampling x8; Resolution: 19 bit / .33 Pa */
    P_OS_X16          /* Ultra high resolution; oversampling x16; Resolution: 20 bit / .16 Pa */
} bmp280_OsP_t;


/* #################### MODE #################### */

// #define BMP280_FORCED_MODE      0b01        /* Forced mode */
typedef enum bmp280_PowerMode {
    SLEEP_MODE,             /* Sleep mode */
    FORCED_MODE,            /* Forced mode */
    NORMAL_MODE = 3         /* Normal mode */
} bmp280_PowerMode_t;


#endif
