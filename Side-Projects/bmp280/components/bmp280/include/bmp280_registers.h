#ifndef BMP280_REGISTERS_H
#define BMP280_REGISTERS_H

/* #################### ADDRESS #################### */
#define BMP280_CONFIG_REG       0xF5    /* Settings register */
#define BMP280_CTRL_MEAS_REG    0xF4    /* Measurements register */


/* #################### CONFIG #################### */
#define BMP280_TSB          0b000       /* Standby time of .5 ms */
#define BMP280_FILTER_16    0b100       /* IIR filter coefficient of 16 */
#define BMP280_SPI32_DIS    0b0         /* Disable 3-wire SPI interface */


/* #################### CONFIG #################### */
#define BMP280_OSRS_T_2         0b010       /* Temperature oversampling of x2 */
#define BMP280_OSRS_P_16        0b101       /* Pressure oversampling of x16 */
#define BMP280_FORCED_MODE      0b01        /* Forced mode */


/* #################### CONFIG #################### */
#define BMP280_ID_REG   0xD0    /* Chip ID */

#endif
