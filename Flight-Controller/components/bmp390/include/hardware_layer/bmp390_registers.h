#ifndef BMP390_REGISTERS_H
#define BMP390_REGISTERS_H

/**
 * Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
 * Last update:
 * Rev:
 */

/**
 * Registers types
 * - RW: read/write
 * - Read only: RO
 * - Write only: WO
 */

/**
 * Registers name's format
 * <sensor_name>_<register_name>_<register_type>_REG
 */

/**
 * Notes
 * - Every register is made of 8 bits
 * - Interface selection is done automatically based on CSB pin status
 *      * CSB = 0 --> SPI (Once CSB pin is pulled down interface remains SPI until the next power on reset)
 *      * CSB = 1 --> I2C
 * - The 7-bit device address is 0b111011x
 *      * The last bit (x) is changeable by SDO pin value
 *          ^ SDO = 0 --> Device address is 0b1110110 = 0x76
 *          ^ SDO = 1 --> Device address is 0b1110111 = 0x77
 *          ^ SDO = floating pin --> Device address is undefined
 */


/* =========== CMD =========== */

#define BMP390_CMD_RW_REG       0x7E    /* Available commands */

typedef enum bmp390_cmd {
    BMP390_CMD_FIFO_FLUSH = 176,        /* Clears all data in the FIFO, does not change FIFO_CONFIG registers */
    BMP390_CMD_SOFTRESET = 182          /* Triggers a reset, all user configuration settings are overwritten with their default state */
} bmp390_cmd_t;

/* =========== CONFIG =========== */

#define BMP390_CONFIG_RW_REG        0x1F    /* IIR coefficients */

typedef enum bmp390_config_bit {
    BMP390_CONFIG_IIR = 1           /* Bits[3:1] Filter coefficient for IIR filter */
} bmp390_config_bit_t;

typedef enum bmp390_config_coef {
    BMP390_CONFIG_COEF_0,       /* Filter coefficient is 0 --> Bypass mode */
    BMP390_CONFIG_COEF_1,       /* Filter coefficient is 1 */
    BMP390_CONFIG_COEF_3,       /* Filter coefficient is 3 */
    BMP390_CONFIG_COEF_7,       /* Filter coefficient is 7 */
    BMP390_CONFIG_COEF_15,      /* Filter coefficient is 15 */
    BMP390_CONFIG_COEF_31,      /* Filter coefficient is 31*/
    BMP390_CONFIG_COEF_63,      /* Filter coefficient is 63 */
    BMP390_CONFIG_COEF_127      /* Filter coefficient is 127 */
} bmp390_config_coef_t;

/* =========== ODR =========== */

#define BMP390_ODR_RW_REG       0x1D    /* Set the configuration of the output data rates */

typedef enum bmp390_odr_sel_bit {
    BMP390_ODR_ODR_SEL          /* Bits[4:0], Subdivision factor for pressure and temperature measurements is 2^value */
} bmp390_odr_sel_bit_t;

typedef enum bmp390_odr_sel {
    BMP390_ODR_SEL_200_HZ,      /* ODR 200 Hz, sampling period of 5 ms */
    BMP390_ODR_SEL_100_HZ,      /* ODR 100 Hz, sampling period of 10 ms */
    BMP390_ODR_SEL_50_HZ,       /* ODR 50 Hz, sampling period of 20 ms */
    BMP390_ODR_SEL_25_HZ,       /* ODR 25 Hz, sampling period of 40 ms */
    BMP390_ODR_SEL_12P5_HZ,     /* ODR 25/2 Hz, sampling period of 80 ms */
    BMP390_ODR_SEL_6P25_HZ,     /* ODR 25/4 Hz, sampling period of 160 ms */
    BMP390_ODR_SEL_3P1_HZ,      /* ODR 25/8 Hz, sampling period of 320 ms */
    BMP390_ODR_SEL_1P5_HZ,      /* ODR 25/16 Hz, sampling period of 640 ms */
    BMP390_ODR_SEL_0P78_HZ,     /* ODR 25/32 Hz, sampling period of 1.28 s */
    BMP390_ODR_SEL_0P39_HZ,     /* ODR 25/64 Hz, sampling period of 2.56 s */
    BMP390_ODR_SEL_0P2_HZ,      /* ODR 25/128 Hz, sampling period of 5.12 s */
    BMP390_ODR_SEL_0P1_HZ,      /* ODR 25/256 Hz, sampling period of 10.24 s */
    BMP390_ODR_SEL_0P05_HZ,     /* ODR 25/512 Hz, sampling period of 20.48 s */
    BMP390_ODR_SEL_0P02_HZ,     /* ODR 25/1024 Hz, sampling period of 40.96 s */
    BMP390_ODR_SEL_0P01_HZ,     /* ODR 25/2048 Hz, sampling period of 81.92 s */
    BMP390_ODR_SEL_0P006_HZ,    /* ODR 25/4096 Hz, sampling period of 163.84 s */
    BMP390_ODR_SEL_0P003_HZ,    /* ODR 25/8192 Hz, sampling period of 327.68 s */
    BMP390_ODR_SEL_0P0015_HZ    /* ODR 25/16384 Hz, sampling period of 655.36 s */
} bmp390_odr_sel_t;

/* =========== OSR =========== */

#define BMP390_OSR_RW_REG       0x1C    /* Controls the oversampling settings for pressure and temperature measurements */

typedef enum bmp390_osr_bit {
    BMP390_OSR_P,               /* Bits[2:0], Oversampling setting pressure measurement */
    BMP390_OSR_T = 3            /* Bits[5:3], Oversampling setting temperature measurement */
} bmp390_osr_bit_t;

typedef enum bmp390_osr_press {
    BMP390_OSR_P_X1,        /* No oversampling 16 bit / 2.64 Pa */
    BMP390_OSR_P_X2,        /* x2 oversampling 17 bit / 1.32 Pa */
    BMP390_OSR_P_X4,        /* x4 oversampling 18 bit / 0.66 Pa */
    BMP390_OSR_P_X8,        /* x8 oversampling 19 bit / 0.33 Pa */
    BMP390_OSR_P_X16,       /* x16 oversampling 20 bit / 0.17 Pa */
    BMP390_OSR_P_X32        /* x32 oversampling 21 bit / 0.085 Pa */
} bmp390_osr_press_t;

typedef enum bmp390_osr_temp {
    BMP390_OSR_T_X1,        /* No oversampling 16 bit / 0.005 °C */
    BMP390_OSR_T_X2,        /* x2 oversampling 17 bit / 0.0025 °C */
    BMP390_OSR_T_X4,        /* x4 oversampling 18 bit / 0.0012 °C */
    BMP390_OSR_T_X8,        /* x8 oversampling 19 bit / 0.0006 °C */
    BMP390_OSR_T_X16,       /* x16 oversampling 20 bit / 0.0003 °C */
    BMP390_OSR_T_X32        /* x32 oversampling 21 bit / 0.00015 °C */
} bmp390_osr_temp_t;

/* =========== PWR_CTRL =========== */

#define BMP390_PWR_CTRL_RW_REG      0x1B    /* Enables or disables pressure and temperature measurements and set power mode */

typedef enum bmp390_pwr_ctrl_bit {
    BMP390_PWR_CTRL_PRESS_EN,           /* Bit 0, Enable or disable pressure sensor. 0: Disable pressure sensor, 1: Enable pressure sensor */
    BMP390_PWR_CTRL_TEMP_EN,            /* Bit 1, Enable or disable temperature sensor. 0: Disable temperature sensor, 1: Enable temperature sensor */
    BMP390_PWR_CTRL_MODE = 4            /* Bits[5:4], 00: Sleep mode, 01/10: Forced mode, 11: Normal mode */
} bmp390_pwr_ctrl_bit_t;

typedef enum bmp390_pwr_ctrl_press_en {
    BMP390_PWR_CTRL_PRESS_OFF,      /* Disable pressure sensor */
    BMP390_PWR_CTRL_PRESS_ON        /* Enable pressure sensor */
} bmp390_pwr_ctrl_press_en_t;

typedef enum bmp390_pwr_ctrl_temp_en {
    BMP390_PWR_CTRL_TEMP_OFF,      /* Disable temperature sensor */
    BMP390_PWR_CTRL_TEMP_ON        /* Enable temperature sensor */
} bmp390_pwr_ctrl_temp_en_t;

typedef enum bmp390_pwr_ctrl_mode {
    BMP390_PWR_CTRL_SLEEP_MODE,         /* Sleep mode */
    BMP390_PWR_CTRL_FORCED_MODE,        /* Forced mode */
    BMP390_PWR_CTRL_NORMAL_MODE = 3     /* Normal mode */
} bmp390_pwr_ctrl_mode_t;

/* =========== IF_CONF =========== */

#define BMP390_IF_CONF_RW_REG          0x1A    /* Controls the serial interface settings */

typedef enum bmp390_if_conf_reg_bit {
    BMP390_IF_CONF_SPI3,                /* Bit 0, Configure SPI interface mode for primary interface. 0: SPI 4-wire mode, 1: SPI 3-wire mode */
    BMP390_IF_CONF_I2C_WDT_EN,          /* Bit 1, Enable for the I2C watchdog timer, backed by NVM. 0: Disable watchdog timer, 1: Enable watchdog timer */
    BMP390_IF_CONF_I2C_WDT_SEL          /* Bit 2, Select timer period for I2C watchdog, backed by NVM. 0: I2C watchdog timeout after 1.25 ms, 1: I2C watchdog timeout after 40 ms  */
} bmp390_if_conf_reg_bit_t;

typedef enum bmp390_if_conf_reg_spi {
    BMP390_IF_CONF_SPI3_SPI4_MODE,      /* SPI 4-wire mode */
    BMP390_IF_CONF_SPI3_SPI3_MODE       /* SPI 3-wire mode */
} bmp390_if_conf_reg_spi_t;

typedef enum bmp390_if_conf_reg_i2c_wdt_en {
    BMP390_IF_CONF_I2C_WDT_OFF,         /* Disable watchdog timer */
    BMP390_IF_CONF_I2C_WDT_ON           /* Enable watchdog timer */
} bmp390_if_conf_reg_i2c_wdt_en_t;

typedef enum bmp390_if_conf_reg_i2c_wdt_tout {
    BMP390_IF_CONF_I2C_WDT_SEL_1250US,      /* I2C watchdog timeout after 1.25 ms = 1250 us */
    BMP390_IF_CONF_I2C_WDT_SEL_40000US      /* I2C watchdog timeout after 40 ms = 40000 us */
} bmp390_if_conf_reg_i2c_wdt_tout_t;

/* =========== INT_CTRL =========== */

#define BMP390_INT_CTRL_RW_REG              0x19    /* Interrupt configuration. It affects INT_STATUS registers and the INT pin */
#define BMP390_INT_OD_BIT                   0x00    /* Bit 0, Coonfigure output. 0: Push-pull, 1: Open-drain */
#define BMP390_INT_LEVEL_BIT                0x01    /* Bit 1, Level of INT pin. 0: Active low, 1: Active high */
#define BMP390_INT_LATCH_BIT                0x02    /* Bit 2, Latching of interrupts for INT pin and INT_STATUS register. 0: Disabled, 1: Enabled */
#define BMP390_INT_DS_BIT                   0x05    /* Bit 5, ??. 0: Low, 1: High */
#define BMP390_INT_DRDY_EN_BIT              0x06    /* Bit 6, Enable pressure / temperature data ready interrupt for INT pin and INT_STATUS. 0: Disabled, 1: Enabled */

#define BMP390_FIFO_CONFIG_2_RW_REG         0x18

#define BMP390_FIFO_CONFIG_1_RW_REG         0x17

#define BMP390_FIFO_WTM_1_RW_REG            0x16

#define BMP390_FIFO_WTM_0_RW_REG            0x15

#define BMP390_FIFO_DATA_RO_REG             0x14

#define BMP390_FIFO_LENGTH_1_RO_REG         0x13

#define BMP390_FIFO_LENGTH_0_RO_REG         0x12

#define BMP390_INT_STATUS_RO_REG            0x11    /* Interrupt status. Cleared after reading */
#define BMP390_DRDY_BIT                     0x03    /* Bit 3, Data ready interrupt */

/* =========== EVENT =========== */

#define BMP390_EVENT_RO_REG                 0x10    /* Event status flags */

typedef enum bmp390_event_reg_bit {
    BMP390_EVENT_POR_DETECTED,          /* Bit 0, 1 after device power up or soft reset. Cleared on read */
    BMP390_EVENT_ITF_ACT_PT             /* Bit 1, 1 when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read */
} bmp390_event_reg_bit_t;

/* =========== SENSORTIME =========== */

#define BMP390_SENSORTIME_2_RO_REG          0x0E    /* Sensor time, sensor_time_23_16 */
#define BMP390_SENSORTIME_1_RO_REG          0x0D    /* Sensor time, sensor_time_15_8 */
#define BMP390_SENSORTIME_0_RO_REG          0x0C    /* Sensor time, sensor_time_7_0 */

/* =========== DATA[5:3] =========== */

#define BMP390_DATA_5_RO_REG                0x09    /* Temperature data, TEMP_MSB_23_16: Most significative part of temperature data */
#define BMP390_DATA_4_RO_REG                0x08    /* Temperature data, TEMP_LSB_15_8: Less significative part of temperature data */
#define BMP390_DATA_3_RO_REG                0x07    /* Temperature data, TEMP_XLSB_7_0: Contains selected resolution */

/* =========== DATA[2:0] =========== */

#define BMP390_DATA_2_RO_REG                0x06    /* Pressure data, PRESS_MSB_23_16: Most significative part of pressure data */
#define BMP390_DATA_1_RO_REG                0x05    /* Pressure data, PRESS_LSB_15_8: Less significative part of pressure data */
#define BMP390_DATA_0_RO_REG                0x04    /* Pressure data, PRESS_XLSB_7_0: Contains selected resolution */

/* =========== STATUS =========== */

#define BMP390_STATUS_RO_REG                0x03    /* Sensor status flags */

typedef enum bmp390_status_reg_bit {
    BMP390_STATUS_CMD_RDY = 4,          /* Bit 4, cmd_rdy: CMD decoder status. 0: Command in progress, 1: Command decoder is ready to accept a new command */
    BMP390_STATUS_DRDY_PRESS,           /* Data ready for pressure. It gets reset, when one pressure DATA register is read out */
    BMP390_STATUS_DRDY_TEMP             /* Data ready for temperature. It gets reset, when one temperature DATA register is read out */
} bmp390_status_reg_bit_t;

/* =========== ERR_REG =========== */

#define BMP390_ERR_REG_RO_REG               0x02    /* Sensor error conditions */

typedef enum bmp390_err_reg_bit {
    BMP390_ERR_REG_FATAL_ERR,           /* Bit 0, fatal_err. 0: No errors exist, 1: Fatal error */
    BMP390_ERR_REG_CMD_ERR,             /* Bit 1, cmd_err. 0: No errors exist, 1: Command execution failed. Cleared on read */
    BMP390_ERR_REG_CONF_ERR             /* Bit 2, conf_err. 0: No errors exist, 1: Sensor configuration error detected (only in normal mode). Cleared on read */
} bmp390_err_reg_bit_t;

/* =========== REV_ID =========== */

#define BMP390_REV_ID_RO_REG                0x01    /* Contains the mask revision of the ASIC */

/* =========== CHIP_ID =========== */

#define BMP390_CHIP_ID_RO_REG               0x00    /* Chip identification code */

/* =========== CALIBRATION_DATA =========== */

#define BMP390_CALIBRATION_DATA_INITIAL_ADDR        0x31        /* Calibration data starts at address 0x31 */

#endif
