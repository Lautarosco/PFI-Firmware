#include <hardware_layer/bmp390_modes_names.h>

typedef struct bmp390_modes_names {
    const char *name;
    union {
        bmp390_config_coef_t iir_coef;
        bmp390_odr_sel_t odr_sel;
        bmp390_osr_press_t osr_press;
        bmp390_osr_temp_t osr_temp;
        bmp390_pwr_ctrl_mode_t pwr_mode;
        bmp390_if_conf_reg_spi_t spi_mode;
        bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout;
        bmp390_cmd_t cmd;
    };
} bmp390_modes_names_t;

/* ========== Matrixes of name-mode ========== */

bmp390_modes_names_t iir_coef_arr[] = {
    {.name = "IIR coefficient is 0",    .iir_coef = BMP390_CONFIG_COEF_0},
    {.name = "IIR coefficient is 1",    .iir_coef = BMP390_CONFIG_COEF_1},
    {.name = "IIR coefficient is 3",    .iir_coef = BMP390_CONFIG_COEF_3},
    {.name = "IIR coefficient is 7",    .iir_coef = BMP390_CONFIG_COEF_7},
    {.name = "IIR coefficient is 15",   .iir_coef = BMP390_CONFIG_COEF_15},
    {.name = "IIR coefficient is 31",   .iir_coef = BMP390_CONFIG_COEF_31},
    {.name = "IIR coefficient is 63",   .iir_coef = BMP390_CONFIG_COEF_63},
    {.name = "IIR coefficient is 127",  .iir_coef = BMP390_CONFIG_COEF_127}
};

bmp390_modes_names_t odr_sel_arr[] = {
    {.name = "Sampling rate is 200 Hz",    .odr_sel = BMP390_ODR_SEL_200_HZ},
    {.name = "Sampling rate is 100 Hz",    .odr_sel = BMP390_ODR_SEL_100_HZ},
    {.name = "Sampling rate is 50 Hz",     .odr_sel = BMP390_ODR_SEL_50_HZ},
    {.name = "Sampling rate is 25 Hz",     .odr_sel = BMP390_ODR_SEL_25_HZ},
    {.name = "Sampling rate is 12.5 Hz",   .odr_sel = BMP390_ODR_SEL_12P5_HZ},
    {.name = "Sampling rate is 6.25 Hz",   .odr_sel = BMP390_ODR_SEL_6P25_HZ},
    {.name = "Sampling rate is 3.1 Hz",    .odr_sel = BMP390_ODR_SEL_3P1_HZ},
    {.name = "Sampling rate is 1.5 Hz",    .odr_sel = BMP390_ODR_SEL_1P5_HZ},
    {.name = "Sampling rate is 0.78 Hz",   .odr_sel = BMP390_ODR_SEL_0P78_HZ},
    {.name = "Sampling rate is 0.39 Hz",   .odr_sel = BMP390_ODR_SEL_0P39_HZ},
    {.name = "Sampling rate is 0.2 Hz",    .odr_sel = BMP390_ODR_SEL_0P2_HZ},
    {.name = "Sampling rate is 0.1 Hz",    .odr_sel = BMP390_ODR_SEL_0P1_HZ},
    {.name = "Sampling rate is 0.05 Hz",   .odr_sel = BMP390_ODR_SEL_0P05_HZ},
    {.name = "Sampling rate is 0.02 Hz",   .odr_sel = BMP390_ODR_SEL_0P02_HZ},
    {.name = "Sampling rate is 0.01 Hz",   .odr_sel = BMP390_ODR_SEL_0P01_HZ},
    {.name = "Sampling rate is 0.006 Hz",  .odr_sel = BMP390_ODR_SEL_0P006_HZ},
    {.name = "Sampling rate is 0.003 Hz",  .odr_sel = BMP390_ODR_SEL_0P003_HZ},
    {.name = "Sampling rate is 0.0015 Hz", .odr_sel = BMP390_ODR_SEL_0P0015_HZ}
};

bmp390_modes_names_t osr_press_arr[] = {
    {.name = "Pressure resolution is 16 bit / 2.64 Pa",     .osr_press = BMP390_OSR_P_X1},
    {.name = "Pressure resolution is 17 bit / 1.32 Pa",     .osr_press = BMP390_OSR_P_X2},
    {.name = "Pressure resolution is 18 bit / 0.66 Pa",     .osr_press = BMP390_OSR_P_X4},
    {.name = "Pressure resolution is 19 bit / 0.33 Pa",     .osr_press = BMP390_OSR_P_X8},
    {.name = "Pressure resolution is 20 bit / 0.17 Pa",    .osr_press = BMP390_OSR_P_X16},
    {.name = "Pressure resolution is 21 bit / 0.085 Pa",    .osr_press = BMP390_OSR_P_X32}
};

bmp390_modes_names_t osr_temp_arr[] = {
    {.name = "Temperature resolution is 16 bit / 0.005 °C",     .osr_temp = BMP390_OSR_T_X1},
    {.name = "Temperature resolution is 17 bit / 0.0025 °C",     .osr_temp = BMP390_OSR_T_X2},
    {.name = "Temperature resolution is 18 bit / 0.0012 °C",     .osr_temp = BMP390_OSR_T_X4},
    {.name = "Temperature resolution is 19 bit / 0.0006 °C",     .osr_temp = BMP390_OSR_T_X8},
    {.name = "Temperature resolution is 20 bit / 0.0003 °C",    .osr_temp = BMP390_OSR_T_X16},
    {.name = "Temperature resolution is 21 bit / 0.00015 °C",    .osr_temp = BMP390_OSR_T_X32}
};

bmp390_modes_names_t pwr_mode_arr[] = {
    {.name = "Power mode is Sleep mode",  .pwr_mode = BMP390_PWR_CTRL_SLEEP_MODE},
    {.name = "Power mode is Forced mode", .pwr_mode = BMP390_PWR_CTRL_FORCED_MODE},
    {.name = "Power mode is Normal mode", .pwr_mode = BMP390_PWR_CTRL_NORMAL_MODE}
};

bmp390_modes_names_t spi_mode_arr[] = {
    {.name = "SPI mode is SPI 4-wire", .spi_mode = BMP390_IF_CONF_SPI3_SPI4_MODE},
    {.name = "SPI mode is SPI 3-wire", .spi_mode = BMP390_IF_CONF_SPI3_SPI3_MODE}
};

bmp390_modes_names_t i2c_wdt_tout_arr[] = {
    {.name ="I2C watchdog timeout is 1.25 ms = 1250 us",    .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_1250US},
    {.name ="I2C watchdog timeout is 40 ms = 40000 us",   .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_40000US}
};

bmp390_modes_names_t cmd_arr[] = {
    {.name = "Soft reset (power on reset)", .cmd = BMP390_CMD_SOFTRESET}
};

/* ========== Public functions to get mode name ========== */

const char *bmp390_get_iir_coef_name(bmp390_config_coef_t iir_coef) {
    for(int i = 0; i < ((sizeof(iir_coef_arr)) / (sizeof(iir_coef_arr[0]))); i++) {
        if(iir_coef_arr[i].iir_coef == iir_coef) {
            return iir_coef_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_odr_sel_name(bmp390_odr_sel_t odr_sel) {
    for(int i = 0; i < ((sizeof(odr_sel_arr)) / (sizeof(odr_sel_arr[0]))); i++) {
        if(odr_sel_arr[i].odr_sel == odr_sel) {
            return odr_sel_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_osr_press_name(bmp390_osr_press_t osr_press) {
    for(int i = 0; i < ((sizeof(osr_press_arr)) / (sizeof(osr_press_arr[0]))); i++) {
        if(osr_press_arr[i].osr_press == osr_press) {
            return osr_press_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_osr_temp_name(bmp390_osr_temp_t osr_temp) {
    for(int i = 0; i < ((sizeof(osr_temp_arr)) / (sizeof(osr_temp_arr[0]))); i++) {
        if(osr_temp_arr[i].osr_temp == osr_temp) {
            return osr_temp_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_pwr_mode_name(bmp390_pwr_ctrl_mode_t pwr_mode) {
    for(int i = 0; i < ((sizeof(pwr_mode_arr)) / (sizeof(pwr_mode_arr[0]))); i++) {
        if(pwr_mode_arr[i].pwr_mode == pwr_mode) {
            return pwr_mode_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_spi_mode_name(bmp390_if_conf_reg_spi_t spi_mode) {
    for(int i = 0; i < ((sizeof(spi_mode_arr)) / (sizeof(spi_mode_arr[0]))); i++) {
        if(spi_mode_arr[i].spi_mode == spi_mode) {
            return spi_mode_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_i2c_wdt_tout_name(bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout) {
    for(int i = 0; i < ((sizeof(i2c_wdt_tout_arr)) / (sizeof(i2c_wdt_tout_arr[0]))); i++) {
        if(i2c_wdt_tout_arr[i].i2c_wdt_tout == i2c_wdt_tout) {
            return i2c_wdt_tout_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_cmd_name(bmp390_cmd_t cmd) {
    for(int i = 0; i < ((sizeof(cmd_arr)) / (sizeof(cmd_arr[0]))); i++) {
        if(cmd_arr[i].cmd == cmd) {
            return cmd_arr[i].name;
        }
    }

    return "NOT FOUND";
}
