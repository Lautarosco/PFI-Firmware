#include <hardware_layer/bmp390_modes_names.h>

typedef struct bmp390_modes_names {
    const char *name;
    unsigned int mode_value;
} bmp390_modes_desc_t;

/* ========== Matrixes of name-mode ========== */

bmp390_modes_desc_t iir_coef_arr[] = {
    {.name = "IIR internal filter coefficient is 0",   .mode_value = BMP390_CONFIG_COEF_0},
    {.name = "IIR internal filter coefficient is 1",   .mode_value = BMP390_CONFIG_COEF_1},
    {.name = "IIR internal filter coefficient is 3",   .mode_value = BMP390_CONFIG_COEF_3},
    {.name = "IIR internal filter coefficient is 7",   .mode_value = BMP390_CONFIG_COEF_7},
    {.name = "IIR internal filter coefficient is 15",  .mode_value = BMP390_CONFIG_COEF_15},
    {.name = "IIR internal filter coefficient is 31",  .mode_value = BMP390_CONFIG_COEF_31},
    {.name = "IIR internal filter coefficient is 63",  .mode_value = BMP390_CONFIG_COEF_63},
    {.name = "IIR internal filter coefficient is 127", .mode_value = BMP390_CONFIG_COEF_127}
};

bmp390_modes_desc_t odr_sel_arr[] = {
    {.name = "Sampling rate is 200 Hz",    .mode_value = BMP390_ODR_SEL_200_HZ},
    {.name = "Sampling rate is 100 Hz",    .mode_value = BMP390_ODR_SEL_100_HZ},
    {.name = "Sampling rate is 50 Hz",     .mode_value = BMP390_ODR_SEL_50_HZ},
    {.name = "Sampling rate is 25 Hz",     .mode_value = BMP390_ODR_SEL_25_HZ},
    {.name = "Sampling rate is 12.5 Hz",   .mode_value = BMP390_ODR_SEL_12P5_HZ},
    {.name = "Sampling rate is 6.25 Hz",   .mode_value = BMP390_ODR_SEL_6P25_HZ},
    {.name = "Sampling rate is 3.1 Hz",    .mode_value = BMP390_ODR_SEL_3P1_HZ},
    {.name = "Sampling rate is 1.5 Hz",    .mode_value = BMP390_ODR_SEL_1P5_HZ},
    {.name = "Sampling rate is 0.78 Hz",   .mode_value = BMP390_ODR_SEL_0P78_HZ},
    {.name = "Sampling rate is 0.39 Hz",   .mode_value = BMP390_ODR_SEL_0P39_HZ},
    {.name = "Sampling rate is 0.2 Hz",    .mode_value = BMP390_ODR_SEL_0P2_HZ},
    {.name = "Sampling rate is 0.1 Hz",    .mode_value = BMP390_ODR_SEL_0P1_HZ},
    {.name = "Sampling rate is 0.05 Hz",   .mode_value = BMP390_ODR_SEL_0P05_HZ},
    {.name = "Sampling rate is 0.02 Hz",   .mode_value = BMP390_ODR_SEL_0P02_HZ},
    {.name = "Sampling rate is 0.01 Hz",   .mode_value = BMP390_ODR_SEL_0P01_HZ},
    {.name = "Sampling rate is 0.006 Hz",  .mode_value = BMP390_ODR_SEL_0P006_HZ},
    {.name = "Sampling rate is 0.003 Hz",  .mode_value = BMP390_ODR_SEL_0P003_HZ},
    {.name = "Sampling rate is 0.0015 Hz", .mode_value = BMP390_ODR_SEL_0P0015_HZ}
};

bmp390_modes_desc_t osr_press_arr[] = {
    {.name = "Pressure resolution is (x1) 16 bit / 2.64 Pa",  .mode_value = BMP390_OSR_P_X1},
    {.name = "Pressure resolution is (x2) 17 bit / 1.32 Pa",  .mode_value = BMP390_OSR_P_X2},
    {.name = "Pressure resolution is (x4) 18 bit / 0.66 Pa",  .mode_value = BMP390_OSR_P_X4},
    {.name = "Pressure resolution is (x8) 19 bit / 0.33 Pa",  .mode_value = BMP390_OSR_P_X8},
    {.name = "Pressure resolution is (x16) 20 bit / 0.17 Pa",  .mode_value = BMP390_OSR_P_X16},
    {.name = "Pressure resolution is (x32) 21 bit / 0.085 Pa", .mode_value = BMP390_OSR_P_X32}
};

bmp390_modes_desc_t osr_temp_arr[] = {
    {.name = "Temperature resolution is (x1) 16 bit / 0.005 °C",   .mode_value = BMP390_OSR_T_X1},
    {.name = "Temperature resolution is (x2) 17 bit / 0.0025 °C",  .mode_value = BMP390_OSR_T_X2},
    {.name = "Temperature resolution is (x4) 18 bit / 0.0012 °C",  .mode_value = BMP390_OSR_T_X4},
    {.name = "Temperature resolution is (x8) 19 bit / 0.0006 °C",  .mode_value = BMP390_OSR_T_X8},
    {.name = "Temperature resolution is (x16) 20 bit / 0.0003 °C",  .mode_value = BMP390_OSR_T_X16},
    {.name = "Temperature resolution is (x32) 21 bit / 0.00015 °C", .mode_value = BMP390_OSR_T_X32}
};

bmp390_modes_desc_t pwr_mode_arr[] = {
    {.name = "Sleep mode",  .mode_value = BMP390_PWR_CTRL_SLEEP_MODE},
    {.name = "Forced mode", .mode_value = BMP390_PWR_CTRL_FORCED_MODE},
    {.name = "Normal mode", .mode_value = BMP390_PWR_CTRL_NORMAL_MODE}
};

bmp390_modes_desc_t press_en_arr[] = {
    {.name = "Pressure sensor enabled",     .mode_value = BMP390_PWR_CTRL_PRESS_ON},
    {.name = "Pressure sensor disabled",    .mode_value = BMP390_PWR_CTRL_PRESS_OFF}
};

bmp390_modes_desc_t temp_en_arr[] = {
    {.name = "Temperature sensor enabled",  .mode_value = BMP390_PWR_CTRL_TEMP_ON},
    {.name = "Temperature sensor disabled", .mode_value = BMP390_PWR_CTRL_TEMP_OFF}
};

bmp390_modes_desc_t spi_mode_arr[] = {
    {.name = "SPI mode is SPI 4-wire", .mode_value = BMP390_IF_CONF_SPI3_SPI4_MODE},
    {.name = "SPI mode is SPI 3-wire", .mode_value = BMP390_IF_CONF_SPI3_SPI3_MODE}
};

bmp390_modes_desc_t i2c_wdt_en_arr[] = {
    {.name ="I2C watchdog timeout enabled",  .mode_value = BMP390_IF_CONF_I2C_WDT_ON},
    {.name ="I2C watchdog timeout disabled", .mode_value = BMP390_IF_CONF_I2C_WDT_OFF}
};

bmp390_modes_desc_t i2c_wdt_tout_arr[] = {
    {.name ="I2C watchdog timeout is 1.25 ms = 1250 us", .mode_value = BMP390_IF_CONF_I2C_WDT_SEL_1250US},
    {.name ="I2C watchdog timeout is 40 ms = 40000 us",  .mode_value = BMP390_IF_CONF_I2C_WDT_SEL_40000US}
};

bmp390_modes_desc_t cmd_arr[] = {
    {.name = "Soft reset (power on reset)", .mode_value = BMP390_CMD_SOFTRESET}
};

/* ========== Public functions to get mode name ========== */

const char *bmp390_get_iir_coef_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(iir_coef_arr)) / (sizeof(iir_coef_arr[0]))); i++) {
        if(iir_coef_arr[i].mode_value == mode_value) {
            return iir_coef_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_odr_sel_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(odr_sel_arr)) / (sizeof(odr_sel_arr[0]))); i++) {
        if(odr_sel_arr[i].mode_value == mode_value) {
            return odr_sel_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_osr_press_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(osr_press_arr)) / (sizeof(osr_press_arr[0]))); i++) {
        if(osr_press_arr[i].mode_value == mode_value) {
            return osr_press_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_osr_temp_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(osr_temp_arr)) / (sizeof(osr_temp_arr[0]))); i++) {
        if(osr_temp_arr[i].mode_value == mode_value) {
            return osr_temp_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_pwr_mode_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(pwr_mode_arr)) / (sizeof(pwr_mode_arr[0]))); i++) {
        if(pwr_mode_arr[i].mode_value == mode_value) {
            return pwr_mode_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_press_status(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(press_en_arr)) / (sizeof(press_en_arr[0]))); i++) {
        if(press_en_arr[i].mode_value == mode_value) {
            return press_en_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_temp_status(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(temp_en_arr)) / (sizeof(temp_en_arr[0]))); i++) {
        if(temp_en_arr[i].mode_value == mode_value) {
            return temp_en_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_spi_mode_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(spi_mode_arr)) / (sizeof(spi_mode_arr[0]))); i++) {
        if(spi_mode_arr[i].mode_value == mode_value) {
            return spi_mode_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_i2c_wdt_tout_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(i2c_wdt_tout_arr)) / (sizeof(i2c_wdt_tout_arr[0]))); i++) {
        if(i2c_wdt_tout_arr[i].mode_value == mode_value) {
            return i2c_wdt_tout_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_i2c_wdt_en_status(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(i2c_wdt_en_arr)) / (sizeof(i2c_wdt_en_arr[0]))); i++) {
        if(i2c_wdt_en_arr[i].mode_value == mode_value) {
            return i2c_wdt_en_arr[i].name;
        }
    }

    return "NOT FOUND";
}

const char *bmp390_get_cmd_name(unsigned int mode_value) {
    for(int i = 0; i < ((sizeof(cmd_arr)) / (sizeof(cmd_arr[0]))); i++) {
        if(cmd_arr[i].mode_value == mode_value) {
            return cmd_arr[i].name;
        }
    }

    return "NOT FOUND";
}
