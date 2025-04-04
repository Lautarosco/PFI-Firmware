#include <get_modes_names.h>

#include <bmp280_registers.h>


typedef struct bmp280_modes_names {
    union {
        bmp280_TStandby_t t_sb;
        bmp280_IIRCoeff_t iir;
        bmp280_SerialInterface_t serial;
        bmp280_OsT_t os_temp;
        bmp280_OsP_t os_press;
        bmp280_PowerMode_t power_mode;
    };
    const char * name;
} bmp280_modes_names_t;

bmp280_modes_names_t tsb_arr[] = {
    {.t_sb = T_NO_MEASURE, .name = "T_NO_MEASURE"},
    {.t_sb = T_OS_X1,      .name = "T_OS_X1"},
    {.t_sb = T_OS_X2,      .name = "T_OS_X2"},
    {.t_sb = T_OS_X4,      .name = "T_OS_X4"},
    {.t_sb = T_OS_X8,      .name = "T_OS_X8"},
    {.t_sb = T_OS_X16,     .name = "T_OS_X16"}
};

bmp280_modes_names_t iir_arr[] = {
    {.iir = IIR_NO_FILTER, .name = "IIR_NO_FILTER"},
    {.iir = IIR_2,         .name = "IIR_2"},
    {.iir = IIR_4,         .name = "IIR_4"},
    {.iir = IIR_8,         .name = "IIR_8"},
    {.iir = IIR_16,        .name = "IIR_16"}
};

bmp280_modes_names_t serial_arr[] = {
    {.serial = I2C,        .name = "I2C"},
    {.serial = SPI_4_WIRE, .name = "SPI_4_WIRE"},
    {.serial = SPI_3_WIRE, .name = "SPI_3_WIRE"}
};

bmp280_modes_names_t ost_arr[] = {
    {.os_temp = T_NO_MEASURE, .name = "T_NO_MEASURE"},
    {.os_temp = T_OS_X1,      .name = "T_OS_X1"},
    {.os_temp = T_OS_X2,      .name = "T_OS_X2"},
    {.os_temp = T_OS_X4,      .name = "T_OS_X4"},
    {.os_temp = T_OS_X8,      .name = "T_OS_X8"},
    {.os_temp = T_OS_X16,     .name = "T_OS_X16"}
};

bmp280_modes_names_t osp_arr[] = {
    {.os_press = P_NO_MEASURE, .name = "P_NO_MEASURE"},
    {.os_press = P_OS_X1,      .name = "P_OS_X1"},
    {.os_press = P_OS_X2,      .name = "P_OS_X2"},
    {.os_press = P_OS_X4,      .name = "P_OS_X4"},
    {.os_press = P_OS_X8,      .name = "P_OS_X8"},
    {.os_press = P_OS_X16,     .name = "P_OS_X16"}
};

bmp280_modes_names_t powerMode_arr[] = {
    {.power_mode = SLEEP_MODE,  .name = "SLEEP_MODE"},
    {.power_mode = FORCED_MODE, .name = "FORCED_MODE"},
    {.power_mode = NORMAL_MODE, .name = "NORMAL_MODE"}
};

const char * bmp280_GetTsbName(bmp280_TStandby_t t_sb) {
    for(int i = 0; i < ((sizeof(tsb_arr)) / (sizeof(tsb_arr[0]))); i++) {
        if(tsb_arr[i].t_sb == t_sb) {
            return tsb_arr[i].name;
        }
    }

    return "NOT FOUND";
}


const char * bmp280_GetIIRName(bmp280_IIRCoeff_t iir) {
    for(int i = 0; i < ((sizeof(iir_arr)) / (sizeof(iir_arr[0]))); i++) {
        if(iir_arr[i].iir == iir) {
            return iir_arr[i].name;
        }
    }

    return "NOT FOUND";
}


const char * bmp280_GetSerialName(bmp280_SerialInterface_t serial) {
    for(int i = 0; i < ((sizeof(serial_arr)) / (sizeof(serial_arr[0]))); i++) {
        if(serial_arr[i].serial == serial) {
            return serial_arr[i].name;
        }
    }

    return "NOT FOUND";
}


const char * bmp280_GetOsTName(bmp280_OsT_t os_temp) {
    for(int i = 0; i < ((sizeof(ost_arr)) / (sizeof(ost_arr[0]))); i++) {
        if(ost_arr[i].os_temp == os_temp) {
            return ost_arr[i].name;
        }
    }

    return "NOT FOUND";
}


const char * bmp280_GetOsPName(bmp280_OsP_t os_press) {
    for(int i = 0; i < ((sizeof(osp_arr)) / (sizeof(osp_arr[0]))); i++) {
        if(osp_arr[i].os_press == os_press) {
            return osp_arr[i].name;
        }
    }

    return "NOT FOUND";
}


const char * bmp280_GetPowerModeName(bmp280_PowerMode_t power_mode) {
    for(int i = 0; i < ((sizeof(powerMode_arr)) / (sizeof(powerMode_arr[0]))); i++) {
        if(powerMode_arr[i].power_mode == power_mode) {
            return powerMode_arr[i].name;
        }
    }

    return "NOT FOUND";
}
