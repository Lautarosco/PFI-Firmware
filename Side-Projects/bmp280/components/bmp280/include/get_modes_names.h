#ifndef GET_MODES_NAMES_H
#define GET_MODES_NAMES_H

#include <bmp280_registers.h>


/**
 * @brief Get name of selected standby time
 * 
 * @param t_sb: Selected standby time
 * 
 * @return Standby time name
 */
const char * bmp280_GetTsbName(bmp280_TStandby_t t_sb);

/**
 * @brief Get name of selected IIR filter coefficient
 * 
 * @param iir: Selected IIR filter coefficient
 * 
 * @return IIR filter coefficient name
 */
const char * bmp280_GetIIRName(bmp280_IIRCoeff_t iir);

/**
 * @brief Get name of selected serial interface
 * 
 * @param serial: Selected serial interface
 * 
 * @return Serial interface name
 */
const char * bmp280_GetSerialName(bmp280_SerialInterface_t serial);

/**
 * @brief Get name of selected oversampling temperature
 * 
 * @param os_temp: Selected oversampling temperature
 * 
 * @return Oversampling temperature name
 */
const char * bmp280_GetOsTName(bmp280_OsT_t os_temp);

/**
 * @brief Get name of selected oversampling pressure
 * 
 * @param os_press: Selected oversampling pressure
 * 
 * @return Oversampling pressure name
 */
const char * bmp280_GetOsPName(bmp280_OsP_t os_press);

/**
 * @brief Get name of selected power mode
 * 
 * @param power_mode: Selected power mode
 * 
 * @return Power mode name
 */
const char * bmp280_GetPowerModeName(bmp280_PowerMode_t power_mode);

#endif
