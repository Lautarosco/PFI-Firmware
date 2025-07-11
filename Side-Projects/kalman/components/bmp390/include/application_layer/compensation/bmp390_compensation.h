#ifndef BMP390_COMPENSATION_H
#define BMP390_COMPENSATION_H

#include <stdint.h>

typedef struct bmp390_calib_data bmp390_calib_data_t;

/**
 * @brief Compensate temperature raw values using compensation coefficients
 * 
 * @note New values are computed in °C (Celsius)
 * 
 * @param adc_temp: Raw temperature measure
 * @param calib_data: Compensation coefficients
 * 
 * @retval
 *      - (double) Compensated temperature in °C
 */
double bmp390_compensate_temp_celsius(uint32_t adc_temp, bmp390_calib_data_t calib_data);

/**
 * @brief Compensate pressure raw values using compensation coefficients
 * 
 * @note New values are computed in Pa (Pascal)
 * 
 * @param adc_press: Raw pressure measure
 * @param comp_temp: Compensated temperature result of last temperature measurerement
 * @param calib_data: Compensation coefficients
 * 
 * @retval
 *      - (double) Compensated pressure in Pa
 */
double bmp390_compensate_press_pascal(uint32_t adc_press, double comp_temp, bmp390_calib_data_t calib_data);

/**
 * @brief Compensate pressure raw values using compensation coefficients
 * 
 * @note New values are computed in hPa (hecto-Pascal)
 * 
 * @param adc_press: Raw pressure measure
 * @param comp_temp: Compensated temperature result of last temperature measurerement
 * @param calib_data: Compensation coefficients
 * 
 * @retval
 *      - (double) Compensated pressure in hPa
 */
double bmp390_compensate_press_hectopascal(uint32_t adc_press, double comp_temp, bmp390_calib_data_t calib_data);

#endif
