#ifndef BMP280_H
#define BMP280_H

#include <bmp280_structs.h>
#include <driver/i2c_master.h>

/**
 * @brief Make an instance of Bmp280 Class
 * 
 * @return Pointer to Bmp280 object
 */
bmp280_t * Bmp280(void);

/**
 * @brief Calculate pressure 'n' times and get average value. Should be used as an alternative to sea level pressure
 * 
 * @param bmp: Bmp280 object
 * @param n: Total samples
 * 
 * @return 64-bit calculated average pressure
 */
double bmp280_GetRelativeP(bmp280_t bmp, int n);

#endif
