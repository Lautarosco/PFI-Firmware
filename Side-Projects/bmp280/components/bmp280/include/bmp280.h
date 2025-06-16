#ifndef BMP280_H
#define BMP280_H

#include <bmp280_structs.h>
#include <driver/i2c_master.h>

/**
 * @brief Takes a bmp280_t variable to make an instance of Bmp280 Class
 * @param bmp: bmp280_t bmp
 * @return none
 */
void Bmp280(bmp280_t *bmp);

#endif
