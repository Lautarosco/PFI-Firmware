#include "stdint.h"
#include "esp_err.h"

#ifndef COMMUNICATION
#define COMMUNICATION
esp_err_t i2c_init(int sda, int scl);
esp_err_t bmi160_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data);
esp_err_t bmi160_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
#endif