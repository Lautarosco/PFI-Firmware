#include "bmp180.h"
#include "communication.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"

static const char* TAG = "BMP180";

// Read calibration data
static esp_err_t read_calibration_data(bmp180_t* dev) {
    uint8_t calib_data[BMP180_CALIBRATION_DATA_LEN];
    esp_err_t ret = bmp180_read_bytes(dev->i2c_addr, BMP180_CALIBRATION_REG, calib_data, BMP180_CALIBRATION_DATA_LEN);
    
    if(ret != ESP_OK) return ret;

    dev->calib.AC1 = (calib_data[0] << 8) | calib_data[1];
    dev->calib.AC2 = (calib_data[2] << 8) | calib_data[3];
    dev->calib.AC3 = (calib_data[4] << 8) | calib_data[5];
    dev->calib.AC4 = (calib_data[6] << 8) | calib_data[7];
    dev->calib.AC5 = (calib_data[8] << 8) | calib_data[9];
    dev->calib.AC6 = (calib_data[10] << 8) | calib_data[11];
    dev->calib.B1  = (calib_data[12] << 8) | calib_data[13];
    dev->calib.B2  = (calib_data[14] << 8) | calib_data[15];
    dev->calib.MB  = (calib_data[16] << 8) | calib_data[17];
    dev->calib.MC  = (calib_data[18] << 8) | calib_data[19];
    dev->calib.MD  = (calib_data[20] << 8) | calib_data[21];
    
    return ESP_OK;
}

// Initialize sensor
esp_err_t bmp180_init(bmp180_t* dev, uint8_t i2c_addr, int sda, int scl, uint8_t oss) {
    dev->i2c_addr = i2c_addr;
    dev->sda_pin = sda;
    dev->scl_pin = scl;
    dev->oss = oss;

    ESP_ERROR_CHECK(i2c_init(sda, scl));
    ESP_ERROR_CHECK(read_calibration_data(dev));
    return ESP_OK;
}

// Raw temperature measurement
static esp_err_t read_raw_temperature(bmp180_t* dev, int32_t* ut) {
    ESP_ERROR_CHECK(bmp180_write_byte(dev->i2c_addr, BMP180_CTRL_MEAS_REG, BMP180_TEMP_MEASURE));
    vTaskDelay(pdMS_TO_TICKS(5));  // Conversion delay
    uint8_t data[2];
    ESP_ERROR_CHECK(bmp180_read_bytes(dev->i2c_addr, BMP180_OUT_MSB_REG, data, 2));
    *ut = (data[0] << 8) | data[1];
    return ESP_OK;
}

// Raw pressure measurement
static esp_err_t read_raw_pressure(bmp180_t* dev, int32_t* up) {
    uint8_t cmd = BMP180_PRESS_MEASURE + (dev->oss << 6);
    ESP_ERROR_CHECK(bmp180_write_byte(dev->i2c_addr, BMP180_CTRL_MEAS_REG, cmd));
    
    // Wait based on oversampling
    uint32_t delay = (1 << dev->oss) * 3 + 5;
    vTaskDelay(pdMS_TO_TICKS(delay));
    
    uint8_t data[3];
    ESP_ERROR_CHECK(bmp180_read_bytes(dev->i2c_addr, BMP180_OUT_MSB_REG, data, 3));
    *up = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - dev->oss);
    return ESP_OK;
}

// Calculate true temperature
esp_err_t bmp180_read_temperature(bmp180_t* dev, float* temperature) {
    int32_t ut;
    read_raw_temperature(dev, &ut);
    
    int32_t X1 = (ut - dev->calib.AC6) * dev->calib.AC5 >> 15;
    int32_t X2 = dev->calib.MC << 11 / (X1 + dev->calib.MD);
    int32_t B5 = X1 + X2;
    *temperature = ((B5 + 8) >> 4) / 10.0f;
    
    return ESP_OK;
}

// Calculate true pressure
esp_err_t bmp180_read_pressure(bmp180_t* dev, int32_t* pressure) {
    int32_t ut, up;
    read_raw_temperature(dev, &ut);  // Required for B5
    read_raw_pressure(dev, &up);
    
    // Temperature calculations
    int32_t X1 = (ut - dev->calib.AC6) * dev->calib.AC5 >> 15;
    int32_t X2 = dev->calib.MC << 11 / (X1 + dev->calib.MD);
    int32_t B5 = X1 + X2;
    
    // Pressure calculations
    int32_t B6 = B5 - 4000;
    X1 = (dev->calib.B2 * (B6 * B6 >> 12)) >> 11;
    X2 = dev->calib.AC2 * B6 >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((dev->calib.AC1 * 4 + X3) << dev->oss) + 2) / 4;
    
    X1 = dev->calib.AC3 * B6 >> 13;
    X2 = (dev->calib.B1 * (B6 * B6 >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = dev->calib.AC4 * (uint32_t)(X3 + 32768) >> 15;
    uint32_t B7 = ((uint32_t)up - B3) * (50000 >> dev->oss);
    
    if(B7 < 0x80000000) {
        *pressure = (B7 * 2) / B4;
    } else {
        *pressure = (B7 / B4) * 2;
    }
    
    X1 = (*pressure >> 8) * (*pressure >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * *pressure) >> 16;
    *pressure += (X1 + X2 + 3791) >> 4;
    
    return ESP_OK;
}