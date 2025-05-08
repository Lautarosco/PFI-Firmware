#include "bmp180.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"

void app_main(void) {
    bmp180_t sensor;
    bmp180_init(&sensor, 0x77, 21, 22, BMP180_OSS_ULTRA_HIGH_RESOLUTION);
    
    float temp;
    int32_t pressure;
    
    while(1) {
        bmp180_read_temperature(&sensor, &temp);
        bmp180_read_pressure(&sensor, &pressure);
        printf("Temperature: %.2f C, Pressure: %ld Pa\n", temp, pressure);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}