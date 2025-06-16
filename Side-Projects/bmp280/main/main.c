#include <stdio.h>
#include <bmp280.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#define BMP280_ADDR     0x76    /* Sensor address - 0x76 if SDO = 0 or 0x77 if SDO = 1 */
#define GPIO_SDA        21      /* I2C SDA data */
#define GPIO_SCL        22      /* I2C SCL clock */

static const char * main_tag = "MAIN APP";  /* Main tag */

i2c_master_bus_handle_t master_i2c_bus_handler; /* I2C bus handler for master device (MCU) */


void vTaskBmp280Measure(void * bmp);


void app_main(void) {
    bmp280_t bmp;
    Bmp280(&bmp);   /* Make an instance of Bmp280 Class */

    if(bmp.init(&bmp, &master_i2c_bus_handler, BMP280_ADDR, GPIO_SDA, GPIO_SCL) != ESP_OK) {
        return;
    }

    double p0 = bmp.get_avg_pressure(bmp, 1500);    /* n = 6000 ~ 4 minutes */
    // double p0 = 1015.867004; /* Relative pressure (depends on location) */
    double z0 = bmp.get_avg_altitude(bmp, p0, 100);
    double t = 0.0;
    double p = 0.0;
    double z = 0.0;

    xTaskCreatePinnedToCore(vTaskBmp280Measure, "task1", 1024 * 2, (void *) &bmp, 1, NULL, 1);

    while(1) {

        t = bmp.get_temperature();
        p = bmp.get_pressure();
        z = bmp.get_altitude(p, p0) - z0;

        printf("Temperature: %lf °C\tPressure: %lf hPa\tAltitude: %lf m\r\n", t, p, z);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vTaskBmp280Measure(void * _bmp) {
    bmp280_t * bmp = (bmp280_t *) _bmp;

    while(1) {
        bmp->measure(bmp->i2c.bmp280_i2c_bus_handler);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
