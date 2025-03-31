#include <stdio.h>
#include <bmp280.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#include <registers_map/bmp280_registers.h>
#include <bmp280_ll_drivers/communications/com.h>


#define BMP280_ADDR     0x76    /* Sensor address */
#define GPIO_SDA        21      /* I2C SDA data */
#define GPIO_SCL        22      /* I2C SCL clock */

static const char * main_tag = "MAIN APP";

void app_main(void) {
    bmp280_t * bmp = Bmp280();

    if(bmp->init(bmp, BMP280_ADDR, GPIO_SDA, GPIO_SCL) != ESP_OK) {
        ESP_LOGE(main_tag, "Failed to initialized Bmp280 object");
        return;
    }

/*
    bmp->measure(bmp);
    return;
*/

    while(1) {
        bmp->measure(bmp);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
