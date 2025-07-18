#include "indicators.h"
#include <string.h>
#include <driver/gpio.h>
#include <gpio_layout.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>


esp_err_t Indicators(indicators_t * indicators) {
    ESP_LOGI("INDICATORS", "INIT OK\n");
    memset(indicators, 0, sizeof(indicators_t));

    /* Initialize indicators */
    indicators->power.gpio_pin = RED_LED_PIN;
    indicators->gps.gpio_pin = GRE_LED_PIN;
    indicators->transmitter.gpio_pin = BLU_LED_PIN;
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = (1ULL << indicators->power.gpio_pin) | 
                        (1ULL << indicators->gps.gpio_pin) | 
                        (1ULL << indicators->transmitter.gpio_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        // Handle configuration error
        printf("GPIO config failed: %s\n", esp_err_to_name(ret));
    }
    gpio_set_level(indicators->power.gpio_pin, 0);
    gpio_set_level(indicators->gps.gpio_pin, 0);
    gpio_set_level(indicators->transmitter.gpio_pin, 0);

    return ESP_OK;
}