#include "indicators.h"
#include <string.h>
#include <driver/gpio.h>

// GPIO pin definitions for indicators
#define LED1_PIN       GPIO_NUM_12
#define LED2_PIN       GPIO_NUM_13
#define LED3_PIN       GPIO_NUM_14

esp_err_t Indicators(indicators_t * indicators) {
    memset(indicators, 0, sizeof(indicators_t));

    /* Initialize indicators */
    indicators->power.gpio_pin = LED3_PIN;
    indicators->gps.gpio_pin = LED2_PIN;
    indicators->transmitter.gpio_pin = LED1_PIN;

    return ESP_OK;
}