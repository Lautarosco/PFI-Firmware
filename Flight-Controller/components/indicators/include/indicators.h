#pragma once

#include <esp_err.h>
#include <stdbool.h>

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINKING_FAST,
    LED_BLINKING_SLOW
} indicator_state_t;

typedef struct led {
    /* LED GPIO pin */
    int gpio_pin;

    /* LED state */
    indicator_state_t state;

} led_t;

typedef struct buzzer {
    /* Buzzer GPIO pin */
    int gpio_pin;

    /* Buzzer state */
    indicator_state_t state;
} buzzer_t;

typedef struct indicators {
    led_t power;
    led_t gps;
    led_t transmitter;
    buzzer_t buzzer;
} indicators_t;

esp_err_t Indicators(indicators_t * indicators);
