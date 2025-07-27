#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

typedef struct {
    TickType_t cycle_start_time;
    uint16_t cycle_period_ms;    // Shared cycle period (e.g., 5000ms)
} indicator_timing_t;

typedef struct {
    uint8_t blink_count;        // Number of blinks per cycle
    uint16_t on_time_ms;        // Time LED stays ON (milliseconds)
    uint16_t off_time_ms;       // Time LED stays OFF between blinks (milliseconds)
    int _cycle_time_ms;
    // No cycle_period_ms here - it's shared globally
} blink_profile_t;

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINKING_FAST,
    LED_BLINKING_SLOW,

    LED_BLINKING_PROFILE  // may replace fast and slow blinking
} indicator_state_t;

typedef struct led led_t;
typedef struct led {
    gpio_num_t gpio_pin;
    indicator_state_t state;
    blink_profile_t profile;

    esp_err_t (* set_profile)(led_t* led, blink_profile_t profile);
    
    // Internal state tracking for this specific LED
    struct {
        TickType_t last_toggle_time;
        uint8_t current_blink;
        bool is_on;
        bool blink_sequence_done;
    } _internal;
} led_t;

typedef struct buzzer {
    /* Buzzer GPIO pin */
    int gpio_pin;

    /* Buzzer state */
    indicator_state_t state;
} buzzer_t;

typedef struct indicators indicators_t;

typedef struct indicators {
    led_t power;
    led_t gps;
    led_t transmitter;
    
    indicator_timing_t led_timing;

    esp_err_t (*create_profile)(indicators_t* indicators, int count, int t_on, int t_off, blink_profile_t* profile);

    
    buzzer_t buzzer;
} indicators_t;

esp_err_t Indicators(indicators_t * indicators, uint16_t cycle_period_ms);
