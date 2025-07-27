#include "indicators.h"
#include <string.h>
#include <driver/gpio.h>
#include <gpio_layout.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>


esp_err_t Led(led_t * led);
esp_err_t led_set_profile(led_t* led, const blink_profile_t profile);
esp_err_t indicator_create_profile(indicators_t* indicator, int Count, int T_On, int T_off, blink_profile_t* profile);

esp_err_t Led(led_t* led) {
    led->set_profile = led_set_profile;
    return ESP_OK;
}

esp_err_t Indicators(indicators_t * indicators, uint16_t cycle_period_ms) {
    memset(indicators, 0, sizeof(indicators_t));

    /* Initialize indicators */
    indicators->led_timing.cycle_period_ms = cycle_period_ms;
    indicators->led_timing.cycle_start_time = xTaskGetTickCount();

    Led(&indicators->power);
    Led(&indicators->gps);
    Led(&indicators->transmitter);

    indicators->power.gpio_pin = RED_LED_PIN;
    indicators->gps.gpio_pin = GRE_LED_PIN;
    indicators->transmitter.gpio_pin = BLU_LED_PIN;

    indicators->create_profile = indicator_create_profile;
    
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

    ESP_LOGI("INDICATORS", "INIT OK\n");
    return ESP_OK;
}

esp_err_t indicator_create_profile(indicators_t* indicator, int Count, int T_On, int T_off, blink_profile_t* profile) {
    memset(profile, 0, sizeof(blink_profile_t));
    profile->blink_count = Count;
    profile->on_time_ms = T_On;
    profile->off_time_ms = T_off;
    profile->_cycle_time_ms = indicator->led_timing.cycle_period_ms;

    return ESP_OK;

}

esp_err_t led_set_profile(led_t* led, const blink_profile_t profile) {

    int profile_time = (profile.on_time_ms + profile.off_time_ms)*profile.blink_count;

    if ( profile_time > profile._cycle_time_ms ) {
        ESP_LOGE("INDICATOR", "ERROR: Blink profile exceedes sync timing. (%d ms exceeds %d ms) \n", profile_time, profile._cycle_time_ms);
        return ESP_FAIL;
    }

    led->profile = profile;
    led->state = LED_BLINKING_PROFILE;
    
    // Reset internal state (will be properly synced on next cycle)
    led->_internal.current_blink = 0;
    led->_internal.is_on = false;
    led->_internal.blink_sequence_done = false;
    led->_internal.last_toggle_time = xTaskGetTickCount();
    
    return ESP_OK;
}