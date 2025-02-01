#ifndef PWM_STRUCTS_H
#define PWM_STRUCTS_H

#include <driver/ledc.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Convert pulse width of pwm signal to duty cycle
 * @param pulse_ms: Pulse width in milliseconds
 * @param pwm_f: Frequency of timer in Hertz
 * @retval uint32_t
 */
#define PULSE_WIDTH_TO_DUTY( pulse_ms, pwm_f ) ( ( uint32_t ) ( ( pulse_ms / 1000.0f ) * pwm_f ) )


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details timer and channel configs of a pwm signal */

typedef struct pwm_cfg {
    /* Struct with pwm timer cfgs */
    ledc_timer_config_t timer_cfg;

    /* Struct with pwm channel cfgs */
    ledc_channel_config_t channel_cfg;

    /* Minimum width of pwm pulse in milliseconds */
    float Ton_min;
    /* Maximum width of pwm pulse in milliseconds */
    float Ton_max;

} pwm_cfg_t;


/** @details Forward declaration to avoid warning in function pointers */

typedef struct pwm pwm_t;

typedef struct pwm {
    
    /* [ A ] pwm configs */
    pwm_cfg_t pwm_cfg;
    
    /* [ A ] Minimum duty cycle of pwm signal, from 0 to 1 */
    float dc_min;
    
    /* [ A ] Maximum duty cycle of pwm signal, from 0 to 1 */
    float dc_max;
    
    /* [ A ] Maximum number to count used to limit pwm signal width */
    float max_count;
    
    /* [ A ] Minimum number to count used to limit pwm signal width */
    float min_count;
    
    /* [ A ] Maximum possible number = 2^bits */
    double n;
    
    /* [ A ] Pwm resolution = 1 / 2^bits */
    double resolution;
    
    /* [ A ] Tag of pwm signal */
    int tag;
    
    /* [ A ] Flag to check if object is initialized */
    bool init_ok;

    /** @brief [ M ] Get pwm Duty Cycle @param pwm: Address of Pwm object @retval Duty cycle of Pwm object */
    double ( * get_pwm_dc )( pwm_t * obj );

    /** @brief [ M ] Set pwm Duty Cycle @param pwm: Address of Pwm object @param duty: Duty Cycle from 0 to 1 @retval ESP_OK Success - ESP_ERR_INVALID_ARG Parameter error */
    esp_err_t ( * set_pwm_dc )( pwm_t * obj, float duty );

    /** @brief Initialize pwm signal of Brushless DC motor @param pwm: Address of Pwm object @param PwmConfigs: Pwm configs @retval ESP_OK Success - ESP_FAIL */
    esp_err_t ( *init )( pwm_t * obj, pwm_cfg_t PwmConfigs );

} pwm_t;


#endif
