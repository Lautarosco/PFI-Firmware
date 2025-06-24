#include <stdio.h>
#include <pwm.h>
#include "pwm.h"
#include <esp_log.h>
#include <esp_system.h>
#include <math.h>


static const char * PWM_TAG = "PWM";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize pwm signal of Brushless DC motor
 * @param pwm: Address of Pwm object
 * @param PwmConfigs: Pwm configs
 * @retval ESP_OK if success - ESP_FAIL
 */
static esp_err_t Pwm_Init( pwm_t * pwm, pwm_cfg_t PwmConfigs ) {

    ESP_LOGI( PWM_TAG, "Initializing Pwm %d object...", pwm->tag + 1 );
    
    pwm->init_ok = true;
    pwm->pwm_cfg = PwmConfigs;

    /* Set timer configs */
    if( ledc_timer_config( &( pwm->pwm_cfg.timer_cfg ) ) != ESP_OK ) {
        
        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong timer settings!", pwm->tag );
        return ESP_FAIL;
    }

    /* Set channel configs */
    if( ledc_channel_config( &( pwm->pwm_cfg.channel_cfg ) ) != ESP_OK ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong channel settings!", pwm->tag );
        return ESP_FAIL;
    }

    pwm->n          = pow( 2, pwm->pwm_cfg.timer_cfg.duty_resolution );
    pwm->resolution = 1 / pwm->n;
    pwm->dc_min     = ( ( pwm->pwm_cfg.Ton_min / 1000.0f ) * pwm->pwm_cfg.timer_cfg.freq_hz );
    pwm->dc_max     = ( ( pwm->pwm_cfg.Ton_max / 1000.0f ) * pwm->pwm_cfg.timer_cfg.freq_hz );
    pwm->min_count  = pwm->dc_min * pwm->n;
    pwm->max_count  = pwm->dc_max * pwm->n;

    /* Set pwm duty cycle to it's minimum value */
    pwm->set_pwm_dc( pwm, pwm->dc_min );

    ESP_LOGI( PWM_TAG, "Pwm %d object initialized", pwm->tag + 1 );

    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Set pwm Duty Cycle
 * @param pwm: Address of Pwm object
 * @param duty: Duty Cycle from 0 to 1
 * @retval ESP_OK if success - ESP_FAIL
 */
static esp_err_t Pwm_SetDc( pwm_t * pwm, float duty ) {

    /* Check if object is initialized */
    if( !pwm->init_ok ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Object is not initialized!", pwm->tag + 1 );
        esp_restart();
    }

    /* Check range of duty cycle */
    if( ( duty < 0 ) || ( duty > 1 ) ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong Duty Cycle '%f'. Select values from 0 to 1 only", pwm->tag, duty );
        return ESP_FAIL;
    }

    else {

        if( ( duty >= pwm->dc_min ) && ( duty <= pwm->dc_max ) ) {

            /* Set duty cycle */
            if( ledc_set_duty( pwm->pwm_cfg.timer_cfg.speed_mode, pwm->pwm_cfg.channel_cfg.channel, duty * pwm->n ) != ESP_OK ) {

                ESP_LOGE( PWM_TAG, "[ pwm %d ] Failed to set duty dycle", pwm->tag + 1 );
                return ESP_FAIL;
            }

            /* Update duty cycle */
            if( ledc_update_duty( pwm->pwm_cfg.timer_cfg.speed_mode, pwm->pwm_cfg.channel_cfg.channel ) != ESP_OK ) {

                ESP_LOGE( PWM_TAG, "[ pwm %d ] Failed to update duty dycle", pwm->tag + 1 );
                return ESP_FAIL;
            }
        } else {

            ESP_LOGE( PWM_TAG, "[ pwm %d ] Duty Cycle '%f' is out of range. Select values from %f to %f", pwm->tag + 1, duty, pwm->dc_min, pwm->dc_max );
            return ESP_FAIL;
        }

    }

    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Get pwm Duty Cycle
 * @param pwm: Address of Pwm object
 * @retval pwm duty cycle from 0 to 1
 */
static double Pwm_GetDc( pwm_t * pwm ) {

    /* Check object is initialized */
    if( !pwm->init_ok ) {

        ESP_LOGE( PWM_TAG, "Object is not initialized!" );
        esp_restart();
    }
    
    /* Return pwm duty cycle */
    return ledc_get_duty( pwm->pwm_cfg.timer_cfg.speed_mode, pwm->pwm_cfg.channel_cfg.channel ) / pwm->n;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions implementation */

void Pwm( pwm_t* pwm, int tag ) {

    ESP_LOGI( PWM_TAG, "Making an instance of Pwm %d Class...", tag + 1 );

    pwm->dc_min     = 0;              /* Initialize attributes */
    pwm->dc_max     = 0;
    pwm->tag        = tag;
    pwm->n          = 0;
    pwm->resolution = 0;
    pwm->max_count  = 0;
    pwm->min_count  = 0;
    pwm->init_ok    = false;
    
    pwm->init       = Pwm_Init;   /* Pointer assignment to functions */
    pwm->get_pwm_dc = Pwm_GetDc;
    pwm->set_pwm_dc = Pwm_SetDc;

    ESP_LOGI( PWM_TAG, "Instance successfully made" );

}
