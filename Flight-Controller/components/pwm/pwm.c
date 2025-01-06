#include <stdio.h>
#include <pwm.h>
#include <esp_log.h>
#include <esp_system.h>
#include <math.h>


static const char * PWM_TAG = "PWM";

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize pwm signal of Brushless DC motor
 * @param obj: Address of Pwm object
 * @param PwmConfigs: Pwm configs
 * @retval esp_err_t
 */
static esp_err_t Pwm_Init( pwm_t * obj, pwm_cfg_t PwmConfigs ) {

    ESP_LOGI( PWM_TAG, "Initializing Pwm %d object...", obj->tag + 1 );
    
    obj->init_ok = true;
    obj->pwm_cfg = PwmConfigs;

    if( ledc_timer_config( &( obj->pwm_cfg.timer_cfg ) ) != ESP_OK ) {
        
        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong timer settings!", obj->tag );
        return ESP_FAIL;
    }

    if( ledc_channel_config( &( obj->pwm_cfg.channel_cfg ) ) != ESP_OK ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong channel settings!", obj->tag );
        return ESP_FAIL;
    }

    obj->n          = pow( 2, obj->pwm_cfg.timer_cfg.duty_resolution );
    obj->resolution = 1 / obj->n;
    obj->dc_min     = ( ( obj->pwm_cfg.Ton_min / 1000.0f ) * obj->pwm_cfg.timer_cfg.freq_hz );
    obj->dc_max     = ( ( obj->pwm_cfg.Ton_max / 1000.0f ) * obj->pwm_cfg.timer_cfg.freq_hz );
    obj->min_count  = obj->dc_min * obj->n;
    obj->max_count  = obj->dc_max * obj->n;

    ESP_LOGI( PWM_TAG, "Pwm %d object initialized", obj->tag + 1 );

    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Set pwm Duty Cycle
 * @param obj: Address of Pwm object
 * @param duty: Duty Cycle from 0 to 1
 * @retval esp_err_t
 */
static esp_err_t Pwm_SetDc( pwm_t * obj, float duty ) {

    if( !obj->init_ok ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Object is not initialized!", obj->tag + 1 );
        esp_restart();
    }

    if( ( duty < 0 ) || ( duty > 1 ) ) {

        ESP_LOGE( PWM_TAG, "[ pwm %d ] Wrong Duty Cycle '%f'. Select values from 0 to 1 only", obj->tag, duty );
        return ESP_FAIL;
    }
    else {

        if( ( duty >= obj->dc_min ) && ( duty <= obj->dc_max ) ) {

            if( ledc_set_duty( obj->pwm_cfg.timer_cfg.speed_mode, obj->pwm_cfg.channel_cfg.channel, duty * obj->n ) != ESP_OK ) {

                ESP_LOGE( PWM_TAG, "[ pwm %d ] Failed to set duty dycle", obj->tag + 1 );
                return ESP_FAIL;
            }

            if( ledc_update_duty( obj->pwm_cfg.timer_cfg.speed_mode, obj->pwm_cfg.channel_cfg.channel ) != ESP_OK ) {

                ESP_LOGE( PWM_TAG, "[ pwm %d ] Failed to update duty dycle", obj->tag + 1 );
                return ESP_FAIL;
            }
        }
    }

    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Get pwm Duty Cycle
 * @param obj: Address of Pwm object
 * @retval uint32_t
 */
static double Pwm_GetDc( pwm_t * obj ) {

    if( !obj->init_ok ) {

        ESP_LOGE( PWM_TAG, "Object is not initialized!" );
        esp_restart();
    }
    
    return ledc_get_duty( obj->pwm_cfg.timer_cfg.speed_mode, obj->pwm_cfg.channel_cfg.channel ) / obj->n;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions implementation */

pwm_t * Pwm( int tag ) {

    ESP_LOGI( PWM_TAG, "Making an instance of Pwm %d Class...", tag + 1 );

    pwm_t * pwm = ( pwm_t * ) malloc( sizeof( pwm_t ) );

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

    return pwm;
}
