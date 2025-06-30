#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "state_machine.h"
#include "button_helper.h"


void StCalibrationFunc( drone_t * drone ) {
    
    /* Calibration routine for Hobbywing Skywalker ESC's */
    ESP_LOGW( STATE_MACHINE_TAG, "Starting calibration routine..."  );

    /* 1. Move throttle to maximum position */

    for( int i = 0; i < ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++ ) {

        drone->attributes.components.pwm[ i ].set_pwm_dc( &drone->attributes.components.pwm[ i ], drone->attributes.components.pwm[ i ].dc_max );
    }

    /* 2. Wait until ESC's are connected ( user pressed X button )  */

    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC are connected"  );
    while( 1 ) {

        if( drone->attributes.global_variables.tx_buttons.cross ) {

            #if WEBSV_TX
                /* Reset button */
                drone->attributes.global_variables.tx_buttons.cross = false;
            #endif

            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* 3. Wait until ESC's have latched maximum value ( user pressed X button ) */

    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC have latched maximum value"  );
    while( 1 ) {

        if( drone->attributes.global_variables.tx_buttons.cross ) {

            #if WEBSV_TX
                /* Reset button */
                drone->attributes.global_variables.tx_buttons.cross = false;
            #endif

            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* 4. Move throttle to minimum position */

    for( int i = 0; i < ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++ ) {

        drone->attributes.components.pwm[ i ].set_pwm_dc( &drone->attributes.components.pwm[ i ], drone->attributes.components.pwm[ i ].dc_min );
    }

    /* 5. Wait until ESC's have latched minimum value ( user pressed X button ) */
    
    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC have latched minimum value"  );
    while( 1 ) {

        if( drone->attributes.global_variables.tx_buttons.cross ) {
        
            #if WEBSV_TX
                /* Reset button */
                drone->attributes.global_variables.tx_buttons.cross = false;
            #endif

            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* End of calibration routine */
    ESP_LOGW( STATE_MACHINE_TAG, "Calibration completed."  );

    #if WEBSV_TX
        /* Reset button */
        drone->attributes.global_variables.tx_buttons.triangle = false;  /* TESTING */
    #endif
}