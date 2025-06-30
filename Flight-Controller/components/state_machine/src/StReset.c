#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "state_machine.h"
#include "drone.h"
#include "button_helper.h"


void StResetFunc( drone_t * drone ) {

    for (int i = 0; i < ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++ ) {
        /* Set all pwm duty cycle to minimum */
        drone->attributes.components.pwm[ i ].set_pwm_dc( &drone->attributes.components.pwm[ i ], drone->attributes.components.pwm[ i ].dc_min );
    }

    esp_restart();
}