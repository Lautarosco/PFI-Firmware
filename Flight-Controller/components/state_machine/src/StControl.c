#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "state_machine.h"
#include "drone.h"
#include "button_helper.h"


float getYawError(float current_yaw, float target_yaw) {
    float error = target_yaw - current_yaw;
    if (error > 180.0f) return error - 360.0f;
    if (error < -180.0f) return error + 360.0f;
    return error;
}

void StControlFunc( drone_t * drone ) {

    /* Compute PID algorithm for all states */

    /* ROLL - Cascaded PID*/

    // float alpha_ema = 2/(drone->attributes.global_variables.ema_filter_roll+1);
    // filtered_roll = alpha_ema*drone->attributes.states.roll + (1-alpha_ema)*filtered_roll;

    float CRoll = drone->attributes.components.controllers[ ROLL ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL ],
        drone->attributes.states.roll,
        drone->attributes.sp.roll
    );
    
    // float sine = __sin( 80.0f, 2*M_PI*(1 / 1.0f), 10 );

    drone->attributes.sp.roll_dot = CRoll;

    float CRolld = drone->attributes.components.controllers[ ROLL_D ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL_D ],
        drone->attributes.states.roll_dot,
        drone->attributes.sp.roll_dot
    );
    
    /* Update MMA inputs with PID outputs */
    drone->attributes.components.mma.input[ C_ROLL ] = CRolld;

    /* Compute MMA algorithm */
    drone->attributes.components.mma.compute(
        &drone->attributes.components.mma,
        drone->attributes.components.pwm[ 0 ].dc_min * drone->attributes.config.mma_out_limits.lower,
        drone->attributes.components.pwm[ 0 ].dc_max * drone->attributes.config.mma_out_limits.upper
    );

    /* Update all pwm duty cycle */
    for(int i = 0; i < ( ( sizeof( drone->attributes.components.mma.output ) ) / ( sizeof( drone->attributes.components.mma.output[ 0 ] ) ) ); i++) {
        
        drone->attributes.components.pwm[ i ].set_pwm_dc(
            &drone->attributes.components.pwm[ i ],
            drone->attributes.components.mma.output[ i ]
        );
    }
}
