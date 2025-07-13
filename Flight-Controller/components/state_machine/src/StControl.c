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

    /* ROLL - Cascaded PID */

    float CRoll = drone->attributes.components.controllers[ ROLL ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL ],
        drone->attributes.states.roll,
        drone->attributes.sp.roll
    );
    
    drone->attributes.sp.roll_dot = CRoll;

    float CRolld = drone->attributes.components.controllers[ ROLL_D ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL_D ],
        drone->attributes.states.roll_dot,
        drone->attributes.sp.roll_dot
    );
    
    /* PITCH - Cascaded PID */

    float CPitch = drone->attributes.components.controllers[ PITCH ].pidUpdate(
        &drone->attributes.components.controllers[ PITCH ],
        drone->attributes.states.pitch,
        drone->attributes.sp.pitch
    );

    drone->attributes.sp.pitch_dot = CPitch;

    float CPitchd = drone->attributes.components.controllers[ PITCH_D ].pidUpdate(
        &drone->attributes.components.controllers[ PITCH_D ],
        drone->attributes.states.pitch_dot,
        drone->attributes.sp.pitch_dot
    );

    /* Z - Cascaded PID */

    float CZ = drone->attributes.components.controllers[ Z ].pidUpdate(
        &drone->attributes.components.controllers[ Z ],
        drone->attributes.states.z,
        drone->attributes.sp.z
    );

    drone->attributes.sp.z_dot = CZ;

    float CZd = 0*drone->attributes.components.controllers[ Z_D ].pidUpdate(  // multiplied by 0 to disable Z control
        &drone->attributes.components.controllers[ Z_D ],
        drone->attributes.states.z_dot,
        drone->attributes.sp.z_dot
    );

    /* Update MMA inputs with PID outputs */
    drone->attributes.components.mma.input[ C_ROLL ] = CRolld;
    drone->attributes.components.mma.input[ C_PITCH ] = CPitchd;
    drone->attributes.components.mma.input[ C_Z ] = CZd; // Z control is not implemented yet

    float range = drone->attributes.components.pwm[ 0 ].dc_max - drone->attributes.components.pwm[ 0 ].dc_min;

    /* Compute MMA algorithm */
    drone->attributes.components.mma.compute(
        &drone->attributes.components.mma,
        drone->attributes.components.pwm[ 0 ].dc_min + 0.3*range,
        drone->attributes.components.pwm[ 0 ].dc_min + 0.8*range
    );

    /* Update all pwm duty cycle */
    for(int i = 0; i < ( ( sizeof( drone->attributes.components.mma.output ) ) / ( sizeof( drone->attributes.components.mma.output[ 0 ] ) ) ); i++) {
        
        drone->attributes.components.pwm[ i ].set_pwm_dc(
            &drone->attributes.components.pwm[ i ],
            drone->attributes.components.mma.output[ i ]
        );
    }
}
