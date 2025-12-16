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

float prev_error = 0;
void StControlFunc( drone_t * drone ) {


    // Freeze states and setpoints
    drone_states_t local_setpoint = drone->attributes.sp;
    drone_states_t local_state = drone->attributes.states;
    /* Compute PID algorithm for all states */

    /* ROLL - Cascaded PID */


    float CRoll = drone->attributes.components.controllers[ ROLL ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL ],
        local_state.roll,
        local_setpoint.roll
    );
    
    if (drone->attributes.global_variables.misc_floats[0] <= 0 && drone->attributes.control_mode == CONTROL_MODE_ANGLE) {
        drone->attributes.sp.roll_dot = CRoll;
    }

    float CRolld = drone->attributes.components.controllers[ ROLL_D ].pidUpdate(
        &drone->attributes.components.controllers[ ROLL_D ],
        local_state.roll_dot,
        local_setpoint.roll_dot
    );

    float current_error = local_setpoint.roll_dot - local_state.roll_dot;
    drone->attributes.global_variables.misc_floats[9] = drone->attributes.components.controllers[ROLL_D].gain.kd*(current_error - prev_error);
    prev_error = current_error;

    /* PITCH - Cascaded PID */

    float CPitch = drone->attributes.components.controllers[ PITCH ].pidUpdate(
        &drone->attributes.components.controllers[ PITCH ],
        local_state.pitch,
        local_setpoint.pitch
    );

    drone->attributes.sp.pitch_dot = CPitch;

    float CPitchd = drone->attributes.components.controllers[ PITCH_D ].pidUpdate(
        &drone->attributes.components.controllers[ PITCH_D ],
        local_state.pitch_dot,
        local_setpoint.pitch_dot
    );

    /* Z - Cascaded PID */

    float CZ = drone->attributes.components.controllers[ Z ].pidUpdate(
        &drone->attributes.components.controllers[ Z ],
        local_state.z,
        local_setpoint.z
    );

    drone->attributes.sp.z_dot = CZ;

    float CZd = 0*drone->attributes.components.controllers[ Z_D ].pidUpdate(  // multiplied by 0 to disable Z control
        &drone->attributes.components.controllers[ Z_D ],
        local_state.z_dot,
        local_setpoint.z_dot
    );

    /* Update MMA inputs with PID outputs */
    drone->attributes.components.mma.input[ C_ROLL ] = CRolld;
    drone->attributes.components.mma.input[ C_PITCH ] = CPitchd*0; // Ignore pitch control for now
    drone->attributes.components.mma.input[ C_Z ] = drone->attributes.sp.z; // Z control is not implemented yet

    float range = drone->attributes.components.pwm[ 0 ].dc_max - drone->attributes.components.pwm[ 0 ].dc_min;
    float min = drone->attributes.components.pwm[ 0 ].dc_min + 0.2*range;
    float max = drone->attributes.components.pwm[ 0 ].dc_min + 0.8*range;
    
    drone->attributes.global_variables.misc_floats[6] = min*1000;
    drone->attributes.global_variables.misc_floats[7] = max*1000;
    
    /* Compute MMA algorithm */
    drone->attributes.components.mma.compute(
        &drone->attributes.components.mma,
        min,
        max
    );

    /* Update all pwm duty cycle */
    static uint32_t last_pwm_update = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (current_time - last_pwm_update >= 20) {
        for(int i = 0; i < ( ( sizeof( drone->attributes.components.mma.output ) ) / ( sizeof( drone->attributes.components.mma.output[ 0 ] ) ) ); i++) {
            
            drone->attributes.components.pwm[ i ].set_pwm_dc(
                &drone->attributes.components.pwm[ i ],
                drone->attributes.components.mma.output[ i ]
            );
        }
        last_pwm_update = current_time;
    }
}
