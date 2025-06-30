#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "tasks.h"

#include "drone.h"
#include "string.h"

void vTaskprint( void * drone_ ) {

    drone_t* drone = ( drone_t * ) drone_;

    /**
     * @brief Variable format: <printer:var,%.2f\n>
     * @details Used for sending variables such as drone states, pid values, etc.
     * @example i.e, <printer:roll,%f>  This will send roll values to plotter app
     * 
     * 
     * @brief Static format: <static:var_name/var_attr,%.2f|\n>
     * @details Used for sending static variables
     * @example i.e, <static:roll/P,%f>  This will send proportional action of roll pid to plotter app
     */
    static char buf[1024];

    while( 1 ) {

        if( drone->attributes.init_ok) {

            // Dynamic
            snprintf(buf, sizeof(buf),
                "printer:roll,%.2f|roll_d,%.2f|roll_sp,%.2f|roll_d_sp,%.2f|"
                "pitch,%.2f|pitch_d,%.2f|pitch_sp,%.2f|pitch_d_sp,%.2f|"
                "yaw,%.2f|yaw_d,%.2f|yaw_sp,%.2f|yaw_d_sp,%.2f|"
                "height,%.2f|height_sp,%.2f|"
                "dc1,%.2f|dc2,%.2f|dc3,%.2f|dc4,%.2f|"
                "gyro_x,%.2f|gyro_y,%.2f|gyro_z,%.2f"
                "\n"  // end of dynamic values
                "static:roll/P,%.2f|roll/I,%.2f|roll/D,%.2f|roll_d/P,%.2f|roll_d/I,%.2f|roll_d/D,%.2f|"
                "pitch/P,%.2f|pitch/I,%.2f|pitch/D,%.2f|pitch_d/P,%.2f|pitch_d/I,%.2f|pitch_d/D,%.2f|"
                "yaw/P,%.2f|yaw/I,%.2f|yaw/D,%.2f|yaw_d/P,%.2f|yaw_d/I,%.2f|yaw_d/D,%.2f|"
                "ema_roll,%.2f|ema_pitch,%.2f|ema_yaw,%.2f|"
                "state,%s\n",

                // dynamic state
                drone->attributes.states.roll,
                drone->attributes.states.roll_dot,
                drone->attributes.sp.roll,
                drone->attributes.sp.roll_dot,
                drone->attributes.states.pitch,
                drone->attributes.states.pitch_dot,
                drone->attributes.sp.pitch,
                drone->attributes.sp.pitch_dot,
                drone->attributes.states.yaw,
                drone->attributes.states.yaw_dot,
                drone->attributes.sp.yaw,
                drone->attributes.sp.yaw_dot,
                drone->attributes.states.z,
                drone->attributes.sp.z,
                drone->attributes.components.pwm[0].get_pwm_dc(&drone->attributes.components.pwm[0])*1000,
                drone->attributes.components.pwm[1].get_pwm_dc(&drone->attributes.components.pwm[1])*1000,
                drone->attributes.components.pwm[2].get_pwm_dc(&drone->attributes.components.pwm[2])*1000,
                drone->attributes.components.pwm[3].get_pwm_dc(&drone->attributes.components.pwm[3])*1000,
                drone->attributes.components.bmi.Gyro.x,
                drone->attributes.components.bmi.Gyro.y,
                drone->attributes.components.bmi.Gyro.z,

                // roll gains
                drone->attributes.components.controllers[ROLL].gain.kp,
                drone->attributes.components.controllers[ROLL].gain.ki,
                drone->attributes.components.controllers[ROLL].gain.kd,
                drone->attributes.components.controllers[ROLL_D].gain.kp,
                drone->attributes.components.controllers[ROLL_D].gain.ki,
                drone->attributes.components.controllers[ROLL_D].gain.kd,

                // pitch gains
                drone->attributes.components.controllers[PITCH].gain.kp,
                drone->attributes.components.controllers[PITCH].gain.ki,
                drone->attributes.components.controllers[PITCH].gain.kd,
                drone->attributes.components.controllers[PITCH_D].gain.kp,
                drone->attributes.components.controllers[PITCH_D].gain.ki,
                drone->attributes.components.controllers[PITCH_D].gain.kd,

                // yaw gains
                drone->attributes.components.controllers[YAW].gain.kp,
                drone->attributes.components.controllers[YAW].gain.ki,
                drone->attributes.components.controllers[YAW].gain.kd,
                drone->attributes.components.controllers[YAW_D].gain.kp,
                drone->attributes.components.controllers[YAW_D].gain.ki,
                drone->attributes.components.controllers[YAW_D].gain.kd,

                // ema values
                drone->attributes.config.IIR_coeff_roll_dot,
                drone->attributes.config.IIR_coeff_pitch_dot,
                drone->attributes.config.IIR_coeff_yaw_dot,
            
                // state machine current state
                StateMachine_GetStateName(drone->attributes.state_machine.curr_state)
            );
            printf("%s", buf);
        
        }
        fflush(stdout);  // check

        vTaskDelay( pdMS_TO_TICKS( 50 ) );
    }
}
