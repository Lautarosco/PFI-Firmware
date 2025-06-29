#include <stdio.h>
#include <state_machine.h>
#include <drone.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>

#include "button_helper.h"

const char * STATE_MACHINE_TAG = "STATE_MACHINE";
// sm_state_machine_t state_machine;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief State function
 */
typedef struct state_function {
    
    /* Name of state function */
    const char * name;

    /** @brief Compute the function of a given state
    *   @param drone: Address of Drone object */
    void ( * func )( drone_t * drone );
    
} state_func_row_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @details Private functions definitions
 */

static void StIdleFunc( drone_t * drone ) {

    // printf( "IDLE\r\n" );
}

static void StInitFunc( drone_t * drone ) {

    ESP_ERROR_CHECK( drone->methods.init( drone ) );

    #if WEBSV_TX
        /* Reset button */
        drone->attributes.global_variables.tx_buttons.cross = false;
    #endif

    // tomar ACC_AVG_NUM mediciones de acelerómetro y promediarlas
    float sum = 0;
    #define ACC_AVG_NUM 20.0
    for (int i = 0; i < ACC_AVG_NUM; i++) {
        // leer acelerómetro y calcular roll_acc
        float roll_acc = atan2( drone->attributes.components.bmi.Acc.y, drone->attributes.components.bmi.Acc.z ) * ( 180.0f / M_PI );
        printf("Roll ACC: %.2f\n", roll_acc);
        sum += roll_acc;
        vTaskDelay(pdMS_TO_TICKS(10)); // espera entre muestras
    }
    printf("Sum: %.2f\n", sum);
    drone->attributes.states.roll = sum / ACC_AVG_NUM;
}

static void StWaitingFunc( drone_t * drone ) {

    // printf( "WAITING\r\n" );
}


float filtered_roll = 0.0f;

static void toggle_motor(drone_t* drone, int pwm_num);
static void increase_motor_duty(drone_t* drone, int pwm_num);

static void toggle_motor(drone_t* drone, int pwm_num) {

    float current_pwm_dc = drone->attributes.components.pwm[ pwm_num ].get_pwm_dc(&drone->attributes.components.pwm[ pwm_num ]);
    printf("Current PWM DC for motor %d: %.2f\n", pwm_num, current_pwm_dc);

    float dc_min = drone->attributes.components.pwm[pwm_num].dc_min;
    float dc_target = dc_min*drone->attributes.config.mma_out_limits.lower; 
    float avg = (dc_min + dc_target) / 2.0;

    if (current_pwm_dc > avg) {

        drone->attributes.components.pwm[ pwm_num ].set_pwm_dc(
            &drone->attributes.components.pwm[ pwm_num ],
            dc_min  // Apagado
        );
        printf("Motor %d OFF with duty cycle: %f \n", pwm_num, dc_min);

    } else {
        // float dc_target = dc_min + ((dc_max - dc_min) / 2); 
        printf("dc_min: %f\n", dc_min);
        printf("dc_target: %f\n", dc_target);
        printf("avg: %f\n", avg);
        printf("lower_limit: %f\n", drone->attributes.components.mma.limit.lower);
        
        drone->attributes.components.pwm[ pwm_num ].set_pwm_dc(
            &drone->attributes.components.pwm[ pwm_num ],
            dc_target  // Velocidad minima
        );
        printf("Motor %d ON with duty cycle: %f \n", pwm_num, dc_target);
    }
}

// Increase motor duty cycle to next value in a predefined array
static void increase_motor_duty(drone_t* drone, int pwm_num) {
    // Define the array of duty cycle values (example values, adjust as needed)
    static const float duty_steps[] = {0.0f, 0.15f, 0.25f, 0.35f, 0.45f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f, 1.0f};
    static const int num_steps = sizeof(duty_steps) / sizeof(duty_steps[0]);

    float dc_min = drone->attributes.components.pwm[pwm_num].dc_min;
    float dc_max = drone->attributes.components.pwm[pwm_num].dc_max;

    float current_pwm_dc = drone->attributes.components.pwm[pwm_num].get_pwm_dc(&drone->attributes.components.pwm[pwm_num]);
    // Normalize current duty cycle to [0,1] range
    float norm = (current_pwm_dc - dc_min) / (dc_max - dc_min);

    int idx = 0;
    // Find the current step index
    for (int i = 0; i < num_steps; ++i) {
        if (norm < duty_steps[i]) {
            idx = i;
            break;
        }
        idx = i;
    }
    // Move to next step, but don't exceed array bounds
    if (idx < num_steps - 1) {
        idx++;
    }
    float new_norm = duty_steps[idx];
    float new_dc = dc_min + new_norm * (dc_max - dc_min);

    drone->attributes.components.pwm[pwm_num].set_pwm_dc(
        &drone->attributes.components.pwm[pwm_num],
        new_dc
    );
    printf("Motor %d increased to duty cycle: %f (step %d)\n", pwm_num, new_dc, idx);
}

static void StVibrationCheck(drone_t* drone) {
    float alpha = 0.1f; // Smoothing factor (0 < alpha < 1)

    static bool initialized = false;
    static float acc_av_vector = 0;
    static float vibration_total_result = 0;
    static int vibration_counter = 0;
    
    if (pressed(drone, EV_SQUARE) && initialized) {
        
        /* Turn off motors */
        for (int pwm_num = 0; pwm_num < 4; pwm_num++) {
            drone->attributes.components.pwm[pwm_num].set_pwm_dc(
                &drone->attributes.components.pwm[pwm_num],
                drone->attributes.components.pwm[ pwm_num ].dc_min);
        }
        initialized = false;  // Reset for next entry
        printf("Exiting vibration check mode\n");
        drone->attributes.request_state_transition = true;
        drone->attributes.requested_transition_event = EV_SQUARE;
        return;
    }
    
    // Initialize with first reading
    if (!initialized) {
        acc_av_vector = sqrt(
            drone->attributes.components.bmi.Acc.x * drone->attributes.components.bmi.Acc.x +
            drone->attributes.components.bmi.Acc.y * drone->attributes.components.bmi.Acc.y +
            drone->attributes.components.bmi.Acc.z * drone->attributes.components.bmi.Acc.z
        );

        acc_av_vector = 0;
        vibration_total_result = 0;
        vibration_counter = 0;
        initialized = true;

    }

    // Motor control
    if(pressed(drone, EV_UP)) toggle_motor(drone, 0);
    else if(pressed(drone, EV_DOWN)) toggle_motor(drone, 1);
    else if(pressed(drone, EV_LEFT)) toggle_motor(drone, 2);
    else if(pressed(drone, EV_RIGHT)) toggle_motor(drone, 3);

    if (pressed(drone, EV_R1)) increase_motor_duty(drone, 0);
    
    // Vibration calculation
    float current_acc_vector = sqrtf(
        drone->attributes.components.bmi.Acc.x * drone->attributes.components.bmi.Acc.x +
        drone->attributes.components.bmi.Acc.y * drone->attributes.components.bmi.Acc.y +
        drone->attributes.components.bmi.Acc.z * drone->attributes.components.bmi.Acc.z
    );
    
    acc_av_vector = (alpha) * current_acc_vector + (1-alpha) * acc_av_vector;
    vibration_total_result += fabsf(current_acc_vector - acc_av_vector);
    if (++vibration_counter >= 20) {
        ESP_LOGI("VIBRATION", "Vibration: %.2f", vibration_total_result / 20);
        vibration_total_result = 0;
        vibration_counter = 0;
    }
}


float getYawError(float current_yaw, float target_yaw) {
    float error = target_yaw - current_yaw;
    if (error > 180.0f) return error - 360.0f;
    if (error < -180.0f) return error + 360.0f;
    return error;
}

static void StControlFunc( drone_t * drone ) {


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

static void StCalibrationFunc( drone_t * drone ) {
    
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

static void StResetFunc( drone_t * drone ) {

    for (int i = 0; i < ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++ ) {
        /* Set all pwm duty cycle to minimum */
        drone->attributes.components.pwm[ i ].set_pwm_dc( &drone->attributes.components.pwm[ i ], drone->attributes.components.pwm[ i ].dc_min );
    }

    esp_restart();
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static state_func_row_t state_function_array[] = {

    { .name = "ST_IDLE",                  .func = &StIdleFunc },
    { .name = "ST_INIT",                  .func = &StInitFunc },
    { .name = "ST_WAITING",               .func = &StWaitingFunc },
    { .name = "ST_CALIBRATION",           .func = &StCalibrationFunc },
    { .name = "ST_CONTROL",               .func = &StControlFunc },
    { .name = "ST_PROPELLER_CALIBRATION", .func = &StVibrationCheck },
    { .name = "ST_RESET",                 .func = &StResetFunc },

};


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Complete description of a state
 */
typedef struct state_transition_row {

    /* Current state */
    sm_state_t curr_state;
    
    /* Occured event */
    sm_event_t event;
    
    /* State to go next */
    sm_state_t next_state;

} state_trans_row_t;


/**
 * @brief State transition matrix
 */
static const state_trans_row_t state_trans_matrix[] = {

    /* From IDLE to ... */
    { .curr_state = ST_IDLE,                  .event = EV_ANY,      .next_state = ST_IDLE },
    { .curr_state = ST_IDLE,                  .event = EV_CROSS,    .next_state = ST_INIT },
    { .curr_state = ST_IDLE,                  .event = EV_PS,       .next_state = ST_RESET },

    /* From INIT to ... */
    { .curr_state = ST_INIT,                  .event = EV_ANY,      .next_state = ST_WAITING },
    { .curr_state = ST_INIT,                  .event = EV_PS,       .next_state = ST_RESET },

    /* From WAITING to ... */
    { .curr_state = ST_WAITING,               .event = EV_ANY,      .next_state = ST_WAITING },
    { .curr_state = ST_WAITING,               .event = EV_TRIANGLE, .next_state = ST_CALIBRATION },
    { .curr_state = ST_WAITING,               .event = EV_CIRCLE,   .next_state = ST_CONTROL },
    { .curr_state = ST_WAITING,               .event = EV_SQUARE,    .next_state = ST_PROPELLER_CALIBRATION },
    { .curr_state = ST_WAITING,               .event = EV_PS,       .next_state = ST_RESET },

    /* From CALIBRATION to ... */
    { .curr_state = ST_CALIBRATION,           .event = EV_ANY,      .next_state = ST_WAITING },
    { .curr_state = ST_CALIBRATION,           .event = EV_PS,       .next_state = ST_RESET },


    /* From CONTROL to ... */
    { .curr_state = ST_CONTROL,               .event = EV_ANY,      .next_state = ST_CONTROL },
    { .curr_state = ST_CONTROL,               .event = EV_PS,       .next_state = ST_RESET },

    /* From PROPELLER CALIBRATION to ... */
    { .curr_state = ST_PROPELLER_CALIBRATION, .event = EV_ANY,      .next_state = ST_PROPELLER_CALIBRATION },
    { .curr_state = ST_PROPELLER_CALIBRATION, .event = EV_SQUARE,   .next_state = ST_WAITING },
    { .curr_state = ST_PROPELLER_CALIBRATION, .event = EV_PS,       .next_state = ST_RESET },
};


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @details Public functions definitions
 */

void StateMachine_Init( sm_state_machine_t * state_machine ) {

    /* Default state */
    state_machine->curr_state = ST_IDLE;
}

void StateMachine_RunIteration(drone_t * drone) {

    sm_state_t prev_state = drone->attributes.state_machine.curr_state;

    /* Loop through the entire transition matrix to match actual state and occurred event */
    for( int i = 0; i < sizeof( state_trans_matrix ) / sizeof( state_trans_matrix[ 0 ] ); i++ ) {

        /* If matched actual state */
        if( state_trans_matrix[ i ].curr_state == drone->attributes.state_machine.curr_state ) {

            /* If matched occurred event */
            if( state_trans_matrix[ i ].event == drone->attributes.state_machine.event ) {

                /* Go to the next state */
                drone->attributes.state_machine.curr_state = state_trans_matrix[ i ].next_state;

                /* Log transition if state changed */
                if (drone->attributes.state_machine.curr_state != prev_state) {
                    ESP_LOGI(STATE_MACHINE_TAG, "Transition: %s -> %s on event %s",
                        StateMachine_GetStateName(prev_state),
                        StateMachine_GetStateName(drone->attributes.state_machine.curr_state),
                        StateMachine_GetEventName(drone->attributes.state_machine.event));
                }

                /* Run new actual state respective function */
                state_function_array[ drone->attributes.state_machine.curr_state ].func( drone );
                break;
            }
        }
    }

}

const char * StateMachine_GetStateName( sm_state_t state ) {

    return state_function_array[ state ].name;
}

const char * StateMachine_GetEventName( sm_event_t event ) {

    switch ( event ) {

        case EV_CROSS:
            return "EV_CROSS";
            break;

        case EV_TRIANGLE:
            return "EV_TRIANGLE";
            break;

        case EV_CIRCLE:
            return "EV_CIRCLE";
            break;

        case EV_SQUARE:
            return "EV_SQUARE";
            break;

        case EV_UP:
            return "EV_UP";
            break;

        case EV_DOWN:
            return "EV_DOWN";
            break;

        case EV_LEFT:
            return "EV_LEFT";
            break;

        case EV_RIGHT:
            return "EV_RIGHT";
            break;

        case EV_R1:
            return "EV_R1";
            break;

        case EV_R2:
            return "EV_R2";
            break;

        case EV_L1:
            return "EV_L1";
            break;

        case EV_L2:
            return "EV_L2";
            break;

        case EV_ANY:
            return "EV_ANY";
            break;

        case EV_PS:
            return "EV_PS";
            break;

        default:
            return "EVENT NOT FOUND";
            break;
    }
}
