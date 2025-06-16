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

    /** @brief Compute the function of a given state @param obj: Address of Drone object */
    void ( * func )( drone_t * obj );
    
} state_func_row_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @details Private functions definitions
 */

static void StIdleFunc( drone_t * obj ) {

    // printf( "IDLE\r\n" );
}

static void StInitFunc( drone_t * obj ) {

    ESP_ERROR_CHECK( obj->methods.init( obj ) );

    #if WEBSV_TX
        /* Reset button */
        obj->attributes.global_variables.tx_buttons.cross = false;
    #endif

    // tomar ACC_AVG_NUM mediciones de acelerómetro y promediarlas
    float sum = 0;
    #define ACC_AVG_NUM 20.0
    for (int i = 0; i < ACC_AVG_NUM; i++) {
        // leer acelerómetro y calcular roll_acc
        float roll_acc = atan2( obj->attributes.components.bmi.Acc.y, obj->attributes.components.bmi.Acc.z ) * ( 180.0f / M_PI );
        printf("Roll ACC: %.2f\n", roll_acc);
        sum += roll_acc;
        vTaskDelay(pdMS_TO_TICKS(10)); // espera entre muestras
    }
    printf("Sum: %.2f\n", sum);
    obj->attributes.states.roll = sum / ACC_AVG_NUM;
}

static void StWaitingFunc( drone_t * obj ) {

    // printf( "WAITING\r\n" );
}


float filtered_roll = 0.0f;

#define AVG (drone->attributes.components.pwm[ pwm_num ]->dc_max+drone->attributes.components.pwm[ pwm_num ]->dc_min)/2  // buscar una solucion mas prolija

static void toggle_motor(drone_t* drone, int pwm_num);
static void toggle_motor(drone_t* drone, int pwm_num) {

    float current_pwm_dc = drone->attributes.components.pwm[ pwm_num ]->get_pwm_dc(drone->attributes.components.pwm[ pwm_num ]);
    printf("Current PWM DC for motor %d: %.2f\n", pwm_num, current_pwm_dc);

    if (current_pwm_dc > AVG) {

        drone->attributes.components.pwm[ pwm_num ]->set_pwm_dc(
            drone->attributes.components.pwm[ pwm_num ],
            drone->attributes.components.pwm[ pwm_num ]->dc_min
        );
        printf("Motor %d OFF\n", pwm_num);

    } else {

        drone->attributes.components.pwm[ pwm_num ]->set_pwm_dc(
            drone->attributes.components.pwm[ pwm_num ],
            drone->attributes.components.pwm[ pwm_num ]->dc_max*0  // TODO: no hacerlo tan grande
        );
        printf("Motor %d ON\n", pwm_num);


    }

}

static void StControlVibrationCheck(drone_t* drone) {
    float alpha = 0.1f; // Smoothing factor (0 < alpha < 1)

    static bool initialized = false;
    static float acc_av_vector = 0;
    static float vibration_total_result = 0;
    static int vibration_counter = 0;
    
    if (pressed(drone, EV_SQUARE) && initialized) {
        
        /* Turn off motors */
        for (int pwm_num = 0; pwm_num < 4; pwm_num++) {
            drone->attributes.components.pwm[pwm_num]->set_pwm_dc(
                drone->attributes.components.pwm[pwm_num],
                drone->attributes.components.pwm[ pwm_num ]->dc_min);
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

static void StControlFunc( drone_t * obj ) {


    /* Compute PID algorithm for all states */

    /* ROLL - Cascaded PID*/

    // float alpha_ema = 2/(obj->attributes.global_variables.ema_filter_roll+1);
    // filtered_roll = alpha_ema*obj->attributes.states.roll + (1-alpha_ema)*filtered_roll;

    float CRoll = obj->attributes.components.controllers[ ROLL ]->pidUpdate(
        obj->attributes.components.controllers[ ROLL ],
        obj->attributes.states.roll,
        obj->attributes.sp.roll
    );
    
    // float sine = __sin( 80.0f, 2*M_PI*(1 / 1.0f), 10 );

    obj->attributes.sp.roll_dot = CRoll;

    float CRolld = obj->attributes.components.controllers[ ROLL_D ]->pidUpdate(
        obj->attributes.components.controllers[ ROLL_D ],
        obj->attributes.states.roll_dot,
        obj->attributes.sp.roll_dot
    );
    
    /* Update MMA inputs with PID outputs */
    obj->attributes.components.mma->input[ C_ROLL ] = CRolld;

    /* Compute MMA algorithm */
    obj->attributes.components.mma->compute(
        obj->attributes.components.mma,
        obj->attributes.components.pwm[ 0 ]->dc_min * obj->attributes.config.mma_out_limits.lower,
        obj->attributes.components.pwm[ 0 ]->dc_max * obj->attributes.config.mma_out_limits.upper
    );

    /* Update all pwm duty cycle */
    for(int i = 0; i < ( ( sizeof( obj->attributes.components.mma->output ) ) / ( sizeof( obj->attributes.components.mma->output[ 0 ] ) ) ); i++) {
        
        obj->attributes.components.pwm[ i ]->set_pwm_dc(
            obj->attributes.components.pwm[ i ],
            obj->attributes.components.mma->output[ i ]
        );
    }
}

static void StCalibrationFunc( drone_t * obj ) {
    
    /* Calibration routine for Hobbywing Skywalker ESC's */
    ESP_LOGW( STATE_MACHINE_TAG, "Starting calibration routine..."  );

    /* 1. Move throttle to maximum position */

    for( int i = 0; i < ( sizeof( obj->attributes.components.pwm ) ) / ( sizeof( obj->attributes.components.pwm[ 0 ] ) ); i++ ) {

        obj->attributes.components.pwm[ i ]->set_pwm_dc( obj->attributes.components.pwm[ i ], obj->attributes.components.pwm[ i ]->dc_max );
    }

    /* 2. Wait until ESC's are connected ( user pressed X button )  */

    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC are connected"  );
    while( 1 ) {

        if( obj->attributes.global_variables.tx_buttons.cross ) {

            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons.cross = false;
            #endif

            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* 3. Wait until ESC's have latched maximum value ( user pressed X button ) */

    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC have latched maximum value"  );
    while( 1 ) {

        if( obj->attributes.global_variables.tx_buttons.cross ) {

            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons.cross = false;
            #endif

            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* 4. Move throttle to minimum position */

    for( int i = 0; i < ( sizeof( obj->attributes.components.pwm ) ) / ( sizeof( obj->attributes.components.pwm[ 0 ] ) ); i++ ) {

        obj->attributes.components.pwm[ i ]->set_pwm_dc( obj->attributes.components.pwm[ i ], obj->attributes.components.pwm[ i ]->dc_min );
    }

    /* 5. Wait until ESC's have latched minimum value ( user pressed X button ) */
    
    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC have latched minimum value"  );
    while( 1 ) {

        if( obj->attributes.global_variables.tx_buttons.cross ) {
        
            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons.cross = false;
            #endif

            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* End of calibration routine */
    ESP_LOGW( STATE_MACHINE_TAG, "Calibration completed."  );

    #if WEBSV_TX
        /* Reset button */
        obj->attributes.global_variables.tx_buttons.triangle = false;  /* TESTING */
    #endif
}

static void StResetFunc( drone_t * drone ) {

    for (int i = 0; i < ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++ ) {
        /* Set all pwm duty cycle to minimum */
        drone->attributes.components.pwm[ i ]->set_pwm_dc( drone->attributes.components.pwm[ i ], drone->attributes.components.pwm[ i ]->dc_min );
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
    { .name = "ST_PROPELLER_CALIBRATION", .func = &StControlVibrationCheck },
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

    sm_state_t prev_state = state_machine->curr_state;

    /* Loop through the entire transition matrix to match actual state and occurred event */
    for( int i = 0; i < sizeof( state_trans_matrix ) / sizeof( state_trans_matrix[ 0 ] ); i++ ) {

        /* If matched actual state */
        if( state_trans_matrix[ i ].curr_state == drone->attributes.state_machine.curr_state ) {

            /* If matched occurred event */
            if( state_trans_matrix[ i ].event == drone->attributes.state_machine.event ) {

                /* Go to the next state */
                drone->attributes.state_machine.curr_state = state_trans_matrix[ i ].next_state;

                /* Log transition if state changed */
                if (state_machine->curr_state != prev_state) {
                    ESP_LOGI(STATE_MACHINE_TAG, "Transition: %s -> %s on event %s",
                        StateMachine_GetStateName(prev_state),
                        StateMachine_GetStateName(state_machine->curr_state),
                        StateMachine_GetEventName(state_machine->event));
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
