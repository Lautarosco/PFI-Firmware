#include <stdio.h>
#include <state_machine.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

const char * STATE_MACHINE_TAG = "STATE_MACHINE";


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
        obj->attributes.global_variables.tx_buttons->cross = false;
    #endif
}

static void StUpdateStatesFunc( drone_t * obj ) {

    obj->methods.update_states( obj, GetDroneConfigs().ControllersConfigs[ z ].ts );
}

/* Timer used to get sine values */
static float timer = 0.0f;

static void StControlFunc( drone_t * obj ) {

    /* Get Drone Class generic configs */
    // drone_cfg_t DroneConfigs = GetDroneConfigs();

    /* Update sp */
    obj->attributes.sp.roll = ( 180 / M_PI ) * __sin( 0.02f, 1.0f, timer );

    /* Compute PID algorithm for all states */
    float CRoll = obj->attributes.components.controllers[ roll ]->pid(
        obj->attributes.components.controllers[ roll ],
        obj->attributes.states.roll,
        obj->attributes.sp.roll
    );

    // printf( "%f\r\n", ( 180 / M_PI ) * __sin( 0.02f, 1.0f, timer ) );

    /* If timer exceeds from 2π */
    if( timer > ( 2 * M_PI ) ) {

        /* Sine function goes from 0 to 2π, hence start again from 0 */
        timer = 0.0f;
    }

    /* Timer value is between 0 and 2π */
    else {

        /* Increment timer by 0.1 ms */
        timer += 10 / 1000.0f;
    }
    
    /* Update MMA inputs with PID outputs */
    obj->attributes.components.mma->input[ roll ] = CRoll;

    /* Compute MMA algorithm */
    obj->attributes.components.mma->compute(
        obj->attributes.components.mma,
        obj->attributes.components.pwm[ 0 ]->dc_min,
        obj->attributes.components.pwm[ 0 ]->dc_max
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

        if( obj->attributes.global_variables.tx_buttons->cross ) {

            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons->cross = false;
            #endif

            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* 3. Wait until ESC's have latched maximum value ( user pressed X button ) */

    ESP_LOGW( STATE_MACHINE_TAG, "Press X button when all ESC have latched maximum value"  );
    while( 1 ) {

        if( obj->attributes.global_variables.tx_buttons->cross ) {

            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons->cross = false;
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

        if( obj->attributes.global_variables.tx_buttons->cross ) {
        
            #if WEBSV_TX
                /* Reset button */
                obj->attributes.global_variables.tx_buttons->cross = false;
            #endif

            break;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }

    /* End of calibration routine */
    ESP_LOGW( STATE_MACHINE_TAG, "Calibration completed."  );

    #if WEBSV_TX
        /* Reset button */
        obj->attributes.global_variables.tx_buttons->triangle = false;  /* TESTING */
    #endif
}

static void StResetFunc( drone_t * obj ) {

    esp_restart();
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static state_func_row_t state_function_array[  ] = {

    { .name = "ST_IDLE",          .func = &StIdleFunc },
    { .name = "ST_INIT",          .func = &StInitFunc },
    { .name = "ST_CALIBRATION",   .func = &StCalibrationFunc },
    { .name = "ST_UPDATE_STATES", .func = &StUpdateStatesFunc },
    { .name = "ST_CONTROL",       .func = &StControlFunc },
    { .name = "ST_RESET",         .func = &StResetFunc },

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
static const state_trans_row_t state_trans_matrix[  ] = {

    { .curr_state = ST_IDLE,          .event = EV_ANY,      .next_state = ST_IDLE },
    { .curr_state = ST_IDLE,          .event = EV_CROSS,    .next_state = ST_INIT },
    { .curr_state = ST_IDLE,          .event = EV_TRIANGLE, .next_state = ST_CALIBRATION },
    { .curr_state = ST_IDLE,          .event = EV_CIRCLE,   .next_state = ST_UPDATE_STATES },
    { .curr_state = ST_IDLE,          .event = EV_PS,       .next_state = ST_RESET },
    { .curr_state = ST_INIT,          .event = EV_ANY,      .next_state = ST_IDLE },
    { .curr_state = ST_CALIBRATION,   .event = EV_ANY,      .next_state = ST_IDLE },
    { .curr_state = ST_UPDATE_STATES, .event = EV_ANY,      .next_state = ST_CONTROL },
    { .curr_state = ST_UPDATE_STATES, .event = EV_PS,       .next_state = ST_RESET },
    { .curr_state = ST_CONTROL,       .event = EV_ANY,      .next_state = ST_UPDATE_STATES },
    { .curr_state = ST_CONTROL,       .event = EV_PS,       .next_state = ST_RESET },
};


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @details Public functions definitions
 */

void StateMachine_Init( sm_state_machine_t * state_machine ) {

    /* Default state */
    state_machine->curr_state = ST_IDLE;
}

void StateMachine_RunIteration( sm_state_machine_t * state_machine, drone_t * drone ) {

    // printf( "Current state: %s\r\nCurrent event: %s\r\n", StateMachine_GetStateName( state_machine->curr_state ), StateMachine_GetEventName( state_machine->event ) );

    /* Loop through the entire transition matrix to match actual state and occurred event */
    for( int i = 0; i < sizeof( state_trans_matrix ) / sizeof( state_trans_matrix[ 0 ] ); i++ ) {

        /* If matched actual state */
        if( state_trans_matrix[ i ].curr_state == state_machine->curr_state ) {

            /* If matched occurred event */
            if( state_trans_matrix[ i ].event == state_machine->event ) {

                /* Go to the next state */
                state_machine->curr_state = state_trans_matrix[ i ].next_state;

                /* Run new actual state respective function */
                state_function_array[ state_machine->curr_state ].func( drone );
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
