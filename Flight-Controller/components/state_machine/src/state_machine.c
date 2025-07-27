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
/* Idle and Waiting Functions*/

void StIdleFunc( drone_t * drone ) {
    // printf( "IDLE\r\n" );
}

void StWaitingFunc( drone_t * drone ) {

    // printf( "WAITING\r\n" );
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
 * @details Public functions definitions
 */

void StateMachine_Init( sm_state_machine_t * state_machine ) {

    /* Default state */
    state_machine->curr_state = ST_IDLE;
}

void StateMachine_RunIteration(drone_t * drone) {
    char *task_name = pcTaskGetName(NULL);

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
                    ESP_LOGI(task_name, "Transition: %s -> %s on event %s",
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
