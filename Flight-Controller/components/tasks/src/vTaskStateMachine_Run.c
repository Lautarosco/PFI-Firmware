#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>
#include <esp_timer.h>
#include "tasks.h"

#include "button_helper.h"
#include "state_machine.h"
#include "drone.h"

/** @details Private functions definitions */

/**
 * @brief Get ocurred event and update state machine object with it
 * @param state_machine: Address of state machine
 * @param drone: Drone object
 * @retval none
 */
static void getEvent( sm_state_machine_t * state_machine, drone_t* drone ) {

    state_machine->event = EV_ANY;

    // Handle pending state transitions requests first
    if (drone->attributes.request_state_transition) {
        state_machine->event = drone->attributes.requested_transition_event;
        drone->attributes.request_state_transition = false;
        drone->attributes.requested_transition_event = EV_ANY; // Reset the requested transition event
        return;
    }



    /* Previous = Current */
    memcpy(&drone->attributes.buttons.previous,
           &drone->attributes.buttons.current,
           sizeof(tx_buttons_t));

    /* Current = Global.tx_buttons*/
    memcpy(&drone->attributes.buttons.current,
           &drone->attributes.global_variables.tx_buttons,
           sizeof(tx_buttons_t));


    switch (state_machine->curr_state) {
        case ST_PROPELLER_CALIBRATION:
            if (pressed(drone, EV_SQUARE)) {
                printf("Propeller calibration finished\n");
                state_machine->event = EV_ANY;
                return;
            }  // positive edge
            break;

        default:
            /* If user pressed PS */
            if( pressed(drone, EV_PS) ) {
                state_machine->event = EV_PS;

            } else if( pressed(drone, EV_START) ) {
                state_machine->event = EV_START;
            }
            /* If user pressed Cross*/
             else if( pressed(drone, EV_CROSS) ) {
                state_machine->event = EV_CROSS;

            /* If user pressed Circle*/
            } else if( pressed(drone, EV_CIRCLE) ) {
                state_machine->event = EV_CIRCLE;

            /* If user pressed Triangle*/
            } else if( pressed(drone, EV_TRIANGLE) ) {
                state_machine->event = EV_TRIANGLE;

            } else if( pressed(drone, EV_SQUARE) ) {
                state_machine->event = EV_SQUARE;
                printf("Square pressed\n");

            } else if( pressed(drone, EV_UP) ) {
                state_machine->event = EV_UP;

            } else if( pressed(drone, EV_DOWN) ) {
                state_machine->event = EV_DOWN;

            } else if( pressed(drone, EV_LEFT) ) {
                state_machine->event = EV_LEFT;

            } else if( pressed(drone, EV_RIGHT) ) {
                state_machine->event = EV_RIGHT;

            } else if( pressed(drone, EV_R1) ) {
                state_machine->event = EV_R1;

            } else if( pressed(drone, EV_R2) ) {
                state_machine->event = EV_R2;

            } else if( pressed(drone, EV_L1) ) {
                state_machine->event = EV_L1;

            } else if( pressed(drone, EV_L2) ) {
                state_machine->event = EV_L2;

            }
    
    }
}


void vTaskStateMachine_Run( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * drone = ( drone_t * ) pvParameters;

    /* Create a state_machine object */

    /* TESTING */

    // sm_state_machine_t state_machine;    /* It will end being local */
    // extern sm_state_machine_t state_machine;

    /* TESTING */

    /* Initialize state_machine object */
    StateMachine_Init( &drone->attributes.state_machine );

    while( 1 ) {

        // int64_t start_time = esp_timer_get_time();

        /* Get occurred event */
        getEvent( &drone->attributes.state_machine, drone );

        /* Go to the next state and run it's respective function */
        StateMachine_RunIteration(drone);

        vTaskDelay( pdMS_TO_TICKS( 1 ) );

        // int64_t end_time = esp_timer_get_time();
        // printf("State machine iteration took %lld us\n", (end_time - start_time));
    }
}