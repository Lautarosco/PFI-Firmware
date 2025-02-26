#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/**
 * @brief Machine states
 */
typedef enum state_machine_states {
    
    ST_IDLE,
    ST_INIT,
    ST_WAITING,
    ST_CALIBRATION,
    ST_CONTROL,
    ST_PROPELLER_CALIBRATION,
    ST_RESET

} sm_state_t;

/**
 * @brief Events of machine
 */
typedef enum state_machine_events {

    EV_ANY,
    EV_CROSS,
    EV_TRIANGLE,
    EV_CIRCLE,
    EV_PS

} sm_event_t;

/**
 * @brief State machine
 */
typedef struct state_machine {
    /* Current state */
    sm_state_t curr_state;

    /* Occured event */
    sm_event_t event;

} sm_state_machine_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * Public functions prototypes
 * ---------------------------
 */

/**
 * @brief Initialize state machine
 * @param state_machine: Address of state machine
 * @retval none
 */
void StateMachine_Init( sm_state_machine_t * state_machine );

/* Forward declaration to avoid including drone.h header file */
typedef struct drone drone_t;

/**
 * @brief Transition to the next state
 * @param state_machine: Address of the state machine
 * @param event: Occurred event
 * @retval none
 */
void StateMachine_RunIteration( sm_state_machine_t * state_machine, drone_t * drone );

/**
 * @brief Get the name of a given state
 * @param state: state
 * @retval state name
 */
const char * StateMachine_GetStateName( sm_state_t state );

/**
 * @brief Get the name of a given event
 * @param event: Desired event's name
 * @retval const char *
 */
const char * StateMachine_GetEventName( sm_event_t event );


#endif
