#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/**
 * @brief State machine tag for logging 
 */
extern const char * STATE_MACHINE_TAG;


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

// extern const char *state_names[];

/**
 * @brief Events of machine
 */
typedef enum state_machine_events {

    EV_ANY,
    EV_CROSS,
    EV_TRIANGLE,
    EV_CIRCLE,
    EV_SQUARE,
    EV_PS,
    EV_START,
    EV_UP,
    EV_DOWN,
    EV_LEFT,
    EV_RIGHT,
    EV_R1,
    EV_R2,
    EV_L1,
    EV_L2
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
void StateMachine_RunIteration(drone_t * drone);

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

/**
 * State functions
 * ---------------------------
 */

void StIdleFunc( drone_t * drone );
void StWaitingFunc( drone_t * drone );
void StInitFunc( drone_t * drone );
void StControlFunc( drone_t * drone );
void StCalibrationFunc( drone_t * drone );
void StVibrationCheck( drone_t * drone );
void StResetFunc( drone_t * drone );


#endif
