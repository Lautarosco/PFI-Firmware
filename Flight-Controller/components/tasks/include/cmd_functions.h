#ifndef CMD_FUNCTIONS_H
#define CMD_FUNCTIONS_H

/* Enum containing index of a cmd frame */
typedef enum cmd_index {

    /* Command index */
    CMD_INDEX,

    /* State index */
    STATE_INDEX,

    /* Variable to be updated index */
    VAR_INDEX,

    /* New value index */
    VALUE_INDEX

} cmd_index_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


typedef struct drone drone_t;   /* Forward declaration to avoid header inclusion */

/**
 * @brief Update PID controller gains
 * @param obj: Address of Drone object
 * @param arr: Array containing processed data from cmd received
 * @retval none
 */
void PidGainsCmdFunc( drone_t * obj, char * arr[ 4 ] );

/**
 * @brief Update PID controller actions function
 * @param obj: Address of Drone object
 * @param arr: Array containing processed data from cmd received
 * @retval none
 */
void PidActionsCmdFunc( drone_t * obj, char * arr[ 4 ] );

#endif
