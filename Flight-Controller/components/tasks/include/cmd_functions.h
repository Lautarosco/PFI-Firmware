#ifndef CMD_FUNCTIONS_H
#define CMD_FUNCTIONS_H

/* Enum containing index of a cmd frame */
typedef enum cmd_index {

    /* Command */
    CMD_INDEX,

    /* State */
    STATE_INDEX,

    /* Variable */
    VAR_INDEX,

    /* New value */
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

/**
 * @brief Update general variables
 * 
 * @param drone: Pointer to drone object
 * @param arr: Array containing processed data from received cmd
 * 
 * @return none
 * 
 */
void VarsUpdateCmdFunc(drone_t * drone, char * arr[4]);


/**
 * @brief Update Set Point
 * 
 * @param drone: Pointer to drone object
 * @param arr: Array containing processed data from received cmd
 * 
 * @return none
 * 
 */
void SpUpdateCmdFunc(drone_t * drone, char * arr[4]);

#endif
