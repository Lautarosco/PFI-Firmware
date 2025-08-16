#ifndef CMD_FUNCTIONS_H
#define CMD_FUNCTIONS_H

/* Enum containing index of a cmd frame */
typedef enum cmd_index {

    /* X,_,_,_ */
    CMD_INDEX,

    /* _,X,_,_*/
    ARG1,

    /* _,_,X,_ */
    ARG2,

    /* _,_,_,X */
    ARG3

} cmd_index_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


typedef struct drone drone_t;   /* Forward declaration to avoid header inclusion */

/**
 * @brief Update PID controller actions function
 * @param pwm: Address of Drone object
 * @param arr: Array containing processed data from cmd received
 * @retval none
 */
void PidActionsCmdFunc( drone_t * pwm, char * arr[ 4 ] );

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
 * @brief Store all pointed variables in the ESP32 NVS
 * 
 * @param drone: Pointer to drone object
 * @param arr: Array containing processed data from received cmd
 * 
 * @return none
 * 
 */
void NvsStoreCmdFunc(drone_t * drone, char * arr[4]);

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

/**
 * @brief Emulate a transmitter button action
 * 
 * @param drone: Pointer to drone object
 * @param arr: Array containing processed data from received cmd
 * 
 * @return none
 * 
 */
void TxCmdFunc(drone_t * drone, char * arr[ 4 ]);


#endif
