#ifndef DRONE_TASKS_H
#define DRONE_TASKS_H


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Available MCU CPU Cores
 */
typedef enum cpu {

    /* CPU Core 0 */
    CORE_0,

    /* CPU Core 1 */
    CORE_1,
    
} cpu_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Freertos task to update next state of state machine
 * @param pvParameters: Any
 * @retval none
 */
void vTaskStateMachine_Run( void * pvParameters );

/**
 * @brief Freertos task to measure attitude and update bmi sensor internal registers
 * @param pvParameters: Any
 * @retval none
 */
void vTaskDroneMeasure( void * pvParameters );

/**
 * @brief Freertos task to parse Bluetooth data | Format: [ <pid><state><p/i/d><value> ] => For <state> use states enum
 * @param pvParameters: Any
 * @retval none
 */
void vTaskParseBluetooth( void * pvParameters );

#endif
