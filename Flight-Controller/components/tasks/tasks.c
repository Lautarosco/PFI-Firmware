#include <stdio.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <drone.h>
#include <state_machine.h>
#include <string.h>
#include <esp_log.h>
#include <ctype.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Retrieve state's label
 * @param stateIndex: state index ( See state_t enum defined in controllers_structs.h header file )
 * @retval State's label
 */
static const char * GetStateName( int stateIndex );


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


typedef struct {
    const char *name;
    int index;
} ControllerMap;

ControllerMap controller_map[] = {
    {"z", 0},
    {"roll", 1},
    {"pitch", 2},
    {"yaw", 3},
    {"roll_d", 1},
    {"pitch_d", 2},
    {"yaw_d", 3},
    {NULL, -1} // Sentinel value to indicate the end of the array
};

int get_controller_index(const char *name) {
    for (int i = 0; controller_map[i].name != NULL; i++) {
        if (strcmp(controller_map[i].name, name) == 0) {
            return controller_map[i].index;
        }
    }
    return -1; // Return -1 if no match is found
}

/**
 * @brief Check if Bluetooth command for PID index is correct
 * @param index_str: String identifier (e.g., "roll", "pitch")
 * @param n_obj: Total number of PID controllers
 * @param func: Function caller
 * @param line: Line from caller
 * @retval true if index is OK, false if it's not
 */
static bool pid_index_check(const char *index_str, const char *func, int line) {
    
    /* Convert string to index */
    int index = get_controller_index(index_str);

    /* Log the converted index */
    printf("Converted Index: %d\r\n", index);

    /* Default return value */
    bool ret = false;

    /* Check if conversion failed (index = -1 means not found) */
    if (index == -1) {
        ESP_LOGE("TASK3", "Invalid PID identifier: '%s'. See function %s in line %d", index_str, func, line);
    }
    /* If index is out of range */
    else if (index >= (sizeof(controller_map)/sizeof(controller_map[0])-1) || index < 0) {
        ESP_LOGE("TASK3", "Index error: '%s' out of range. See function %s in line %d", index_str, func, line);
    }
    /* Index is OK */
    else {
        ret = true;
    }

    return ret;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Private functions definitions */

/**
 * @brief Get ocurred event and update state machine object with it
 * @param state_machine: Address of state machine
 * @param obj: Drone object
 * @retval none
 */
static void getEvent( sm_state_machine_t * state_machine, drone_t obj ) {

    switch( state_machine->curr_state ) {
        
        case ST_IDLE:

            /* If user pressed X and drone isn't initialized */
            if( ( obj.attributes.global_variables.tx_buttons->cross ) && ( !obj.attributes.init_ok ) ) {
                
                state_machine->event = EV_CROSS;
            }

            /* If user pressed △ */
            else if( obj.attributes.global_variables.tx_buttons->triangle ) {

                /* If drone isn't initialized */
                if( !obj.attributes.init_ok ) {

                    state_machine->event = EV_CROSS;
                }

                /* If drone is already initialized */
                else {

                    state_machine->event = EV_TRIANGLE;
                }
            }

            /* If user pressed ◯ */
            else if( obj.attributes.global_variables.tx_buttons->circle ) {

                /* If drone isn't initialized */
                if( !obj.attributes.init_ok ) {

                    state_machine->event = EV_CROSS;
                }

                /* If drone is already initialized */
                else {

                    state_machine->event = EV_CIRCLE;
                }
            }

            /* If user pressed PS */
            else if( ( obj.attributes.global_variables.tx_buttons->ps ) ) {

                state_machine->event = EV_PS;
            }

            /* If no one of above buttons is being pressed */
            else {

                state_machine->event = EV_ANY;
            }

            break;

        case ST_INIT:
            state_machine->event = EV_ANY;
            break;

        case ST_CALIBRATION:
            state_machine->event = EV_ANY;
            break;

        case ST_UPDATE_STATES:
            /* If user pressed PS */
            if( obj.attributes.global_variables.tx_buttons->ps ) {

                state_machine->event = EV_PS;
            }

            else {

                state_machine->event = EV_ANY;
            }
            
            break;

        case ST_CONTROL:
            /* If user pressed PS */
            if( obj.attributes.global_variables.tx_buttons->ps ) {

                state_machine->event = EV_PS;
            }

            else {

                state_machine->event = EV_ANY;
            }

            break;

        default:
            break;
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions definitions */

void vTaskStateMachine_Run( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    /* Create a state_machine object */

    /* TESTING */

    // sm_state_machine_t state_machine;    /* It will end being local */
    extern sm_state_machine_t state_machine;

    /* TESTING */

    /* Initialize state_machine object */
    StateMachine_Init( &state_machine );

    /* Get Drone Class general configs */
    drone_cfg_t Droneconfigs = GetDroneConfigs();

    while( 1 ) {

        /* Get occurred event */
        getEvent( &state_machine, *obj );

        /* Go to the next state and run it's respective function */
        StateMachine_RunIteration( &state_machine, obj );
        
        vTaskDelay( pdMS_TO_TICKS( Droneconfigs.ControllersConfigs[ z ].ts ) );
    }
    
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void vTaskDroneMeasure( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    /* Get Drone Class general configs */
    drone_cfg_t Droneconfigs = GetDroneConfigs();

    while( 1 ) {

        /* Measure attitude and update bmi sensor internal registers with respective values */
        obj->attributes.components.bmi.measure( &( obj->attributes.components.bmi ) );

        vTaskDelay( pdMS_TO_TICKS( Droneconfigs.ControllersConfigs[ z ].ts ) );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void vTaskParseBluetooth( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    while( 1 ) {

        /* If data received */
        if( obj->attributes.global_variables.bt_data->state ) {

            /* Reset state to default value */
            obj->attributes.global_variables.bt_data->state = false;

            /* Error detection flags */
            bool err = false;
            bool eof = false;

            /* Pointer to store each char of substring */
            char * ptr = ( char * ) malloc( 256 * sizeof( char ) );

            /* Pointer of char ( array of 4 strings ) */
            char * ptrArr[ 4 ] = { 0 };

            /* Pointer index */
            int ptrIndex = 0;

            /* Array index */
            int ptrArrIndex = 0;

            /* Loop through data received */
            for( int i = 0; i < obj->attributes.global_variables.bt_data->len; i++ ) {

                /**
                 * Frame's format: <pid/state/( p | i | d | min_err )/value>
                 * 
                 * i.e, <pid/1/p/10> which means
                 * 
                 * pid command
                 * state 1 indicates roll ( see states_t enum defined in controllers_struct.h header file )
                 * p indicates proportional action
                 * 10 is the new value for roll Kp gain
                 * 
                 * i.e, <pid/0/min_err/10> which means
                 * 
                 * pid command
                 * state 1 indicates z ( see states_t enum defined in controllers_struct.h header file )
                 * min_err indicates new value for z integral minimum error
                 * 10 is the new minimum value for the integral error
                 */

                /* Get actual char */
                char currChar = obj->attributes.global_variables.bt_data->data[ i ];

                /* Checek if start of frame is correct */
                if( !i ) {

                    if( currChar == '<' ) {

                        continue;
                    }

                    else {

                        err = true;
                        ESP_LOGE( "TASK3", "Frame must start with '<' character. See function %s in line %d", __func__, __LINE__ );
                    }
                }

                /* Check if end of frame is correct */
                else if( currChar == '>' ) {
                
                    /* End of frame is correct */
                    eof = true;
                    
                    /* Store last substring */
                    ptr[ ptrIndex ] = '\0';
                    ptrArr[ ptrArrIndex++ ] = strdup( ptr );

                    break;
                }

                /* Check if end of substring */
                else if( currChar == '/' ) {

                    /* Store substring */
                    ptr[ ptrIndex ] = '\0';
                    ptrArr[ ptrArrIndex++ ] = strdup( ptr );

                    /* Reset substring's index */
                    ptrIndex = 0;
                }

                /* Keep adding char to substring */
                else {

                    ptr[ ptrIndex++ ] = currChar;
                }
            }

            /* Errors check */
            if( !eof ) {

                ESP_LOGE( "TASK3", "Frame must end with '>' character. See function %s in line %d", __func__, __LINE__ );
                continue;
            }

            else if( err == true ) {

                continue;
            }

            /* Check if it's a PID command */
            if (!strcmp(ptrArr[0], "pid")) {

                int index = get_controller_index(ptrArr[1]); // Convert name to index

            if (pid_index_check(ptrArr[1], __func__, __LINE__)) {

                    /* If updating proportional action */
                    if (!strcmp(ptrArr[2], "p")) {
                        obj->attributes.components.controllers[index]->gain.kp = atof(ptrArr[3]);
                        ESP_LOGW("TASK3", "%s new kp [ %.2f ]", GetStateName(index), obj->attributes.components.controllers[index]->gain.kp);
                    }

                    /* If updating integral action */
                    else if (!strcmp(ptrArr[2], "i")) {
                        obj->attributes.components.controllers[index]->gain.ki = atof(ptrArr[3]);
                        ESP_LOGW("TASK3", "%s new ki [ %.2f ]", GetStateName(index), obj->attributes.components.controllers[index]->gain.ki);
                    }

                    /* If updating derivative action */
                    else if (!strcmp(ptrArr[2], "d")) {
                        obj->attributes.components.controllers[index]->gain.kd = atof(ptrArr[3]);
                        ESP_LOGW("TASK3", "%s new kd [ %.2f ]", GetStateName(index), obj->attributes.components.controllers[index]->gain.kd);
                    }

                    /* If updating minimum integral error */
                    else if (!strcmp(ptrArr[2], "min_err")) {
                        obj->attributes.components.controllers[index]->cfg.intMinErr = atof(ptrArr[3]);
                        ESP_LOGW("TASK3", "%s new integral minimum error [ %.2f ]",
                            GetStateName(index),
                            obj->attributes.components.controllers[index]->cfg.intMinErr
                        );
                    }

                    else {
                        ESP_LOGE("TASK3", "Third parameter of bluetooth frame must be 'p' or 'i' or 'd' or 'min_err'");
                    }
                } else {
                    ESP_LOGE("TASK3", "Invalid controller name: %s", ptrArr[1]);
                }
            }
        }

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static const char * GetStateName( int stateIndex ) {

    switch ( stateIndex ) {

        case z:
            return "z";
            break;
        
        case roll:
            return "roll";
            break;

        case pitch:
            return "pitch";
            break;

        case yaw:
            return "yaw";
            break;

        case roll_dot:
            return "roll_dot";
            break;

        case pitch_dot:
            return "pitch_dot";
            break;

        case yaw_dot:
            return "yaw_dot";
            break;

        default:
            return "UNDEFINED STATE";
            break;
    }
}
