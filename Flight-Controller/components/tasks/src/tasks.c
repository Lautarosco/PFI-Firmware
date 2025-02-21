#include <stdio.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <drone.h>
#include <state_machine.h>
#include <string.h>
#include <esp_log.h>
#include <ctype.h>
#include <uart_init.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Retrieve state's label
 * @param stateIndex: state index ( See state_t enum defined in controllers_structs.h header file )
 * @retval State's label
 */
static const char * GetStateName( int stateIndex );


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief State's name and index
 */
typedef struct stateSpecs {

    const char * name;
    int index;

} stateSpecs_t;

/* State's matrix */
static stateSpecs_t state_specs[] = {

    { .name = "z",       .index = z },
    { .name = "roll",    .index = roll },
    { .name = "pitch",   .index = pitch },
    { .name = "yaw",     .index = yaw },
    { .name = "roll_d",  .index = roll_dot },
    { .name = "pitch_d", .index = pitch_dot },
    { .name = "yaw_d",   .index = yaw_dot },
};

/**
 * @brief Get state's index given it's name
 * @param stateName: State's name
 * @retval State's index
 */
int GetStateIndex( const char * stateName );


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Check if Bluetooth command for PID index is correct
 * @param index: Desired index
 * @param n_obj: Total of Pid objects
 * @param func: Function caller
 * @param line: Line from called
 * @retval true if index is OK - false if it's not
 */
static bool PID_INDEX_CHECK( int index, int n_obj, const char * func, int line ) {

    /* Default returning value */
    bool ret = false;

    /* If index is a string */
    if( isalpha( ( unsigned char ) index ) ) {

        ESP_LOGE( "TASK3", "Type error: index must be int. See function %s in line %d", func, line );
    }

    else if( index == -1 ) {

        ESP_LOGE( "TASK3", "Index error ( function %s, line %d ): STATE NOT FOUND ( check 'state_specs' variable from tasks.c source file ).", func, line );
    }

    /* If index is greater that total of Pid objects or less than 0 */
    else if( ( index >= n_obj ) || ( index < 0 ) ) {

        ESP_LOGE( "TASK3", "Index error: index out of range. See function %s in line %d", func, line );
    }

    /* Index is OK */
    else {

        ret = true;
    }

    /* Return answer */
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


static void vTaskUartEvent( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    /* UART port num */
    uart_port_t uart_num = UART_NUM_0;

    /* Initialize UART interface */
    QueueHandle_t uart_queue = uart_init( uart_num );

    /* UART event handler */
    uart_event_t uart_event;

    while ( 1 ) {

        /* Wait until UART interrupt returns an UART event */
        if( xQueueReceive( uart_queue, ( void * ) &uart_event, portMAX_DELAY ) ) {

            switch ( uart_event.type ) {

                /* Data received */
                case UART_DATA:

                    /* Avoid overlapping between UART and Bluetooth */
                    if( !obj->attributes.global_variables.serial_data->state ) {

                        /* Data received flag HIGH */
                        obj->attributes.global_variables.serial_data->state = true;

                        /* Update data length */
                        obj->attributes.global_variables.serial_data->len = uart_event.size;

                        /* Store received data into drone's global variable */
                        uart_read_bytes( uart_num, obj->attributes.global_variables.serial_data->data, uart_event.size, 100 );

                        /* Echo received data */
                        uart_write_bytes( uart_num, obj->attributes.global_variables.serial_data->data, uart_event.size );

                        /* Clear UART Rx buffer */
                        uart_flush( uart_num );
                    }

                    else{

                        ESP_LOGW( "TASK4", "[ %s ] Bluetooth command is currently being prossesed.", __func__ );
                    }

                default:
                    break;
            }
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void vTaskParseCommand( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    xTaskCreatePinnedToCore( vTaskUartEvent, "Task4", 1024 * 3, ( void * ) ( obj ), 0, NULL, CORE_0 );

    while( 1 ) {

        /* If data received */
        if( obj->attributes.global_variables.serial_data->state ) {

            /* Reset state to default value */
            obj->attributes.global_variables.serial_data->state = false;

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
            for( int i = 0; i < obj->attributes.global_variables.serial_data->len; i++ ) {

                /**
                 * Frame's format: <pid/state/@/value>
                 * where '@' could be 'p | i | d | min_err'
                 * 
                 * i.e, <pid/roll/p/10> which means
                 * 
                 * pid command
                 * state roll
                 * p indicates proportional action
                 * 10 is the new value for roll Kp gain
                 * 
                 * i.e, <pid/z/min_err/10> which means
                 * 
                 * pid command
                 * state z
                 * min_err indicates new value for z integral minimum error
                 * 10 is the new minimum value for the integral error
                 */

                /* Get actual char */
                char currChar = obj->attributes.global_variables.serial_data->data[ i ];

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
            if( !strcmp( ptrArr[ 0 ], "pid" ) ) {
                
                /* Get state's index */
                int index = GetStateIndex( ptrArr[ 1 ] );

                if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

                    /* If updating proportional action */
                    if( !strcmp( ptrArr[ 2 ], "p" ) ) {

                        obj->attributes.components.controllers[ index ]->gain.kp = atof( ptrArr[ 3 ] );
                        ESP_LOGW( "TASK3", "%s new kp [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.kp );
                        // char tx[ 50 ];
                        // sprintf( tx, "%f", obj->attributes.components.controllers[ index ]->gain.kp );
                        // uart_write_bytes( UART_NUM_2, ( char * ) tx, strlen( ( char * ) tx ) );
                    }

                    /* If updating integral action */
                    else if( !strcmp( ptrArr[ 2 ], "i" ) ) {

                        obj->attributes.components.controllers[ index ]->gain.ki = atof( ptrArr[ 3 ] );
                        ESP_LOGW( "TASK3", "%s new ki [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.ki );
                    }


                    /* If updating derivative action */
                    else if( !strcmp( ptrArr[ 2 ], "d" ) ) {

                        obj->attributes.components.controllers[ index ]->gain.kd = atof( ptrArr[ 3 ] );
                        ESP_LOGW( "TASK3", "%s new kd [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.kd );
                    }

                    /* If updating minimum integral error */
                    else if( !strcmp( ptrArr[ 2 ], "min_err" ) ) {

                        obj->attributes.components.controllers[ index ]->cfg.intMinErr = atof( ptrArr[ 3 ] );
                        ESP_LOGW( "TASK3", "%s new integral minimum error [ %.2f ]",
                            GetStateName( index ),
                            obj->attributes.components.controllers[ index ]->cfg.intMinErr
                        );
                    }

                    else {

                        ESP_LOGE( "TASK3", "Third parameter of bluetooth frame must be 'p' or 'i' or 'd' or 'min_err' " );
                    }
                }
            }
        }

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static const char * GetStateName( int stateIndex ) {

    for( int i = 0; i < ( ( sizeof( state_specs ) ) / ( sizeof( state_specs[ 0 ] ) ) ); i++ ) {

        if( state_specs[ i ].index == stateIndex ) {
            
            return state_specs[ i ].name;
        }
    }

    return "STATE NOT FOUND";
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


int GetStateIndex( const char * stateName ) {

    for( int i = 0; i < ( ( sizeof( state_specs ) ) / ( sizeof( state_specs[ 0 ] ) ) ); i++ ) {

        if( !strcmp( state_specs[ i ].name, stateName ) ) {
            
            return state_specs[ i ].index;
        }
    }

    return -1;
}
