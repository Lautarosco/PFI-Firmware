#include <stdio.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <drone.h>
#include <state_machine.h>
#include <string.h>

#include <esp_log.h>
// #include <ctype.h>
#include <uart_init.h>

#include <cmd_functions.h>


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

            /* If user pressed PS */
            if( ( obj.attributes.global_variables.tx_buttons->ps ) ) {

                state_machine->event = EV_PS;

            } else {

                state_machine->event = EV_ANY;
            }

            break;

        case ST_WAITING:

            /* If user pressed PS */
            if( ( obj.attributes.global_variables.tx_buttons->ps ) ) {

                state_machine->event = EV_PS;

            } else if( obj.attributes.global_variables.tx_buttons->cross ) {

                state_machine->event = EV_CROSS;

            } else if( obj.attributes.global_variables.tx_buttons->circle ) {

                state_machine->event = EV_CIRCLE;

            } else if( obj.attributes.global_variables.tx_buttons->triangle ) {

                state_machine->event = EV_TRIANGLE;

            } else {

                state_machine->event = EV_ANY;
            }

            break;

        case ST_CALIBRATION:

            /* If user pressed PS */
            if( ( obj.attributes.global_variables.tx_buttons->ps ) ) {

                state_machine->event = EV_PS;

            } else {
            
                state_machine->event = EV_ANY;
            }

            break;

        case ST_CONTROL:
            /* If user pressed PS */
            if( obj.attributes.global_variables.tx_buttons->ps ) {

                state_machine->event = EV_PS;

            } else {

                state_machine->event = EV_ANY;
            }

            break;

        case ST_PROPELLER_CALIBRATION:
            
            /* If user pressed PS */
            if( ( obj.attributes.global_variables.tx_buttons->ps ) ) {

                state_machine->event = EV_PS;

            } else {

                state_machine->event = EV_ANY;
            }

            break;

        default:
            break;
    }
}

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

    while( 1 ) {

        /* Get occurred event */
        getEvent( &state_machine, *obj );

        /* Go to the next state and run it's respective function */
        StateMachine_RunIteration( &state_machine, obj );
        
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void vTaskDroneMeasure( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;
    
    while( 1 ) {

        /* Measure attitude and update bmi sensor internal registers with respective values */
        obj->attributes.components.bmi.measure( &( obj->attributes.components.bmi ) );

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */

void vTaskprint( void * drone_ ) {

    float t = 0.0f;
    drone_t* drone = ( drone_t * ) drone_;

    /**
     * @brief Variable format: <printer:time,%f|var,%f\n>
     * @details Used for sending variables such as drone states, pid values, etc.
     * @example i.e, <printer:t,%f|roll,%f>  This will send roll values to plotter app
     * 
     * 
     * @brief Static format: <static:var_name/var_attr,%f|\n>
     * @details Used for sending static variables
     * @example i.e, <static:roll/P,%f>  This will send proportional action of roll pid to plotter app
     */

    while( 1 ) {

        // printf( "Estado: %s\r\n", StateMachine_GetStateName( state_machine.curr_state ) );

        if( drone->attributes.init_ok) {

            printf(
                "printer:t,%f|roll,%f\n",
                t, drone->attributes.states.roll
            );

            printf(
                "static:roll/P,%.2f|roll/I,%.2f|roll/D,%.2f\n",
                drone->attributes.components.controllers[ ROLL ]->gain.kp,
                drone->attributes.components.controllers[ ROLL ]->gain.ki,
                drone->attributes.components.controllers[ ROLL ]->gain.kd
            );
        
            t += 0.01f;      
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
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


/**
 * @brief Command function
 */
typedef struct cmd_function {

    /* Name of the command */
    const char * cmd_name;

    /** @brief Compute the function of a given command @param obj: Address of Drone object @param arr: Array containing processed data from the original command */
    void ( * func )( drone_t * obj, char * arr[ 4 ] );

} cmd_function_t;

static cmd_function_t cmd_function_array[] = {

    { .cmd_name = "pid gains",   .func = &PidGainsCmdFunc },
    { .cmd_name = "pid actions", .func = &PidActionsCmdFunc },   
};


void vTaskParseCommand( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    /* Start UART cmd detection task */
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
            char * ptr = ( char * ) malloc( 256 * sizeof( char ) ); /* PENDIENTE REEMPLAZAR POR 'char * ptr[ 256 ];' */

            /* Pointer of char ( array of 4 strings ) */
                char * ptrArr[ 4 ] = { 0 };

            /* Pointer index */
            int ptrIndex = 0;

            /* Array index */
            int ptrArrIndex = 0;

            /* Loop through data received */
            for( int i = 0; i < obj->attributes.global_variables.serial_data->len; i++ ) {

                /**
                 * Frame's format: <pid,state,@,value>
                 * where '@' could be 'p | i | d | b'
                 * 
                 * i.e, <pid/roll/p/10> which means
                 * 
                 * pid command
                 * state roll
                 * p indicates proportional action
                 * 10 is the new value for roll Kp gain
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
                else if( currChar == ',' ) {

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

            bool found = false;

            /* Loop through the cmd function array */
            for( int i = 0; i < ( ( sizeof( cmd_function_array ) ) / ( sizeof( cmd_function_array[ 0 ] ) ) ); i++ ) {

                /* Check if recevied command matches listed commands in the array */
                if( !strcmp( ptrArr[ CMD_INDEX ], cmd_function_array[ i ].cmd_name ) ) {

                    cmd_function_array[ i ].func( obj, ptrArr );
                    found = true;
                }
            }

            /* Check if received cmd was not found in cmds array */
            if( !found ) {

                ESP_LOGW( "TASK3", "Command not found.\n[ Details ] See func: %s, in line %d", __func__, __LINE__ );
            }
        }

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}
