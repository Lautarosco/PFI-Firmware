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

/**
 * @brief Command function
 */
typedef struct cmd_function {

    /* Name of the command */
    const char * cmd_name;

    /** @brief Compute the function of a given command @param obj: Address of Drone object @param arr: Array containing processed data from the original command */
    void ( * func )( drone_t * obj, char * arr[ 4 ] );

} cmd_function_t;

/**
 * @brief Update Pid actions with received command
 */
typedef struct pid_action_function {

    /* Action name */
    const char * action_name;

    /* Pointer to Pid Class set action method */
    void ( * pid_setterFunc )( pid_controller_t * obj, ControllerFunction * actionFunc );

    /* Pointer to controller action function */
    float ( * actionFunc )( pid_controller_t * obj, float error );

} pid_action_function_t;

/**
 * @brief Get state's index ( states enum ) given it's name
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

    while( 1 ) {

        // printf( "Estado: %s\r\n", StateMachine_GetStateName( state_machine.curr_state ) );
        #define PRINTER
        #ifdef PRINTER
            if( drone->attributes.init_ok) {

                float vroll = drone->attributes.states.roll;
                float vroll_d = drone->attributes.states.roll_dot;
                float sp_roll = drone->attributes.sp.roll;
                float sp_roll_d = drone->attributes.sp.roll_dot;

                float pid_roll = drone->attributes.components.mma->input[ C_ROLL ];

                float w1 = drone->attributes.components.mma->output[ U1 ];
                float w2 = drone->attributes.components.mma->output[ U2 ];
                float w3 = drone->attributes.components.mma->output[ U3 ];
                float w4 = drone->attributes.components.mma->output[ U4 ];

                float lower_limit = drone->attributes.components.mma->limit.lower;

                printf("printer:t,%.3f|roll,%.2f|roll_d,%.2f|sp_roll,%.2f|sp_roll_d,%.2f", t, vroll, vroll_d, sp_roll, sp_roll_d);
                printf( "|pid_roll_max,%.2f", drone->attributes.config.pid_cfgs[ ROLL ].pid_output_limits.max );
                printf( "|pid_roll_min,%.2f", drone->attributes.config.pid_cfgs[ ROLL ].pid_output_limits.min );
                printf( "|pid_roll_d_max,%.2f", drone->attributes.config.pid_cfgs[ ROLL_D ].pid_output_limits.max );
                printf( "|pid_roll_d_min,%.2f", drone->attributes.config.pid_cfgs[ ROLL_D ].pid_output_limits.min );
                printf("|pid_roll,%.2f", pid_roll);
                printf("|lower_limit,%.2f", lower_limit);
                printf("|w1,%.2f|w2,%.2f|w3,%.2f|w4,%.2f\n", w1, w2, w3, w4);

                // Prints de variables estáticas

                float roll_P = drone->attributes.components.controllers[ROLL]->gain.kp;
                float roll_I = drone->attributes.components.controllers[ROLL]->gain.ki;
                float roll_D = drone->attributes.components.controllers[ROLL]->gain.kd;

                float roll_d_P = drone->attributes.components.controllers[ROLL_D]->gain.kp;
                float roll_d_I = drone->attributes.components.controllers[ROLL_D]->gain.ki;
                float roll_d_D = drone->attributes.components.controllers[ROLL_D]->gain.kd;
                float d_filter_iir_coeff = drone->attributes.config.IIR_coeff_roll_dot;


                printf("static:roll/P,%.2f|roll/I,%.2f|roll/D,%.2f|", roll_P, roll_I, roll_D);
                printf("roll_d/P,%.2f|roll_d/I,%.2f|roll_d/D,%.2f|roll_d/d_filter_iir_coeff,%.2f\n", roll_d_P, roll_d_I, roll_d_D, d_filter_iir_coeff);
                t += 0.010f;
            }
        #endif

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


static void PidGainsCmdFunc( drone_t * obj, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ STATE_INDEX ] );

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        /* If updating proportional gain ( Kp ) */
        if( !strcmp( arr[ VAR_INDEX ], "p" ) ) {

            obj->attributes.components.controllers[ index ]->gain.kp = atof( arr[ VALUE_INDEX ] );
            ESP_LOGW( "TASK3", "%s new Kp [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.kp );
        }

        /* If updating integral gain ( Ki ) */
        else if( !strcmp( arr[ VAR_INDEX ], "i" ) ) {

            obj->attributes.components.controllers[ index ]->gain.ki = atof( arr[ VALUE_INDEX ] );
            ESP_LOGW( "TASK3", "%s new Ki [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.ki );
        }


        /* If updating derivative gain ( Kd ) */
        else if( !strcmp( arr[ VAR_INDEX ], "d" ) ) {

            obj->attributes.components.controllers[ index ]->gain.kd = atof( arr[ VALUE_INDEX ] );
            ESP_LOGW( "TASK3", "%s new Kd [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.kd );
        }

        /* If updating back calculation gain ( Kb ) */
        else if( !strcmp( arr[ VAR_INDEX ], "b" ) ) {

            obj->attributes.components.controllers[ index ]->gain.kd = atof( arr[ VALUE_INDEX ] );
            ESP_LOGW( "TASK3", "%s new Kd [ %.2f ]", GetStateName( index ), obj->attributes.components.controllers[ index ]->gain.kb );
        }

        else {

            ESP_LOGE( "TASK3", "Third parameter of frame must be one of the following 'p, i, d, b' " );
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static pid_action_function_t pid_actions_array[] = {

    { .action_name = "P_Basic",    .pid_setterFunc = &PidSetActionP, .actionFunc = P_Basic },
    { .action_name = "I_Basic",    .pid_setterFunc = &PidSetActionI, .actionFunc = I_Basic },
    { .action_name = "I_Clamping", .pid_setterFunc = &PidSetActionI, .actionFunc = I_Clamping },
    { .action_name = "I_BackCalc", .pid_setterFunc = &PidSetActionI, .actionFunc = I_BackCalc },
    { .action_name = "D_Basic",    .pid_setterFunc = &PidSetActionD, .actionFunc = D_Basic },
    { .action_name = "D_LPF",      .pid_setterFunc = &PidSetActionD, .actionFunc = D_LPF },
};

static void PidActionsCmdFunc( drone_t * obj, char * arr[ 4 ] ) {

    /* Get index ( states enum ) of received state */
    int index = GetStateIndex( arr[ STATE_INDEX ] );

    bool found = false;

    /* Check if received state is valid */
    if( PID_INDEX_CHECK( index, sizeof( obj->attributes.components.controllers ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ), __func__, __LINE__ ) ) {

        for( int i = 0; i < ( ( sizeof( pid_actions_array ) ) / ( sizeof( pid_actions_array[ 0 ] ) ) ); i++ ) {

            if( !strcmp( pid_actions_array->action_name, arr[ VALUE_INDEX ] ) ){

                pid_actions_array->pid_setterFunc( obj->attributes.components.controllers[ index ], pid_actions_array->actionFunc );
                found = true;
            }
        }

        /* Check if action was found */
        if( !found ) {

            ESP_LOGW( "TASK3", "PID action name was not found.\n[ Details ] See func %s, in line %d", __func__, __LINE__ );
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


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
                 * Frame's format: <pid/state/@/value>
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

            bool found = false;

            /* Loop through the cmd function array */
            for( int i = 0; i < ( ( sizeof( cmd_function_array ) ) / ( sizeof( cmd_function_array[ 0 ] ) ) ); i++ ) {

                /* Check if recevied command matches listed commands in the array */
                if( !strcmp( ptrArr[ CMD_INDEX ], cmd_function_array->cmd_name ) ) {

                    cmd_function_array->func( obj, ptrArr );
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


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/* State's matrix */
static stateSpecs_t state_specs[] = {

    { .name = "z",       .index = Z },
    { .name = "roll",    .index = ROLL },
    { .name = "pitch",   .index = PITCH },
    { .name = "yaw",     .index = YAW },
    { .name = "roll_d",  .index = ROLL_D },
    { .name = "pitch_d", .index = PITCH_D },
    { .name = "yaw_d",   .index = YAW_D },
};


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
