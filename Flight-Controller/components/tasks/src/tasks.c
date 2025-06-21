#include <stdio.h>
#include <tasks.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <drone.h>
// #include <state_machine.h>
#include <string.h>

#include <esp_log.h>
// #include <ctype.h>
#include <uart_init.h>

#include <cmd_functions.h>

#include "button_helper.h"

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Private functions definitions */

/**
 * @brief Get ocurred event and update state machine object with it
 * @param state_machine: Address of state machine
 * @param drone: Drone object
 * @retval none
 */
static void getEvent( sm_state_machine_t * state_machine, drone_t* drone ) {

    state_machine->event = EV_ANY;

    // Handle pending state transitions requests first
    if (drone->attributes.request_state_transition) {
        state_machine->event = drone->attributes.requested_transition_event;
        drone->attributes.request_state_transition = false;
        drone->attributes.requested_transition_event = EV_ANY; // Reset the requested transition event
        return;
    }



    /* Previous = Current */
    memcpy(&drone->attributes.buttons.previous,
           &drone->attributes.buttons.current,
           sizeof(tx_buttons_t));

    /* Current = Global.tx_buttons*/
    memcpy(&drone->attributes.buttons.current,
           &drone->attributes.global_variables.tx_buttons,
           sizeof(tx_buttons_t));


    switch (state_machine->curr_state) {
        case ST_PROPELLER_CALIBRATION:
            if (pressed(drone, EV_SQUARE)) {
                printf("Propeller calibration finished\n");
                state_machine->event = EV_ANY;
                return;
            }  // positive edge
            break;

        default:
            /* If user pressed PS */
            if( pressed(drone, EV_PS) ) {
                state_machine->event = EV_PS;

            } else if( pressed(drone, EV_START) ) {
                state_machine->event = EV_START;
            }
            /* If user pressed Cross*/
             else if( pressed(drone, EV_CROSS) ) {
                state_machine->event = EV_CROSS;

            /* If user pressed Circle*/
            } else if( pressed(drone, EV_CIRCLE) ) {
                state_machine->event = EV_CIRCLE;

            /* If user pressed Triangle*/
            } else if( pressed(drone, EV_TRIANGLE) ) {
                state_machine->event = EV_TRIANGLE;

            } else if( pressed(drone, EV_SQUARE) ) {
                state_machine->event = EV_SQUARE;
                printf("Square pressed\n");

            } else if( pressed(drone, EV_UP) ) {
                state_machine->event = EV_UP;

            } else if( pressed(drone, EV_DOWN) ) {
                state_machine->event = EV_DOWN;

            } else if( pressed(drone, EV_LEFT) ) {
                state_machine->event = EV_LEFT;

            } else if( pressed(drone, EV_RIGHT) ) {
                state_machine->event = EV_RIGHT;

            } else if( pressed(drone, EV_R1) ) {
                state_machine->event = EV_R1;

            } else if( pressed(drone, EV_R2) ) {
                state_machine->event = EV_R2;

            } else if( pressed(drone, EV_L1) ) {
                state_machine->event = EV_L1;

            } else if( pressed(drone, EV_L2) ) {
                state_machine->event = EV_L2;

            }
    
    }
}

void vTaskStateMachine_Run( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * drone = ( drone_t * ) pvParameters;

    /* Create a state_machine object */

    /* TESTING */

    // sm_state_machine_t state_machine;    /* It will end being local */
    // extern sm_state_machine_t state_machine;

    /* TESTING */

    /* Initialize state_machine object */
    StateMachine_Init( &drone->attributes.state_machine );

    while( 1 ) {

        /* Get occurred event */
        getEvent( &drone->attributes.state_machine, drone );

        /* Go to the next state and run it's respective function */
        StateMachine_RunIteration(drone);
        
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

        if (obj->attributes.init_ok) {
            /* Update drone states */
            obj->methods.update_states( obj, 10 );

            /* Update sp */
            obj->attributes.sp.roll = 0;
            obj->attributes.sp.pitch = 0;
            obj->attributes.sp.yaw = 0;
            obj->attributes.sp.z = 0;
        }

        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */

void vTaskprint( void * drone_ ) {

    drone_t* drone = ( drone_t * ) drone_;

    /**
     * @brief Variable format: <printer:var,%f\n>
     * @details Used for sending variables such as drone states, pid values, etc.
     * @example i.e, <printer:roll,%f>  This will send roll values to plotter app
     * 
     * 
     * @brief Static format: <static:var_name/var_attr,%f|\n>
     * @details Used for sending static variables
     * @example i.e, <static:roll/P,%f>  This will send proportional action of roll pid to plotter app
     */
    static char buf[1024];

    while( 1 ) {

        // printf( "Estado: %s\r\n", StateMachine_GetStateName( state_machine.curr_state ) );

        if( drone->attributes.init_ok) {

            // Dynamic
            snprintf(buf, sizeof(buf),
                "printer:roll,%.2f|roll_d,%.2f|roll_sp,%.2f|roll_d_sp,%.2f|"
                "pitch,%.2f|pitch_d,%.2f|pitch_sp,%.2f|pitch_d_sp,%.2f|"
                "yaw,%.2f|yaw_d,%.2f|yaw_sp,%.2f|yaw_d_sp,%.2f|"
                "height,%.2f|height_sp,%.2f|"
                "dc1,%.2f|dc2,%.2f|dc3,%.2f|dc4,%.2f\n"  // end of dynamic values
                "static:roll/P,%.2f|roll/I,%.2f|roll/D,%.2f|roll_d/P,%.2f|roll_d/I,%.2f|roll_d/D,%.2f|"
                "pitch/P,%.2f|pitch/I,%.2f|pitch/D,%.2f|pitch_d/P,%.2f|pitch_d/I,%.2f|pitch_d/D,%.2f|"
                "yaw/P,%.2f|yaw/I,%.2f|yaw/D,%.2f|yaw_d/P,%.2f|yaw_d/I,%.2f|yaw_d/D,%.2f|"
                "state,%s\n",

                // dynamic state
                drone->attributes.states.roll,
                drone->attributes.states.roll_dot,
                drone->attributes.sp.roll,
                drone->attributes.sp.roll_dot,
                drone->attributes.states.pitch,
                drone->attributes.states.pitch_dot,
                drone->attributes.sp.pitch,
                drone->attributes.sp.pitch_dot,
                drone->attributes.states.yaw,
                drone->attributes.states.yaw_dot,
                drone->attributes.sp.yaw,
                drone->attributes.sp.yaw_dot,
                drone->attributes.states.z,
                drone->attributes.sp.z,
                drone->attributes.components.pwm[0].get_pwm_dc(&drone->attributes.components.pwm[0])*1000,
                drone->attributes.components.pwm[1].get_pwm_dc(&drone->attributes.components.pwm[1])*1000,
                drone->attributes.components.pwm[2].get_pwm_dc(&drone->attributes.components.pwm[2])*1000,
                drone->attributes.components.pwm[3].get_pwm_dc(&drone->attributes.components.pwm[3])*1000,

                // roll gains
                drone->attributes.components.controllers[ROLL].gain.kp,
                drone->attributes.components.controllers[ROLL].gain.ki,
                drone->attributes.components.controllers[ROLL].gain.kd,
                drone->attributes.components.controllers[ROLL_D].gain.kp,
                drone->attributes.components.controllers[ROLL_D].gain.ki,
                drone->attributes.components.controllers[ROLL_D].gain.kd,

                // pitch gains
                drone->attributes.components.controllers[PITCH].gain.kp,
                drone->attributes.components.controllers[PITCH].gain.ki,
                drone->attributes.components.controllers[PITCH].gain.kd,
                drone->attributes.components.controllers[PITCH_D].gain.kp,
                drone->attributes.components.controllers[PITCH_D].gain.ki,
                drone->attributes.components.controllers[PITCH_D].gain.kd,

                // yaw gains
                drone->attributes.components.controllers[YAW].gain.kp,
                drone->attributes.components.controllers[YAW].gain.ki,
                drone->attributes.components.controllers[YAW].gain.kd,
                drone->attributes.components.controllers[YAW_D].gain.kp,
                drone->attributes.components.controllers[YAW_D].gain.ki,
                drone->attributes.components.controllers[YAW_D].gain.kd,
            
                // state machine current state
                StateMachine_GetStateName(drone->attributes.state_machine.curr_state)
            );
            printf("%s", buf);
        
        }
        fflush(stdout);  // check

        vTaskDelay( pdMS_TO_TICKS( 50 ) );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


static void vLocalUartTxCmd(void *pvParameters) {
    drone_t *drone = (drone_t *) pvParameters;

    // Reset serial_data flag
    drone->attributes.global_variables.serial_data.state = false;

    // tx:{button:cross,action:press}
    char char_ptr[256];
    char * string_ptr[2];
    int char_index = 0;
    int string_index = 0;

    bool eof = false;

    for (int i = 3; i < drone->attributes.global_variables.serial_data.len; i++)
    {   
        char curr_char = drone->attributes.global_variables.serial_data.data[i];

        if(i == 3) {
            if(curr_char == '{') {
                continue;
            } else {
                ESP_LOGE("[UART Tx task]", "Frame must start with '{' character. See function %s in line %d.", __func__, __LINE__);
                vTaskDelete(NULL);
            }
        } else if(curr_char == '}') {
            eof = true;
            char_ptr[char_index] = '\0';
            string_ptr[string_index++] = strdup(char_ptr);
            break;
        } else if(curr_char == ',') {
            char_ptr[char_index] = '\0';
            string_ptr[string_index++] = strdup(char_ptr);
            char_index = 0;
        } else {
            char_ptr[char_index++] = curr_char;
        }
    }

    if(!eof) {
        ESP_LOGE("[UART Tx task]", "Frame must end with '}' character. See function %s in line %d.", __func__, __LINE__);
        vTaskDelete(NULL);
    }

    // printf("string: {%s}, len: {%d}\n", string_ptr[0], strlen(string_ptr[0]));
    // printf("string: {%s}, len: {%d}\n", string_ptr[1], strlen(string_ptr[1]));

    char button[20];
    char action[20];
    for (int i = 0; i < 2; i++)
    {
        char * token = strtok(string_ptr[i], ":");
        for (int j = 0; token != NULL; j++) {
            if (!i) {
                if (!j) {
                    if (strcmp(token, "button")) {
                        ESP_LOGE("[UART Tx task]", "Frame must be <tx:{button:my_button,action:my_action}>. See function %s in line %d.", __func__, __LINE__);
                        vTaskDelete(NULL);
                    }
                } else {
                    strcpy(button, token);
                }
            } else {
                if (!j) {
                    if (strcmp(token, "action")) {
                        ESP_LOGE("[UART Tx task]", "Frame must be <tx:{button:my_button,action:my_action}>. See function %s in line %d.", __func__, __LINE__);
                        vTaskDelete(NULL);
                    }
                } else {
                    strcpy(action, token);
                }
            }
            // printf("string_ptr[%d] (element %d): %s\n", i, j, token);
            
            token = strtok(NULL, ":");
        }
    }

    /* Declared in drone.c source file */
    extern tx_buttons_t * GlobalTxButtons;

    typedef struct tx_btns {
            char * btn_name;
            bool * tx_btn_ptr;
        } tx_btns_t;

        tx_btns_t tx_btns_arr[] = {
            {.btn_name = "cross",    .tx_btn_ptr = &(GlobalTxButtons->cross)},
            {.btn_name = "triangle", .tx_btn_ptr = &(GlobalTxButtons->triangle)},
            {.btn_name = "square",   .tx_btn_ptr = &(GlobalTxButtons->square)},
            {.btn_name = "circle",   .tx_btn_ptr = &(GlobalTxButtons->circle)},
            {.btn_name = "up",       .tx_btn_ptr = &(GlobalTxButtons->up)},
            {.btn_name = "down",     .tx_btn_ptr = &(GlobalTxButtons->down)},
            {.btn_name = "left",     .tx_btn_ptr = &(GlobalTxButtons->left)},
            {.btn_name = "right",    .tx_btn_ptr = &(GlobalTxButtons->right)},
            {.btn_name = "l1",       .tx_btn_ptr = &(GlobalTxButtons->l1)},
            {.btn_name = "l2",       .tx_btn_ptr = &(GlobalTxButtons->l2)},
            {.btn_name = "r1",       .tx_btn_ptr = &(GlobalTxButtons->r1)},
            {.btn_name = "r2",       .tx_btn_ptr = &(GlobalTxButtons->r2)},
            {.btn_name = "start",    .tx_btn_ptr = &(GlobalTxButtons->start)},
            {.btn_name = "reset",    .tx_btn_ptr = &(GlobalTxButtons->ps)}
        };

        bool found = false;
        /* If any button was pressed */
        if( !strcmp( action, "press" ) ) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = true;
                    // printf("<%s> button was pressed\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else if (!strcmp(action, "release")) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = false;
                    // printf("<%s> button was released\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else {
            ESP_LOGE("[UART Tx task]", "Action must be press/release. See function %s in line %d.", __func__, __LINE__);
            vTaskDelete(NULL);
        }

        if (!found) {
            printf("Buttons must be: <");
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++) {
                printf("%s/", tx_btns_arr[i].btn_name);
            }
            printf(">\n");
        }

    vTaskDelete(NULL);
}

static void LocalParseUartCmd(drone_t *drone) {
    const char * tx_label = "tx:";
    if(drone->attributes.global_variables.serial_data.len >= strlen(tx_label)) {
        bool tx_cmd = true;
        for (int i = 0; i < strlen(tx_label); i++)
        {
            if(drone->attributes.global_variables.serial_data.data[i] != tx_label[i]) {
                tx_cmd = false;
                break;
            }
        }
        if(tx_cmd) {
            xTaskCreatePinnedToCore( vLocalUartTxCmd, "Task5", 1024 * 3, ( void * ) ( drone ), 0, NULL, CORE_0 );
        }
    } else {
        // xTaskCreatePinnedToCore( vTaskParseCommand, "Task3", 1024 * 3, ( void * ) ( drone ), 0, NULL, CORE_0 );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void vTaskUartEvent( void * pvParameters ) {

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
                    if( !obj->attributes.global_variables.serial_data.state ) {

                        /* Data received flag HIGH */
                        obj->attributes.global_variables.serial_data.state = true;

                        /* Update data length */
                        obj->attributes.global_variables.serial_data.len = uart_event.size;

                        /* Store received data into drone's global variable */
                        uart_read_bytes( uart_num, obj->attributes.global_variables.serial_data.data, uart_event.size, 100 );
                        LocalParseUartCmd(obj);

                        /* Echo received data */
                        uart_write_bytes( uart_num, obj->attributes.global_variables.serial_data.data, uart_event.size );

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

    {.cmd_name = "pid gains",   .func = &PidGainsCmdFunc},
    {.cmd_name = "pid actions", .func = &PidActionsCmdFunc},
    {.cmd_name = "var update",  .func = &VarsUpdateCmdFunc},
    {.cmd_name = "sp update",  .func = &SpUpdateCmdFunc},
    {.cmd_name = "nvs_store", .func = &NvsStoreCmdFunc }
};


void vTaskParseCommand( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * obj = ( drone_t * ) pvParameters;

    /* Start UART cmd detection task */
    // xTaskCreatePinnedToCore( vTaskUartEvent, "Task4", 1024 * 3, ( void * ) ( obj ), 0, NULL, CORE_0 );

    while( 1 ) {

        /* If data received */
        if( obj->attributes.global_variables.serial_data.state ) {

            /* Reset state to default value */
            obj->attributes.global_variables.serial_data.state = false;

            /* Error detection flags */
            bool err = false;
            bool eof = false;

            /* Pointer to store each char of substring */
            char ptr[256] = {0};

            /* Pointer of char ( array of 4 strings ) */
            char * ptrArr[ 4 ] = { 0 };

            /* Pointer index */
            int ptrIndex = 0;

            /* Array index */
            int ptrArrIndex = 0;

            /* Loop through data received */
            for( int i = 0; i < obj->attributes.global_variables.serial_data.len; i++ ) {

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
                char currChar = obj->attributes.global_variables.serial_data.data[ i ];

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
                printf("%s\n", ptrArr[ CMD_INDEX ]);
                /* Check if recevied command matches listed commands in the array */
                if( !strcmp( ptrArr[ CMD_INDEX ], cmd_function_array[ i ].cmd_name ) ) {

                    printf("%s\n", cmd_function_array[ i ].cmd_name);
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
