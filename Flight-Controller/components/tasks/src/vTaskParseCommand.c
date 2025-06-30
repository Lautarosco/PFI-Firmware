#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>
#include <stdbool.h>

#include "tasks.h"
#include "drone.h"
#include "cmd_functions.h"
/**
 * @brief Command function
 */
typedef struct cmd_function {

    /* Name of the command */
    const char * cmd_name;

    /** @brief Compute the function of a given command @param drone: Address of Drone object @param arr: Array containing processed data from the original command */
    void ( * func )( drone_t * drone, char * arr[ 4 ] );

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
    drone_t * drone = ( drone_t * ) pvParameters;

    /* Start UART cmd detection task */
    // xTaskCreatePinnedToCore( vTaskUartEvent, "Task4", 1024 * 3, ( void * ) ( drone ), 0, NULL, CORE_0 );

    while( 1 ) {

        /* If data received */
        if( drone->attributes.global_variables.serial_data.state ) {

            /* Reset state to default value */
            drone->attributes.global_variables.serial_data.state = false;

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
            for( int i = 0; i < drone->attributes.global_variables.serial_data.len; i++ ) {

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
                char currChar = drone->attributes.global_variables.serial_data.data[ i ];

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
                    cmd_function_array[ i ].func( drone, ptrArr );
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
