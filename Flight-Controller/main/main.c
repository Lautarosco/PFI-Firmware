#include <stdio.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tasks.h>
#include <esp_log.h>
#include <state_machine.h>

#include <esp_chip_info.h>

/* TESTING */
// #define PRINTER
sm_state_machine_t state_machine;   /* Only for debug purpose ( TEMPORAL REQUIREMENT ) */

/* TESTING */

void app_main( void ) {

    /* Make an instance of Drone Class */
    drone_t * drone = Drone();

    /* Check if Drone instance was successfull */
    if( drone == NULL ) {

        ESP_LOGE( "MAIN", "Failed to make an instance of Dron Class... Restarting MCU" );
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
        esp_restart();
    }

    /* Run state machine */
    xTaskCreatePinnedToCore( vTaskStateMachine_Run, "Task1", 1024 * 10, ( void * ) ( drone ), 1, NULL, CORE_1 );

    /* Update sensor measures */
    xTaskCreatePinnedToCore( vTaskDroneMeasure, "Task2", 1024 * 3, ( void * ) ( drone ), 1, NULL, CORE_0 );

    /* Parse Bluetooth commands */
    xTaskCreatePinnedToCore( vTaskParseBluetooth, "Task3", 1024 * 2, ( void * ) ( drone ), 1, NULL, CORE_0 );

    float t = 0.0f;

    while( 1 ) {

        printf( "Estado: %s\r\n", StateMachine_GetStateName( state_machine.curr_state ) );

        #ifdef PRINTER
        if( drone->attributes.init_ok) {

            float roll = drone->attributes.states.roll;
            float roll_d = drone->attributes.states.roll_dot;
            float sp_roll = drone->attributes.sp.roll;
            float sp_roll_d = drone->attributes.sp.roll_dot;

            float pid_roll = drone->attributes.components.mma->input[ C_Roll ];

            float w1 = drone->attributes.components.mma->output[ u1 ];
            float w2 = drone->attributes.components.mma->output[ u2 ];
            float w3 = drone->attributes.components.mma->output[ u3 ];
            float w4 = drone->attributes.components.mma->output[ u4 ];

            float lower_limit = drone->attributes.components.mma->limit.lower;

            printf("printer:t,%.3f|roll,%.2f|roll_d,%.2f|sp_roll,%.2f|sp_roll_d,%.2f", t, roll, roll_d, sp_roll, sp_roll_d);
            printf("|pid_roll,%.2f", pid_roll);
            printf("|lower_limit,%.2f", lower_limit);
            printf("|w1,%.2f|w2,%.2f|w3,%.2f|w4,%.2f\n", w1, w2, w3, w4);
            t += 0.01f;
        }
        #endif

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}
