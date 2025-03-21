#include <stdio.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tasks.h>
#include <esp_log.h>
#include <state_machine.h>

#include <esp_chip_info.h>

/* TESTING */
#define PRINTER
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
    xTaskCreatePinnedToCore( vTaskParseCommand, "Task3", 1024 * 3, ( void * ) ( drone ), 1, NULL, CORE_0 );

    /* Print values over serial */
    xTaskCreatePinnedToCore( vTaskprint, "Task4", 1024*2, ( void * ) ( drone ), 1, NULL, CORE_0 );
}
