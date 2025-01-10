#include <stdio.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tasks.h>
#include <esp_log.h>

#include <esp_chip_info.h>

void app_main( void ) {

    esp_chip_info_t info;
    esp_chip_info( &info );

    printf( "MCU model: %d\r\n", info.model );
    printf( "MCU revision: %d\r\n", info.revision );

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
    xTaskCreatePinnedToCore( vTaskDroneMeasure, "Task2", 1024 * 2, ( void * ) ( drone ), 1, NULL, CORE_0 );

    /* Parse Bluetooth commands */
    xTaskCreatePinnedToCore( vTaskParseBluetooth, "Task3", 1024 * 2, ( void * ) ( drone ), 1, NULL, CORE_0 );

    while( 1 ) {

        printf( "x: %d\r\n", drone->attributes.global_variables.tx_buttons->cross );

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}
