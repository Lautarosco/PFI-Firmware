#include <stdio.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tasks.h>
<<<<<<< Updated upstream

=======
#include <esp_log.h>
#include "state_machine.h"
#include <esp_chip_info.h>
>>>>>>> Stashed changes

void app_main( void ) {

    /* Make an instance of Drone Class */
    drone_t * drone = Drone();

    /* Run state machine */
    xTaskCreatePinnedToCore( vTaskStateMachine_Run, "Task1", 1024 * 4, ( void * ) ( drone ), 1, NULL, CORE_1 );

    /* Update sensor measures */
    xTaskCreatePinnedToCore( vTaskDroneMeasure, "Task2", 1024 * 2, ( void * ) ( drone ), 1, NULL, CORE_0 );

    /* Parse Bluetooth commands */
    xTaskCreatePinnedToCore( vTaskParseBluetooth, "Task3", 1024 * 2, ( void * ) ( drone ), 1, NULL, CORE_0 );
<<<<<<< Updated upstream
=======

    while( 1 ) {

        printf("Roll: %2.2f\r\n", drone->attributes.states.roll );
        printf("Current state: %s.\r\n", state_names[state_machine.curr_state]);

        printf("Acc [X Y Z]: %.2f - %.2f - %.2f", drone->attributes.components.bmi.Acc.x, drone->attributes.components.bmi.Acc.y, drone->attributes.components.bmi.Acc.z);
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
>>>>>>> Stashed changes
}
