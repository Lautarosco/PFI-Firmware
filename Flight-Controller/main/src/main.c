#include <stdio.h>
#include <drone.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <tasks.h>
#include <esp_log.h>

#include <esp_chip_info.h>

void app_main( void ) {

    ESP_LOGI("MEM", "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI("MEM", "Minimum free heap: %lu", esp_get_minimum_free_heap_size());

    // Make an instance of Drone Class
    static drone_t drone;
    Drone(&drone);

    /* Run state machine */     
    xTaskCreatePinnedToCore( vTaskStateMachine_Run, "Task1", 1024 * 10, ( void * ) ( &drone ), 1, NULL, CORE_1 );

    /* Update sensor measures */
    xTaskCreatePinnedToCore( vTaskDroneMeasure, "Task2", 1024 * 3, ( void * ) ( &drone ), 1, NULL, CORE_0 );

    /* Parse Bluetooth commands */
    xTaskCreatePinnedToCore( vTaskParseCommand, "Task3", 1024 * 3, ( void * ) ( &drone ), 1, NULL, CORE_0 );

    /* Print values over serial */
    xTaskCreatePinnedToCore( vTaskprint, "Task4", 4096, ( void * ) ( &drone ), 1, NULL, CORE_0 );

    xTaskCreatePinnedToCore( vTaskUartEvent, "Task5", 1024 * 5, ( void * ) ( &drone ), 1, NULL, CORE_0 );
}
