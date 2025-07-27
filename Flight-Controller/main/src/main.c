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
    xTaskCreatePinnedToCore( vTaskStateMachine_Run, "APP_StateMachine", 1024 * 10, ( void * ) ( &drone ), 5, NULL, CORE_1 );

    /* Update sensor measures */
    xTaskCreatePinnedToCore( vTaskDroneMeasure, "APP_Measure", 1024 * 3, ( void * ) ( &drone ), 5, NULL, CORE_0 );

    /* Parse Bluetooth commands */
    xTaskCreatePinnedToCore( vTaskParseCommand, "APP_Parse", 1024 * 3, ( void * ) ( &drone ), 1, NULL, CORE_0 );
    
    /* Receive Uart data from UART0 */
    xTaskCreatePinnedToCore( vTaskUartEvent, "APP_Uart", 1024 * 5, ( void * ) ( &drone ), 1, NULL, CORE_0 );

    /* Print values over serial */
    xTaskCreatePinnedToCore( vTaskprint, "APP_Print", 4096, ( void * ) ( &drone ), 1, NULL, CORE_0 );

    #ifndef IGNORE_GPS
        xTaskCreatePinnedToCore( vTaskGPSEvents, "APP_GNSS", 1024 * 3, ( void * ) ( &drone.attributes.components.gnss ), 1, NULL, CORE_0 );
    #endif
}
