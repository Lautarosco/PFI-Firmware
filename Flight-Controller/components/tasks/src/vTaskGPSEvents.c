#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <esp_log.h>
#include "application_layer/gnss_app_layer.h"
#include "drone.h"


void vTaskGPSEvents(void *pvParameters) {
    gnss_t *gnss = (gnss_t *) pvParameters;
    uart_event_t event;

    #ifndef CONTAINER_OF
    #define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
    #endif

    // Replace 'drone_t' and the member path with your actual types and member names
    drone_t *drone = CONTAINER_OF(gnss, drone_t, attributes.components.gnss);

    while(1) {
        if (drone->attributes.init_ok) {
            if(xQueueReceive(gnss->uart_queue, &event, portMAX_DELAY)) {
                switch(event.type) {
                    case UART_DATA:
                        if(event.size > 0) {
                            gnss->measure(gnss);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}