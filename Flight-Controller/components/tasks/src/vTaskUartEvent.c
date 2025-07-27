#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>

#include "tasks.h"
#include <uart_init.h>
#include "drone.h"

static void vLocalUartTxCmd(void *pvParameters);
static void LocalParseUartCmd(drone_t *drone);

void vTaskUartEvent( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * drone = ( drone_t * ) pvParameters;

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
                    if( !drone->attributes.global_variables.serial_data.state ) {

                        /* Data received flag HIGH */
                        drone->attributes.global_variables.serial_data.state = true;

                        /* Update data length */
                        drone->attributes.global_variables.serial_data.len = uart_event.size;

                        /* Store received data into drone's global variable */
                        uart_read_bytes( uart_num, drone->attributes.global_variables.serial_data.data, uart_event.size, 100 );
                        
                        /* Echo received data */
                        // uart_write_bytes( uart_num, drone->attributes.global_variables.serial_data.data, uart_event.size );

                        /* Clear UART Rx buffer */
                        uart_flush( uart_num );
                    }

                    else{
                        char *task_name = pcTaskGetName(NULL);
                        ESP_LOGW( task_name, "[ %s ] Bluetooth command is currently being prossesed.", __func__ );  // TODO: add a queue for commands instead of raising warning
                    }

                default:
                    break;
            }
        }
    }
}
