#include <stdio.h>

#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <application_layer/gnss_app_layer.h>      /* GNSS application layer */

#define UART_NUM            UART_NUM_1
#define UART_TX_PIN         17
#define UART_RX_PIN         16


/**
 * Protocolo UBX
 * 
 * Trama: [<Class><ID><Payload_length><Payload><Checksum>]
 * Class: 8-bit
 * ID: 8-bit
 * Payload_length: unsigned 16-bit (little-endian format)
 * Payload: Variable length
 * Checksum: 16-bit
 */

void vTaskUartEvt(void *uart_queue);

void app_main(void)
{
    const int uart_buffer_size = 1024;
    QueueHandle_t uart_queue;
    if(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0) != ESP_OK) {
        printf("Failed to install UART driver\n");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    if(uart_param_config(UART_NUM, &uart_config) != ESP_OK) {
        printf("Failed to configure UART parameters\n");
        uart_driver_delete(UART_NUM);
        return;
    }

    
    if(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        printf("Failed to set UART pins\n");
        uart_driver_delete(UART_NUM);
        return;
    }

    printf("UART driver installed and configured successfully\n");

    // xTaskCreate(vTaskUartEvt, "uart_evt_task", 1024 * 3, (void *) &uart_queue, 1, NULL);

    /*
    uint8_t data[256];
    printf("GSV Activado\n");
    for(int i = 0; i < 4; i++) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data), 100);
        printf("[%d] Chars (%d bytes):\n", i, len);
        for(int i = 0; i < len; i++) {
            printf("%c ", data[i]);
        }
        printf("\n\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    */

    gnss_t gnss;
    gnss_params_t gnss_params = {
        .uart_port = UART_NUM,
        .gnss_protocol = GNSS_ENABLE_UBX_ONLY
    };
    Gnss(&gnss);
    gnss.init(&gnss, gnss_params);

    while(1) {
        gnss_hwl_read_nav_pvt(gnss.uart_port);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskUartEvt(void *uart_queue) {
    QueueHandle_t *queue = (QueueHandle_t *)uart_queue;

    uint8_t data[256];
    uart_event_t uart_evt;

    while(1) {
        if(xQueueReceive(*queue, (void *) &uart_evt, portMAX_DELAY)) {
            switch(uart_evt.type) {
                case UART_DATA:
                    printf("Received %d bytes of data\n", uart_evt.size);
                    int len = uart_read_bytes(UART_NUM, data, uart_evt.size, 100);
                    printf("%s", data);
                    uart_flush(UART_NUM);
                    break;
                case UART_FIFO_OVF:
                    printf("UART FIFO overflow\n");
                    break;
                case UART_BUFFER_FULL:
                    printf("UART buffer full\n");
                    break;
                case UART_BREAK:
                    printf("UART break detected\n");
                    break;
                default:
                    printf("Unknown UART event type: %d\n", uart_evt.type);
                    break;
            }
        }
    }
}
