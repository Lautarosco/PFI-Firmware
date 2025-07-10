#include <stdio.h>

#include <driver/uart.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <application_layer/gnss_app_layer.h>      /* GNSS application layer */

#include <string.h>


/* ========== Defines ========== */

#define UART_NUM            UART_NUM_1
#define UART_TX_PIN         17
#define UART_RX_PIN         16

typedef struct {
    gnss_t *gnss;
    QueueHandle_t queue;
} xGnss_wrapper_t;


static void xTask_uart_event(void *pvParameters) {
    xGnss_wrapper_t *wrapper = (xGnss_wrapper_t *) pvParameters;
    uart_event_t event;

    while(1) {
        if(xQueueReceive(wrapper->queue, &event, portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    if(event.size > 0) {
                        wrapper->gnss->measure(wrapper->gnss);
                    }
                    break;
                default:
                    break;
            }
        }
    }

    vTaskDelete(NULL);
}

/* ========== Main ========== */

void app_main(void)
{
    /* ========== UART initialization ========== */

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

    /* ========== GNSS NEO-XM ========== */

    gnss_t gnss;
    gnss_params_t gnss_params = {
        .neoxm_version = GNSS_NEO_7M,
        .gnss_protocol = GNSS_ENABLE_UBX_ONLY,
        .measRate = GNSS_MEASRATE_DEFAULT,
        .uart_port = UART_NUM,
        .dynModel = UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN,
        .static_hold_threshold = GNSS_STATIC_HOLD_DEFAULT,
    };

    Gnss(&gnss);
    esp_err_t ret = gnss.init(&gnss, gnss_params);
    if(ret != ESP_OK){
        return;
    }

    xGnss_wrapper_t uart_wrapper = {
        .gnss = &gnss,
        .queue = uart_queue
    };

    xTaskCreate(xTask_uart_event, "uart_task_event", 1024 * 4, &uart_wrapper, 10, NULL);

    while(1) {
        printf(
            "%s\t numSV: %u\t hEl: %lf cm\t hMSL: %lf cm\t vDOP: %lf\t Latitude: %lf°\t Longitude: %lf°\n",
            gnss.data.time.utc_timestamp, gnss.data.svs_data.numSV, gnss.data.position.height * 100, gnss.data.position.hMSL * 100, gnss.data.position.vDOP, gnss.data.position.lat, gnss.data.position.lon
        );

        vTaskDelay(pdMS_TO_TICKS(gnss.data.time.measRate));
    }

    return;
}


