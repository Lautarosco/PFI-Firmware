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
                        LocalParseUartCmd(drone);

                        /* Echo received data */
                        uart_write_bytes( uart_num, drone->attributes.global_variables.serial_data.data, uart_event.size );

                        /* Clear UART Rx buffer */
                        uart_flush( uart_num );
                    }

                    else{

                        ESP_LOGW( "TASK4", "[ %s ] Bluetooth command is currently being prossesed.", __func__ );
                    }

                default:
                    break;
            }
        }
    }
}

static void LocalParseUartCmd(drone_t *drone) {
    const char * tx_label = "tx:";
    if(drone->attributes.global_variables.serial_data.len >= strlen(tx_label)) {
        bool tx_cmd = true;
        for (int i = 0; i < strlen(tx_label); i++)
        {
            if(drone->attributes.global_variables.serial_data.data[i] != tx_label[i]) {
                tx_cmd = false;
                break;
            }
        }
        if(tx_cmd) {
            xTaskCreatePinnedToCore( vLocalUartTxCmd, "Task5", 1024 * 3, ( void * ) ( drone ), 0, NULL, CORE_0 );
        }
    } else {
        // xTaskCreatePinnedToCore( vTaskParseCommand, "Task3", 1024 * 3, ( void * ) ( drone ), 0, NULL, CORE_0 );
    }
}

static void vLocalUartTxCmd(void *pvParameters) {
    drone_t *drone = (drone_t *) pvParameters;

    // Reset serial_data flag
    drone->attributes.global_variables.serial_data.state = false;

    // tx:{button:cross,action:press}
    char char_ptr[256];
    char * string_ptr[2];
    int char_index = 0;
    int string_index = 0;

    bool eof = false;

    for (int i = 3; i < drone->attributes.global_variables.serial_data.len; i++)
    {   
        char curr_char = drone->attributes.global_variables.serial_data.data[i];

        if(i == 3) {
            if(curr_char == '{') {
                continue;
            } else {
                ESP_LOGE("[UART Tx task]", "Frame must start with '{' character. See function %s in line %d.", __func__, __LINE__);
                vTaskDelete(NULL);
            }
        } else if(curr_char == '}') {
            eof = true;
            char_ptr[char_index] = '\0';
            string_ptr[string_index++] = strdup(char_ptr);
            break;
        } else if(curr_char == ',') {
            char_ptr[char_index] = '\0';
            string_ptr[string_index++] = strdup(char_ptr);
            char_index = 0;
        } else {
            char_ptr[char_index++] = curr_char;
        }
    }

    if(!eof) {
        ESP_LOGE("[UART Tx task]", "Frame must end with '}' character. See function %s in line %d.", __func__, __LINE__);
        vTaskDelete(NULL);
    }

    // printf("string: {%s}, len: {%d}\n", string_ptr[0], strlen(string_ptr[0]));
    // printf("string: {%s}, len: {%d}\n", string_ptr[1], strlen(string_ptr[1]));

    char button[20];
    char action[20];
    for (int i = 0; i < 2; i++)
    {
        char * token = strtok(string_ptr[i], ":");
        for (int j = 0; token != NULL; j++) {
            if (!i) {
                if (!j) {
                    if (strcmp(token, "button")) {
                        ESP_LOGE("[UART Tx task]", "Frame must be <tx:{button:my_button,action:my_action}>. See function %s in line %d.", __func__, __LINE__);
                        vTaskDelete(NULL);
                    }
                } else {
                    strcpy(button, token);
                }
            } else {
                if (!j) {
                    if (strcmp(token, "action")) {
                        ESP_LOGE("[UART Tx task]", "Frame must be <tx:{button:my_button,action:my_action}>. See function %s in line %d.", __func__, __LINE__);
                        vTaskDelete(NULL);
                    }
                } else {
                    strcpy(action, token);
                }
            }
            // printf("string_ptr[%d] (element %d): %s\n", i, j, token);
            
            token = strtok(NULL, ":");
        }
    }

    /* Declared in drone.c source file */
    extern tx_buttons_t * GlobalTxButtons;

    typedef struct tx_btns {
            char * btn_name;
            bool * tx_btn_ptr;
        } tx_btns_t;

        tx_btns_t tx_btns_arr[] = {
            {.btn_name = "cross",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.cross)},
            {.btn_name = "triangle", .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.triangle)},
            {.btn_name = "square",   .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.square)},
            {.btn_name = "circle",   .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.circle)},
            {.btn_name = "up",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.up)},
            {.btn_name = "down",     .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.down)},
            {.btn_name = "left",     .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.left)},
            {.btn_name = "right",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.right)},
            {.btn_name = "l1",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.l1)},
            {.btn_name = "l2",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.l2)},
            {.btn_name = "r1",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.r1)},
            {.btn_name = "r2",       .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.r2)},
            {.btn_name = "start",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.start)},
            {.btn_name = "reset",    .tx_btn_ptr = &(drone->attributes.global_variables.tx_buttons.ps)}
        };

        bool found = false;
        /* If any button was pressed */
        if( !strcmp( action, "press" ) ) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = true;
                    // printf("<%s> button was pressed\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else if (!strcmp(action, "release")) {
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++)
            {
                if(!strcmp(button, tx_btns_arr[i].btn_name)) {
                    (*tx_btns_arr[i].tx_btn_ptr) = false;
                    // printf("<%s> button was released\n", tx_btns_arr[i].btn_name);
                    found = true;
                    break;
                }
            }
        } else {
            ESP_LOGE("[UART Tx task]", "Action must be press/release. See function %s in line %d.", __func__, __LINE__);
            vTaskDelete(NULL);
        }

        if (!found) {
            printf("Buttons must be: <");
            for (int i = 0; i < ((sizeof(tx_btns_arr)) / (sizeof(tx_btns_arr[0]))); i++) {
                printf("%s/", tx_btns_arr[i].btn_name);
            }
            printf(">\n");
        }

    vTaskDelete(NULL);
}