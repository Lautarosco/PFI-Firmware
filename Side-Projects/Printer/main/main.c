#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/uart.h"

#define TIMER_INTERVAL_MS 10  // Timer interval in milliseconds
#define ROLL_AMPLITUDE 1.0    // Amplitude of the sine wave
#define ROLL_FREQUENCY 0.5    // Frequency of the sine wave in Hz

#define UART_NUM UART_NUM_0   // Using default UART0 (USB serial)
#define UART_BUFFER_SIZE 256  // Buffer size for incoming data

void print_data_task(void *pvParameters) {
    srand((unsigned int)time(NULL)); // Seed the random number generator

    while (true) {
        // Get the current time in seconds
        int64_t time_us = esp_timer_get_time();
        float t = time_us / 1000000.0;

        // Generate random values for w1 and w2 (between 0 and 1)
        float w1 = (float)rand() / RAND_MAX;
        float w2 = (float)rand() / RAND_MAX;

        // Generate the roll value as a sine wave
        float roll = ROLL_AMPLITUDE * sin(2 * M_PI * ROLL_FREQUENCY * t);

        // Print the values in the desired format
        printf("printer:t,%.3f|w1,%.2f|w2,%.2f|roll,%.2f\n", t, w1, w2, roll);

        // Wait for the next interval
        vTaskDelay(pdMS_TO_TICKS(TIMER_INTERVAL_MS));
    }
}

// Task to handle incoming serial commands
void serial_command_task(void *pvParameters) {
    uint8_t data[UART_BUFFER_SIZE];
    while (true) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received string
            printf("Received command: %s\n", (char *)data);

            // Handle commands
            if (strncmp((char *)data, "RESET", 5) == 0) {
                printf("Resetting system...\n");
                esp_restart();
            } else if (strncmp((char *)data, "STATUS", 6) == 0) {
                printf("System is running fine.\n");
            } else {
                printf("Unknown command: %s\n", (char *)data);
            }
        }
    }
}

void app_main(void) {
    // Initialize UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);

    // Start tasks
    xTaskCreate(print_data_task, "PrintDataTask", 4096, NULL, 1, NULL);
    xTaskCreate(serial_command_task, "SerialCommandTask", 4096, NULL, 2, NULL);
}
