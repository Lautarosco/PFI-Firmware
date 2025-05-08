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
#define UART_NUM UART_NUM_0   // Using default UART0 (USB serial)
#define UART_BUFFER_SIZE 256  // Buffer size for incoming data

// Structure to hold PID parameters
typedef struct {
    float P;
    float I;
    float D;
    float d_filter_iir_coeff;
    float i_anti_windup;
} PID_Params;

// Global PID parameters in RAM
PID_Params yaw = {1.0, 0.0, 0.0, 0.5, 10.0};
PID_Params pitch = {1.0, 0.0, 0.0, 0.5, 10.0};
PID_Params roll = {1.0, 0.0, 0.0, 0.5, 10.0};

// Function to update PID parameters
void update_pid_parameter(const char *angle, const char *param, float value) {
    PID_Params *target = NULL;
    if (strcmp(angle, "yaw") == 0) {
        target = &yaw;
    } else if (strcmp(angle, "pitch") == 0) {
        target = &pitch;
    } else if (strcmp(angle, "roll") == 0) {
        target = &roll;
    } else {
        printf("Unknown angle: %s\n", angle);
        return;
    }

    if (strcmp(param, "p") == 0) {
        target->P = value;
    } else if (strcmp(param, "i") == 0) {
        target->I = value;
    } else if (strcmp(param, "d") == 0) {
        target->D = value;
    } else if (strcmp(param, "d_filter_iir_coeff") == 0) {
        target->d_filter_iir_coeff = value;
    } else if (strcmp(param, "i_anti_windup") == 0) {
        target->i_anti_windup = value;
    } else {
        printf("Unknown parameter: %s\n", param);
        return;
    }
    printf("Updated %s %s to %.2f\n", angle, param, value);
}

// Task to handle incoming serial commands
void serial_command_task(void *pvParameters) {
    uint8_t data[UART_BUFFER_SIZE];
    while (true) {
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received string
            printf("Received command: %s\n", (char *)data);
            
            // Parse command
            char angle[10], param[20];
            float value;
            if (sscanf((char *)data, "<pid/%9[^/]/%19[^/]/%f>", angle, param, &value) == 3) {
                update_pid_parameter(angle, param, value);
            } else {
                printf("Invalid command format\n");
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

    // Start serial task
    xTaskCreate(serial_command_task, "SerialCommandTask", 4096, NULL, 2, NULL);

    float time = 0;
    float roll = 0;

    const int upper = 5;   // Upper limit
    const int lower = -5;  // Lower limit
    int step = 1;          // Direction and step size (+1 for up, -1 for down)


    while (1) {
        printf("printer:t,%.3f|roll_d,%.2f\n", time, roll);
        printf("static:yaw/P,%.2f|yaw/  I,%.2f|yaw/D,%.2f\n", yaw.P, yaw.I, yaw.D);
        vTaskDelay(pdMS_TO_TICKS(10));
        time += 0.010;

        if ((roll == upper && step == 1) || (roll == lower && step == -1)) {
            step = -step;
        }
        
        roll += step; // Update the wave value

    }

        

    
}
