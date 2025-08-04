#include "print_manager.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"
#include "soc/uart_reg.h"

/**
 * @brief Local print manager reference that is initialized with the adress passed by the set as system output function 
 */
static print_manager_t* g_print_manager = NULL;

esp_err_t print_manager_set_as_system_output(print_manager_t* manager);
esp_err_t print_manager_register_output(print_manager_t* manager, print_output_t* output);
esp_err_t print_manager_enable_output(print_manager_t* manager, const char* name);
esp_err_t print_manager_disable_output(print_manager_t* manager, const char* name);
esp_err_t print_manager_write(print_manager_t* manager, const char* data, size_t len);

esp_err_t PrintManager(print_manager_t* manager) {
    memset(manager, 0, sizeof(print_manager_t));
    manager->output_count = 0;

    manager->methods.register_output = print_manager_register_output;
    manager->methods.enable_output = print_manager_enable_output;
    manager->methods.disable_output = print_manager_disable_output;
    manager->methods.set_as_system_output = print_manager_set_as_system_output;
    manager->methods.write = print_manager_write;
    // manager->methods. = print_manager_;
    
    return ESP_OK;
}


esp_err_t print_manager_register_output(print_manager_t* manager, print_output_t* output) {
    if (manager->output_count >= 8) {
        return ESP_ERR_NO_MEM;
    }
    
    manager->outputs[manager->output_count] = *output;
    
    // Initialize the output if it has an init function
    if (output->init) {
        esp_err_t ret = output->init(output->context);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    manager->output_count++;
    return ESP_OK;
}

esp_err_t print_manager_enable_output(print_manager_t* manager, const char* name) {
    for (int i = 0; i < manager->output_count; i++) {
        if (strcmp(manager->outputs[i].name, name) == 0) {
            manager->outputs[i].enabled = true;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t print_manager_disable_output(print_manager_t* manager, const char* name) {
    for (int i = 0; i < manager->output_count; i++) {
        if (strcmp(manager->outputs[i].name, name) == 0) {
            manager->outputs[i].enabled = false;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t print_manager_write(print_manager_t* manager, const char* data, size_t len) {
    esp_err_t result = ESP_OK;
    
    for (int i = 0; i < manager->output_count; i++) {
        if (manager->outputs[i].enabled) {
            esp_err_t ret = manager->outputs[i].write(data, len, manager->outputs[i].context);
            if (ret != ESP_OK) {
                result = ret;
            }
        }
    }
    
    return result;
}

static int print_manager_vprintf_handler(const char* format, va_list args) {
    static char buffer[2048];
    
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len < 0) {
        return len;
    }
    
    if (g_print_manager) {
        print_manager_write(g_print_manager, buffer, len);
    }
    
    return len;
}

esp_err_t print_manager_set_as_system_output(print_manager_t* manager) {
    g_print_manager = manager;
    
    // Redirect all ESP_LOG calls
    esp_log_set_vprintf(print_manager_vprintf_handler);

    return ESP_OK;
}

// Override the system printf function
int printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    int result = print_manager_vprintf_handler(format, args);
    va_end(args);
    return result;
}

/* DEFAULT SERIAL PRINT */
esp_err_t serial_write(const char* data, size_t len, void* context) {
    // COMPLETELY RAW UART write using direct register access
    // This bypasses ALL ESP-IDF functions that could trigger logging
    for (size_t i = 0; i < len; i++) {
        // Wait for TX FIFO to have space
        while ((REG_READ(UART_STATUS_REG(UART_NUM_0)) & UART_TXFIFO_CNT_M) >= 128) {
            // Wait
        }
        // Write byte directly to TX FIFO register
        REG_WRITE(UART_FIFO_REG(UART_NUM_0), data[i]);
    }
    return ESP_OK;
}

print_output_t serial_output = {
    .name = "Serial",
    .write = serial_write,
    .context = NULL
};

