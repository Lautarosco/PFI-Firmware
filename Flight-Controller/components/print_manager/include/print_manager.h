#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

typedef struct {
    const char* name;
    esp_err_t (*write)(const char* data, size_t len, void* context);
    esp_err_t (*init)(void* context);
    esp_err_t (*deinit)(void* context);
    void* context;  // Custom data for each output
    bool enabled;
} print_output_t;

typedef struct print_manager print_manager_t;

typedef struct {
    esp_err_t(*register_output)(print_manager_t* manager, print_output_t* output);
    esp_err_t(*enable_output)(print_manager_t* manager, const char* name);
    esp_err_t(*disable_output)(print_manager_t* manager, const char* name);
    esp_err_t(*write)(print_manager_t* manager, const char* data, size_t len);
    esp_err_t(*set_as_system_output)(print_manager_t* manager);
} print_manager_methods;

typedef struct print_manager{
    print_output_t outputs[8];  // Max 8 outputs
    uint8_t output_count;
    print_manager_methods methods;
} print_manager_t;

// Global print manager functions
esp_err_t PrintManager(print_manager_t* manager);

// Defined print outputs
extern print_output_t serial_output;
extern print_output_t bluetooth_output;