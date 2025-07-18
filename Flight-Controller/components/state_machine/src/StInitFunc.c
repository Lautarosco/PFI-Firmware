#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "state_machine.h"
#include "drone.h"
#include "button_helper.h"


void StInitFunc( drone_t * drone ) {

    ESP_ERROR_CHECK( drone->methods.init( drone ) );

    // tomar ACC_AVG_NUM mediciones de acelerómetro y promediarlas
    float sum = 0;
    #define ACC_AVG_NUM 20.0
    for (int i = 0; i < ACC_AVG_NUM; i++) {
        // leer acelerómetro y calcular roll_acc
        float roll_acc = atan2( drone->attributes.components.bmi.Acc.y, drone->attributes.components.bmi.Acc.z ) * ( 180.0f / M_PI );
        printf("Roll ACC: %.2f\n", roll_acc);
        sum += roll_acc;
        vTaskDelay(pdMS_TO_TICKS(1)); // espera entre muestras
    }
    printf("Sum: %.2f\n", sum);
    drone->attributes.states.roll = sum / ACC_AVG_NUM;
}