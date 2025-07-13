#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "state_machine.h"
#include "drone.h"
#include "button_helper.h"

static void toggle_motor(drone_t* drone, int pwm_num);
static void increase_motor_duty(drone_t* drone, int pwm_num);

static void toggle_motor(drone_t* drone, int pwm_num) {

    float current_pwm_dc = drone->attributes.components.pwm[ pwm_num ].get_pwm_dc(&drone->attributes.components.pwm[ pwm_num ]);
    printf("Current PWM DC for motor %d: %.2f\n", pwm_num, current_pwm_dc);

    float dc_min = drone->attributes.components.pwm[pwm_num].dc_min;
    float dc_target = dc_min + ((drone->attributes.components.pwm[pwm_num].dc_max - dc_min) / 2); // Set target to half between min and max
    float avg = (dc_min + dc_target) / 2.0;

    if (current_pwm_dc > avg) {

        drone->attributes.components.pwm[ pwm_num ].set_pwm_dc(
            &drone->attributes.components.pwm[ pwm_num ],
            dc_min  // Apagado
        );
        printf("Motor %d OFF with duty cycle: %f \n", pwm_num, dc_min);

    } else {
        // float dc_target = dc_min + ((dc_max - dc_min) / 2); 
        
        drone->attributes.components.pwm[ pwm_num ].set_pwm_dc(
            &drone->attributes.components.pwm[ pwm_num ],
            dc_target  // Velocidad minima
        );
        printf("Motor %d ON with duty cycle: %f \n", pwm_num, dc_target);
    }
}

// Increase motor duty cycle to next value in a predefined array
static void increase_motor_duty(drone_t* drone, int pwm_num) {
    // Define the array of duty cycle values (example values, adjust as needed)
    static const float duty_steps[] = {0.0f, 0.15f, 0.25f, 0.35f, 0.45f, 0.55f, 0.65f, 0.75f, 0.85f, 0.95f, 1.0f};
    static const int num_steps = sizeof(duty_steps) / sizeof(duty_steps[0]);

    float dc_min = drone->attributes.components.pwm[pwm_num].dc_min;
    float dc_max = drone->attributes.components.pwm[pwm_num].dc_max;

    float current_pwm_dc = drone->attributes.components.pwm[pwm_num].get_pwm_dc(&drone->attributes.components.pwm[pwm_num]);
    // Normalize current duty cycle to [0,1] range
    float norm = (current_pwm_dc - dc_min) / (dc_max - dc_min);

    int idx = 0;
    // Find the current step index
    for (int i = 0; i < num_steps; ++i) {
        if (norm < duty_steps[i]) {
            idx = i;
            break;
        }
        idx = i;
    }
    // Move to next step, but don't exceed array bounds
    if (idx < num_steps - 1) {
        idx++;
    }
    float new_norm = duty_steps[idx];
    float new_dc = dc_min + new_norm * (dc_max - dc_min);

    drone->attributes.components.pwm[pwm_num].set_pwm_dc(
        &drone->attributes.components.pwm[pwm_num],
        new_dc
    );
    printf("Motor %d increased to duty cycle: %f (step %d)\n", pwm_num, new_dc, idx);
}

void StVibrationCheck(drone_t* drone) {
    float alpha = 0.1f; // Smoothing factor (0 < alpha < 1)

    static bool initialized = false;
    static float acc_av_vector = 0;
    static float vibration_total_result = 0;
    static int vibration_counter = 0;
    
    if (pressed(drone, EV_SQUARE) && initialized) {
        
        /* Turn off motors */
        for (int pwm_num = 0; pwm_num < 4; pwm_num++) {
            drone->attributes.components.pwm[pwm_num].set_pwm_dc(
                &drone->attributes.components.pwm[pwm_num],
                drone->attributes.components.pwm[ pwm_num ].dc_min);
        }
        initialized = false;  // Reset for next entry
        printf("Exiting vibration check mode\n");
        drone->attributes.request_state_transition = true;
        drone->attributes.requested_transition_event = EV_SQUARE;
        return;
    }
    
    // Initialize with first reading
    if (!initialized) {
        acc_av_vector = sqrt(
            drone->attributes.components.bmi.Acc.x * drone->attributes.components.bmi.Acc.x +
            drone->attributes.components.bmi.Acc.y * drone->attributes.components.bmi.Acc.y +
            drone->attributes.components.bmi.Acc.z * drone->attributes.components.bmi.Acc.z
        );

        acc_av_vector = 0;
        vibration_total_result = 0;
        vibration_counter = 0;
        initialized = true;

    }

    // Motor control
    if(pressed(drone, EV_UP)) toggle_motor(drone, 0);
    else if(pressed(drone, EV_DOWN)) toggle_motor(drone, 1);
    else if(pressed(drone, EV_LEFT)) toggle_motor(drone, 2);
    else if(pressed(drone, EV_RIGHT)) toggle_motor(drone, 3);

    if (pressed(drone, EV_R1)) increase_motor_duty(drone, 0);
    if (pressed(drone, EV_R2)) increase_motor_duty(drone, 1);
    if (pressed(drone, EV_L1)) increase_motor_duty(drone, 2);
    if (pressed(drone, EV_L2)) increase_motor_duty(drone, 3);

    
    // Vibration calculation
    float current_acc_vector = sqrtf(
        drone->attributes.components.bmi.Acc.x * drone->attributes.components.bmi.Acc.x +
        drone->attributes.components.bmi.Acc.y * drone->attributes.components.bmi.Acc.y +
        drone->attributes.components.bmi.Acc.z * drone->attributes.components.bmi.Acc.z
    );
    
    acc_av_vector = (alpha) * current_acc_vector + (1-alpha) * acc_av_vector;
    vibration_total_result += fabsf(current_acc_vector - acc_av_vector);
    if (++vibration_counter >= 20) {
        ESP_LOGI("VIBRATION", "Vibration: %.2f", vibration_total_result / 20);
        vibration_total_result = 0;
        vibration_counter = 0;
    }
}