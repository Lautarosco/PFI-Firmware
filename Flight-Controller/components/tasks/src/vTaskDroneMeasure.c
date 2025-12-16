#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <math.h>

#include "tasks.h"
#include "drone.h"

void vTaskDroneMeasure( void * pvParameters ) {

    /* Cast parameter into Drone object */
    drone_t * drone = ( drone_t * ) pvParameters;

    while( 1 ) {
        int64_t loop_start_time = esp_timer_get_time();

        if (drone->attributes.init_ok) {
            /* Measure attitude and update bmi sensor internal registers with respective values */
            // esp_err_t bmi_ret = drone->attributes.components.bmi.measure( &( drone->attributes.components.bmi ) );

            esp_err_t ret = drone->attributes.components.imu.measure(&(drone->attributes.components.imu));

            #define IGNORE_BMP 
            #ifndef IGNORE_BMP
            drone->attributes.components.bmp.measure((&drone->attributes.components.bmp));
            #endif

            /* Update drone states */
            if (ret == ESP_OK) {
                drone->methods.update_states( drone, drone->attributes.ts_ms );
            }

            // TODO: Make these parameters
            float MAX_ANGLE = 5.0f;  // Maximum roll angle in degrees
            float MAX_DPS = 100.0f; //  Maximum angular speed in Degrees per Second

            int r_stick_x = drone->attributes.global_variables.tx_buttons.right_stick.x;
            if (r_stick_x > 100) r_stick_x = 100;
            if (r_stick_x < -100) r_stick_x = -100;
            
            // Make the sinewave period adjustable
            float AMPLITUDE = drone->attributes.global_variables.misc_floats[0];
            float T         = drone->attributes.global_variables.misc_floats[1];

            float omega;
            float time_sec = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
            if (T > 0) {
                omega = 2.0f * M_PI / T;
                drone->attributes.sp.roll_dot = AMPLITUDE * sinf(omega * time_sec);
            } else {
                if (drone->attributes.control_mode == CONTROL_MODE_ANGLE) {
                    static float target_roll = 0.0f;
                    static float start_roll = 0.0f;
                    static float ramp_start_time = 0.0f;
                    static int ramp_active = 0;
                    
                    float new_target = (r_stick_x / 100.0f) * MAX_ANGLE;
                    float current_time = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
                    
                    if (fabsf(new_target - target_roll) > 0.1f) {
                        // Start new ramp
                        target_roll = new_target;
                        start_roll = drone->attributes.sp.roll;
                        ramp_start_time = current_time;
                        ramp_active = 1;
                    }
                    
                    if (ramp_active) {
                        float elapsed = current_time - ramp_start_time;
                        float ramp_duration = 2.0f; // 2 seconds
                        
                        if (elapsed >= ramp_duration) {
                            drone->attributes.sp.roll = target_roll;
                            ramp_active = 0;
                        } else {
                            float progress = elapsed / ramp_duration;
                            drone->attributes.sp.roll = start_roll + (target_roll - start_roll) * progress;
                        }
                    }

                } else if (drone->attributes.control_mode == CONTROL_MODE_RATE) {
                    drone->attributes.sp.roll_dot = FirstOrderIIR((r_stick_x / 100.0f) * MAX_DPS, drone->attributes.sp.roll_dot, 0.6);
                    
                }

                // If T is zero or negative, use the joystick value directly
                // drone->attributes.sp.roll = (r_stick_x / 100.0f) * MAX_ANGLE;
            }


            int r_stick_y = drone->attributes.global_variables.tx_buttons.right_stick.y;

            // #define PITCH_SIMULATE
            #ifdef PITCH_SIMULATE

            // Set flags to start right or left pulse
            static int right_pulse_active = 0;
            static int left_pulse_active = 0;
            static float right_start_time = 0.0f;
            static float left_start_time = 0.0f;

            if (r_stick_y > 0 && !right_pulse_active) {
                right_pulse_active = 1;
                right_start_time = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
            } else if (r_stick_y < 0 && !left_pulse_active) {
                left_pulse_active = 1;
                left_start_time = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
            }

            float tau = 0.2f; // time constant in seconds
            float current_time = (float)xTaskGetTickCount() / configTICK_RATE_HZ;

            if (right_pulse_active) {
                float elapsed = current_time - right_start_time;
                drone->attributes.sp.pitch = 10.0f;
                drone->attributes.states.pitch = 10.0f - 10.0f * expf(-elapsed / tau);

                // End pulse after a certain duration (e.g., 1s)
                if (elapsed > 5.0f) {
                    right_pulse_active = 0;
                    drone->attributes.states.pitch = 0.0f;
                    drone->attributes.sp.pitch = 0.0f;
                }
            } else if (left_pulse_active) {
                float elapsed = current_time - left_start_time;
                drone->attributes.sp.pitch = -10.0f;
                drone->attributes.states.pitch = -10.0f + 10.0f * expf(-elapsed / tau);

                // End pulse after a certain duration (e.g., 1s)
                if (elapsed > 5.0f) {
                    left_pulse_active = 0;
                    drone->attributes.states.pitch = 0.0f;
                    drone->attributes.sp.pitch = 0.0f;
                }
            }

            #else
                if (r_stick_y > 100) r_stick_y = 100;
                if (r_stick_y < -100) r_stick_y = -100;
                drone->attributes.sp.pitch = FirstOrderIIR((r_stick_y / 100.0f) * MAX_ANGLE, drone->attributes.sp.pitch, 0.95);
                // drone->attributes.sp.pitch = (r_stick_y / 100.0f) * MAX_ANGLE;
            #endif

            drone->attributes.sp.yaw = 0;


            int l_stick_y = drone->attributes.global_variables.tx_buttons.left_stick.y;

            if (l_stick_y > 80) l_stick_y = 70;
            if (l_stick_y < 20) l_stick_y = 20;
            drone->attributes.sp.z = (l_stick_y);

        
        }
        vTaskDelay( pdMS_TO_TICKS( drone->attributes.ts_ms ) );

        int64_t loop_end_time = esp_timer_get_time();
        int64_t loop_duration = loop_end_time - loop_start_time;
        drone->attributes.measure_cycle_time = loop_duration;

    }
}