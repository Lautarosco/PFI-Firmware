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
            float MAX_ROLL = 5.0f;  // Maximum roll angle in degrees
            float MAX_PITCH = 5.0f; // Maximum pitch angle in degrees

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
                // If T is zero or negative, use the joystick value directly
                drone->attributes.sp.roll = FirstOrderIIR((r_stick_x / 100.0f) * MAX_ROLL, drone->attributes.sp.roll, drone->attributes.ts_ms/1000.0, 0.95);
                // drone->attributes.sp.roll = (r_stick_x / 100.0f) * MAX_ROLL;
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
                drone->attributes.sp.pitch = FirstOrderIIR((r_stick_y / 100.0f) * MAX_PITCH, drone->attributes.sp.pitch, drone->attributes.ts_ms/1000.0, 0.95);
                // drone->attributes.sp.pitch = (r_stick_y / 100.0f) * MAX_PITCH;
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