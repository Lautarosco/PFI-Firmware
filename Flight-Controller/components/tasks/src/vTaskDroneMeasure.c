#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>

#include "tasks.h"
#include "drone.h"

void vTaskDroneMeasure( void * pvParameters ) {

    printf("Entering vTaskDroneMeasure\n");
    /* Cast parameter into Drone object */
    drone_t * drone = ( drone_t * ) pvParameters;
    printf("Casted drone object\n");
    printf("init_ok: ");
    printf("%d\n", drone->attributes.init_ok);
    
    while( 1 ) {

        if (drone->attributes.init_ok) {
            /* Measure attitude and update bmi sensor internal registers with respective values */
            drone->attributes.components.bmi.measure( &( drone->attributes.components.bmi ) );

            /* Update drone states */
            drone->methods.update_states( drone, 10 );

            /* Update sp */
            /*printf("Analog stick values: LX\t LY\t RX\t RY\n%d\t, %d\t, %d\t, %d\n",
                drone->attributes.global_variables.tx_buttons.left_stick.x,
                drone->attributes.global_variables.tx_buttons.left_stick.y,
                drone->attributes.global_variables.tx_buttons.right_stick.x,
                drone->attributes.global_variables.tx_buttons.right_stick.y
            );*/

            // TODO: Make these parameters
            float MAX_ROLL = 10.0f;  // Maximum roll angle in degrees
            float MAX_PITCH = 10.0f; // Maximum pitch angle in degrees

            int r_stick_x = drone->attributes.global_variables.tx_buttons.right_stick.x;
            if (r_stick_x > 100) r_stick_x = 100;
            if (r_stick_x < -100) r_stick_x = -100;
            
            // Make the sinewave period adjustable
            float AMPLITUDE = drone->attributes.global_variables.misc_floats[0];
            float T = drone->attributes.global_variables.misc_floats[1];
            float omega;
            float time_sec = (float)xTaskGetTickCount() / configTICK_RATE_HZ;
            if (T > 0) {
                omega = 2.0f * M_PI / T;
                drone->attributes.sp.roll = AMPLITUDE * sinf(omega * time_sec);
            } else {
                // If T is zero or negative, use the joystick value directly
                drone->attributes.sp.roll = (r_stick_x / 100.0f) * MAX_ROLL;
            }

            int r_stick_y = drone->attributes.global_variables.tx_buttons.right_stick.y;
            if (r_stick_y > 100) r_stick_y = 100;
            if (r_stick_y < -100) r_stick_y = -100;
            drone->attributes.sp.pitch = (r_stick_y / 100.0f) * MAX_PITCH;

            drone->attributes.sp.yaw = 0;
            drone->attributes.sp.z = 0;
        }
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
    

}