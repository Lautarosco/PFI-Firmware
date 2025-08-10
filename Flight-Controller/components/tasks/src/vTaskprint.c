#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "tasks.h"
#include "driver/adc.h"
#include "esp_adc_cal.h" // Add this for ADC calibration functions
#include "driver/gpio.h"
#include "drone.h"
#include "string.h"

esp_err_t print_to_serial(drone_t * drone, char* buff, size_t buff_size);
esp_err_t read_battery_voltage(drone_t * drone);
esp_err_t manage_indicators(drone_t * drone);
esp_err_t decide_indicator_state(drone_t * drone);

float voltage_to_soc(float cell_voltage);


void vTaskprint( void * drone_ ) {

    drone_t* drone = ( drone_t * ) drone_;


    static char buf[1024];

    while( 1 ) {

        manage_indicators(drone);
        decide_indicator_state(drone);
        if( drone->attributes.init_ok) {
            print_to_serial(drone, buf, sizeof(buf));
            read_battery_voltage(drone);
        }

        vTaskDelay( pdMS_TO_TICKS( 50 ) );

    }
}

esp_err_t read_battery_voltage(drone_t * drone) {
    static bool init = false;
    static esp_adc_cal_characteristics_t adc_chars;
    
    if (!init) {
        init = true;
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // If ADC_ATTEN_DB_11 is deprecated, replace with ADC_ATTEN_DB_11 in your ESP-IDF version or use ADC_ATTEN_DB_12 if available.
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    }
    
    uint32_t adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
    float cell_voltage = (esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars)) * 0.001f;  // Convert to volts
    drone->attributes.components.battery.voltage = drone->attributes.components.battery.cells * cell_voltage; // Use 'cells' instead of 'number_of_cells'

    float battery_level = voltage_to_soc(cell_voltage);
    drone->attributes.components.battery.level = battery_level;

    return ESP_OK;

}

float voltage_to_soc(float cell_voltage) {

    // TODO check for battery specs
    if (cell_voltage >= 4.20) return 100.0f;
    else if (cell_voltage >= 3.95) return 75.0f + (cell_voltage - 3.95) * 25.0f / 0.25f;
    else if (cell_voltage >= 3.85) return 50.0f + (cell_voltage - 3.85) * 25.0f / 0.10f;
    else if (cell_voltage >= 3.75) return 25.0f + (cell_voltage - 3.75) * 25.0f / 0.10f;
    else if (cell_voltage >= 3.70) return 10.0f + (cell_voltage - 3.70) * 15.0f / 0.05f;
    else return 0.0f;  // Below 3.30V/cell
}


static void reset_all_led_cycles(indicators_t* indicators, TickType_t now) {
    led_t* leds[3] = {
        &indicators->power,
        &indicators->gps,
        &indicators->transmitter
    };
    
    for (int i = 0; i < 3; i++) {
        if (leds[i]->state == LED_BLINKING_PROFILE) {
            // Reset each LED's internal state for new cycle
            leds[i]->_internal.current_blink = 0;
            leds[i]->_internal.is_on = false;
            leds[i]->_internal.blink_sequence_done = false;
            leds[i]->_internal.last_toggle_time = now;
            gpio_set_level(leds[i]->gpio_pin, 0);
        }
    }
}

static void handle_blink_profile(led_t* led, TickType_t now, TickType_t cycle_start) {
    blink_profile_t* profile = &led->profile;
    
    // If we've completed our blink sequence, stay off until next cycle
    if (led->_internal.blink_sequence_done) {
        gpio_set_level(led->gpio_pin, 0);
        return;
    }
    
    // Convert milliseconds to ticks
    TickType_t on_time_ticks = pdMS_TO_TICKS(profile->on_time_ms);
    TickType_t off_time_ticks = pdMS_TO_TICKS(profile->off_time_ms);
    
    // Handle blink sequence
    TickType_t time_since_toggle = now - led->_internal.last_toggle_time;
    
    if (led->_internal.is_on) {
        // LED is currently ON, check if we should turn it OFF
        if (time_since_toggle >= on_time_ticks) {
            gpio_set_level(led->gpio_pin, 0);
            led->_internal.is_on = false;
            led->_internal.last_toggle_time = now;
            led->_internal.current_blink++;
            
            // Check if we've completed all blinks
            if (led->_internal.current_blink >= profile->blink_count) {
                led->_internal.blink_sequence_done = true;
            }
        }
    } else {
        // LED is currently OFF, check if we should turn it ON (for next blink)
        if (led->_internal.current_blink < profile->blink_count && 
            time_since_toggle >= off_time_ticks) {
            gpio_set_level(led->gpio_pin, 1);
            led->_internal.is_on = true;
            led->_internal.last_toggle_time = now;
        }
    }
}

static void handle_simple_blink(led_t* led, TickType_t now, int led_index) {
    #define BLINK_SLOW_PERIOD pdMS_TO_TICKS(500)
    #define BLINK_FAST_PERIOD pdMS_TO_TICKS(150)
    
    static TickType_t last_tick[3] = {0, 0, 0};
    
    TickType_t period = (led->state == LED_BLINKING_SLOW) ? BLINK_SLOW_PERIOD : BLINK_FAST_PERIOD;
    
    if ((now - last_tick[led_index]) >= period) {
        int current_level = gpio_get_level(led->gpio_pin);
        int next_level = !current_level;
        gpio_set_level(led->gpio_pin, next_level);
        last_tick[led_index] = now;
    }
}



esp_err_t manage_indicators(drone_t * drone) {
    TickType_t now = xTaskGetTickCount();
    indicators_t* indicators = &drone->attributes.components.indicators;
    
    // Check if we need to start a new cycle (shared across all LEDs)
    TickType_t cycle_period_ticks = pdMS_TO_TICKS(indicators->led_timing.cycle_period_ms);
    if ((now - indicators->led_timing.cycle_start_time) >= cycle_period_ticks) {
        // Start new synchronized cycle for all LEDs
        indicators->led_timing.cycle_start_time = now;
        reset_all_led_cycles(indicators, now);
    }
    
    led_t* leds[3] = {
        &indicators->power,
        &indicators->gps,
        &indicators->transmitter
    };


    for (int i = 0; i < 3; i++) {
        switch (leds[i]->state) {
            case LED_OFF:
                gpio_set_level(leds[i]->gpio_pin, 0);
                break;
                
            case LED_ON:
                gpio_set_level(leds[i]->gpio_pin, 1);
                break;
                
            case LED_BLINKING_FAST:
            case LED_BLINKING_SLOW:
                handle_simple_blink(leds[i], now, i);
                break;
                
            case LED_BLINKING_PROFILE:
                handle_blink_profile(leds[i], now, indicators->led_timing.cycle_start_time);
                break;
                
            default:
                break;
        }
    }
    return ESP_OK;
}

esp_err_t decide_indicator_state(drone_t* drone) {
    if (drone->attributes.state_machine.curr_state == ST_IDLE) {
        static bool done = false;
        if (!done) {
            blink_profile_t idle_profile = {
                .off_time_ms = 500,
                .on_time_ms = 2000,
                .blink_count = 2
            };

            drone->attributes.components.indicators.create_profile(
                &drone->attributes.components.indicators,
                2,
                2000,
                500,
                &idle_profile
            );

            drone->attributes.components.indicators.gps.set_profile(
                &drone->attributes.components.indicators.gps,
                idle_profile
            );
            drone->attributes.components.indicators.power.set_profile(
                &drone->attributes.components.indicators.power,
                idle_profile
            );
            drone->attributes.components.indicators.transmitter.set_profile(
                &drone->attributes.components.indicators.transmitter,
                idle_profile
            );
            done = true;
        }
    }
    return ESP_OK;
}


/**
 * @brief Variable format: <printer:var,%.2f\n>
 * @details Used for sending variables such as drone states, pid values, etc.
 * @example i.e, <printer:roll,%f>  This will send roll values to plotter app
 * 
 * 
 * @brief Static format: <static:var_name/var_attr,%.2f|\n>
 * @details Used for sending static variables
 * @example i.e, <static:roll/P,%f>  This will send proportional action of roll pid to plotter app.
 * Make sure that the static variables .json  file is in sync with the name sent here.
 */
esp_err_t print_to_serial(drone_t * drone, char* buff, size_t buff_size){
    
    // Dynamic
    memset(buff, 0, buff_size);
    snprintf(buff, buff_size,
        "printer:roll,%.2f|roll_d,%.2f|roll_sp,%.2f|roll_d_sp,%.2f|"
        "pitch,%.2f|pitch_d,%.2f|pitch_sp,%.2f|pitch_d_sp,%.2f|"
        "yaw,%.2f|yaw_d,%.2f|yaw_sp,%.2f|yaw_d_sp,%.2f|"
        "height,%.2f|height_sp,%.2f|"
        "dc1,%.2f|dc2,%.2f|dc3,%.2f|dc4,%.2f|"
        "gyro_x,%.2f|gyro_y,%.2f|gyro_z,%.2f|"
        "mma_in_roll,%.2f|mma_in_pitch,%.2f|mma_in_yaw,%.2f|"
        "R_X,%d|R_Y,%d|L_X,%d|L_Y,%d"
        "\n"  // end of dynamic values
        "static:roll/P,%.2f|roll/I,%.2f|roll/D,%.2f|roll/KB,%.2f|"
        "roll_d/P,%.2f|roll_d/I,%.2f|roll_d/D,%.2f|roll/KB,%.2f|"
        "pitch/P,%.2f|pitch/I,%.2f|pitch/D,%.2f|roll/KB,%.2f|"
        "pitch_d/P,%.2f|pitch_d/I,%.2f|pitch_d/D,%.2f|roll/KB,%.2f|"
        "yaw/P,%.2f|yaw/I,%.2f|yaw/D,%.2f|roll/KB,%.2f|"
        "yaw_d/P,%.2f|yaw_d/I,%.2f|yaw_d/D,%.2f|roll/KB,%.2f|"
        "ema_roll,%.2f|ema_pitch,%.2f|ema_yaw,%.2f|"
        "misc/0,%.2f|misc/1,%.2f|misc/2,%.2f|misc/3,%.2f|"
        "state,%s\n",

        // dynamic state
        drone->attributes.states.roll, drone->attributes.states.roll_dot, drone->attributes.sp.roll, drone->attributes.sp.roll_dot,
        drone->attributes.states.pitch, drone->attributes.states.pitch_dot, drone->attributes.sp.pitch, drone->attributes.sp.pitch_dot,
        drone->attributes.states.yaw, drone->attributes.states.yaw_dot, drone->attributes.sp.yaw, drone->attributes.sp.yaw_dot,
        drone->attributes.states.z, drone->attributes.sp.z,
        drone->attributes.components.pwm[0].get_pwm_dc(&drone->attributes.components.pwm[0])*1000, drone->attributes.components.pwm[1].get_pwm_dc(&drone->attributes.components.pwm[1])*1000, drone->attributes.components.pwm[2].get_pwm_dc(&drone->attributes.components.pwm[2])*1000, drone->attributes.components.pwm[3].get_pwm_dc(&drone->attributes.components.pwm[3])*1000,
        drone->attributes.components.bmi.Gyro.x, drone->attributes.components.bmi.Gyro.y, drone->attributes.components.bmi.Gyro.z,
        drone->attributes.components.mma.input[C_ROLL], drone->attributes.components.mma.input[C_PITCH], drone->attributes.components.mma.input[C_YAW],
        drone->attributes.global_variables.tx_buttons.right_stick.x,
        drone->attributes.global_variables.tx_buttons.right_stick.y,
        drone->attributes.global_variables.tx_buttons.left_stick.x,
        drone->attributes.global_variables.tx_buttons.left_stick.y,
        // roll gains
        drone->attributes.components.controllers[ROLL].gain.kp,
        drone->attributes.components.controllers[ROLL].gain.ki,
        drone->attributes.components.controllers[ROLL].gain.kd,
        drone->attributes.components.controllers[ROLL].gain.kb,

        drone->attributes.components.controllers[ROLL_D].gain.kp,
        drone->attributes.components.controllers[ROLL_D].gain.ki,
        drone->attributes.components.controllers[ROLL_D].gain.kd,
        drone->attributes.components.controllers[ROLL_D].gain.kb,

        // pitch gains
        drone->attributes.components.controllers[PITCH].gain.kp,
        drone->attributes.components.controllers[PITCH].gain.ki,
        drone->attributes.components.controllers[PITCH].gain.kd,
        drone->attributes.components.controllers[PITCH].gain.kb,

        drone->attributes.components.controllers[PITCH_D].gain.kp,
        drone->attributes.components.controllers[PITCH_D].gain.ki,
        drone->attributes.components.controllers[PITCH_D].gain.kd,
        drone->attributes.components.controllers[PITCH_D].gain.kb,

        // yaw gains
        drone->attributes.components.controllers[YAW].gain.kp,
        drone->attributes.components.controllers[YAW].gain.ki,
        drone->attributes.components.controllers[YAW].gain.kd,
        drone->attributes.components.controllers[YAW].gain.kb,

        drone->attributes.components.controllers[YAW_D].gain.kp,
        drone->attributes.components.controllers[YAW_D].gain.ki,
        drone->attributes.components.controllers[YAW_D].gain.kd,
        drone->attributes.components.controllers[YAW_D].gain.kb,

        // EMA filter coefficients
        drone->attributes.config.IIR_coeff_roll_dot,
        drone->attributes.config.IIR_coeff_pitch_dot,
        drone->attributes.config.IIR_coeff_yaw_dot,

        // Misc. variables
        drone->attributes.global_variables.misc_floats[0],  // T
        drone->attributes.global_variables.misc_floats[1],  // Amplitude
        drone->attributes.global_variables.misc_floats[2],  // Z_Sim
        drone->attributes.global_variables.misc_floats[3],  //

        // state machine current state
        StateMachine_GetStateName(drone->attributes.state_machine.curr_state)
    );
    printf("%s", buff);
    
    // if (drone->attributes.components.Tx.bluetooth_connection.is_connected) {
    //     drone->attributes.components.Tx.methods.send_bt_data(
    //         &drone->attributes.components.Tx,
    //         buff,
    //         strlen(buff)
    //     );
    // } else {
    //     printf("%s", buff);
    // }
    
    
    fflush(stdout);  // check
    return ESP_OK;
    
}
