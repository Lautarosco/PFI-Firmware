#include <drone.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <math.h>
#include <esp_log.h>

const char * DRONE_TAG = "DRONE";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public variables */

/**
 * @brief Pointer to buttons global variable of a Drone object ( used in transmitter component | transmitter_structs.c source file )
 */
tx_buttons_t * GlobalTxButtons;

/**
 * @brief Pointer to Bluetooth data global variable of a Drone object ( used in ps3 component | ps3_spp.c source file )
 */
BluetoothData_t * GlobalBluetoothData;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Drone Class generic configs
 */
static const drone_cfg_t DroneConfigs = {
    .roll = {
        .error = 0.0f,
        .Q     = 0.01f,
        .R     = 0.5f,
    },
    .pitch = {
        .error = 0.0f,
        .Q     = 0.01f,
        .R     = 0.5f,
    },
    .yaw = {
        .error = 0.0f,
        .Q     = 0.01f,
        .R     = 0.5f,
    },
    .IIR_coeff_roll_dot  = 0.9f,
    .IIR_coeff_pitch_dot = 0.9f,
    .IIR_coeff_yaw_dot   = 0.9f,
    .imu_cfg = {
        .imu_i2c_cfg = {
            .address = BMI160_ADDR,
            .scl     = 22,
            .sda     = 21,
        },
        .acc_mode    = BMI160_CMD_ACC_NORMAL_MODE,
        .acc_freq    = BMI160_ACC_CONF_100HZ_NORMAL,
        .acc_range   = BMI160_ACC_RANGE_4G,
        .gyro_mode   = BMI160_CMD_GYRO_NORMAL_MODE,
        .gyro_freq   = BMI160_GYRO_CONF_100HZ_NORMAL,
        .gyro_range  = BMI160_GYRO_RANGE_250DPS,
        .gyro_offset = {
            .x = 6.80f,
            .y = -2.15f,
            .z = 3.28f,
        },
    },
    .esp_mac_addr = { 0xf0, 0xf0, 0x02, 0x43, 0x53, 0x53 },
    .pwm_cfg = {
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_26,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_0,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_5,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_1,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_15,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_2,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        },
        {
            .timer_cfg = {
                .speed_mode      = LEDC_LOW_SPEED_MODE,
                .timer_num       = LEDC_TIMER_0,
                .freq_hz         = 50,
                .duty_resolution = LEDC_TIMER_20_BIT,
                .clk_cfg         = LEDC_APB_CLK
            },
            .channel_cfg = {
                .gpio_num            = GPIO_NUM_18,
                .speed_mode          = LEDC_LOW_SPEED_MODE,
                .channel             = LEDC_CHANNEL_3,
                .intr_type           = LEDC_INTR_DISABLE,
                .timer_sel           = LEDC_TIMER_0,
                .duty                = PULSE_WIDTH_TO_DUTY( 1.1, 50, 20 ),
                .hpoint              = 0,
                .flags.output_invert = 0
            },
            .Ton_min = 1.1,
            .Ton_max = 1.94
        }
    },
    .ControllersConfigs = {
        {
            .tag = z,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
        {
            .tag = roll,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 1, .ki = 0, .kd = 0 }
        },
        {
            .tag = pitch,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
        {
            .tag = yaw,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
        {
            .tag = roll_dot,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
        {
            .tag = pitch_dot,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
        {
            .tag = yaw_dot,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 }
        },
    }
};


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize GPIOx
 * @param GPIOx: GPIO num
 * @param io_mode: Input / output
 * @param up_mode: Pull-up or Pull-down resistor
 * @retval true if success else false
 */
inline bool GPIO_INIT( gpio_num_t GPIOx, gpio_mode_t io_mode, gpio_pull_mode_t up_mode ) {
    gpio_reset_pin( GPIOx ); return ( !gpio_set_direction( GPIOx, io_mode ) && !gpio_set_pull_mode( GPIOx, up_mode ) );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Compute first order IIR filter algorithm
 * @param in: New input to filter
 * @param out: Previous output of filter 
 * @param a: Filter coefficient, 0 <= a <= 1
 * @retval float
 */
static float FirstOrderIIR( float in, float out, float a ) {
    if( ( a < 0.0f ) || ( a > 1.0f ) )  /* Assume default value if 'a' parameter is wrong */
        a =  0.5f;

    return ( ( 1 - a ) * in ) + ( a * out );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Kalman filter for drone's attitude
 * @param drone: Address of drone object
 * @param ts: Sampling time in milliseconds
 * @retval none
 */
static void Kalman( drone_t * obj, float ts ) {
    obj->attributes.states.roll  += ( ts / 1000.0f ) * obj->attributes.components.bmi->Gyro.x;
    obj->attributes.states.pitch += ( ts / 1000.0f ) * obj->attributes.components.bmi->Gyro.y;
    obj->attributes.states.yaw   += ( ts / 1000.0f ) * obj->attributes.components.bmi->Gyro.z;

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();

    DroneConfigs.roll.error  += DroneConfigs.roll.Q;
    DroneConfigs.pitch.error += DroneConfigs.pitch.Q;
    // DroneConfigs.yaw.error   += DroneConfigs.yaw.Q;

    float roll_sensor  = atan2( obj->attributes.components.bmi->Gyro.y, obj->attributes.components.bmi->Gyro.z ) * ( 180.0f / M_PI );
    float pitch_sensor = -atan2(
        -obj->attributes.components.bmi->Gyro.x,
        sqrt( ( obj->attributes.components.bmi->Gyro.y * obj->attributes.components.bmi->Gyro.y ) +
              ( obj->attributes.components.bmi->Gyro.z * obj->attributes.components.bmi->Gyro.z ) )
    ) * ( 180.0f / M_PI );
    // float yaw_sensor   =  * ( 180 / M_PI );

    float roll_K  = DroneConfigs.roll.error  / ( DroneConfigs.roll.error  + DroneConfigs.roll.R );
    float pitch_K = DroneConfigs.pitch.error / ( DroneConfigs.pitch.error + DroneConfigs.pitch.R );
    // float yaw_K   = DroneConfigs.yaw.error   / ( DroneConfigs.yaw.error   + DroneConfigs.yaw.R );

    obj->attributes.states.roll  = ( obj->attributes.states.roll  * ( 1 - roll_K ) )  + ( roll_K  * roll_sensor );
    obj->attributes.states.pitch = ( obj->attributes.states.pitch * ( 1 - pitch_K ) ) + ( pitch_K * pitch_sensor );
    // drone->attributes.states.yaw   = ( drone->attributes.states.yaw   * ( 1 - yaw_K ) )   + ( yaw_K   * yaw_sensor );

    DroneConfigs.roll.error  *= ( 1 - roll_K );
    DroneConfigs.pitch.error *= ( 1 - pitch_K );
    // DroneConfigs.yaw.error   *= ( 1 - yaw_K );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Seek devices in I2C bus
 * @param none
 * @retval bool
 */
static bool i2c_scan( void ) {
    bool found = false;
    for ( uint8_t address = 1; address < 127; address++ )
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK( i2c_master_start( cmd ) );
        ESP_ERROR_CHECK( i2c_master_write_byte( cmd, ( address << 1 ) | I2C_MASTER_WRITE, true ) );
        ESP_ERROR_CHECK( i2c_master_stop( cmd ) );
        
        esp_err_t ret = i2c_master_cmd_begin( I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
        i2c_cmd_link_delete( cmd );
        
        if ( ret == ESP_OK )
        {
            if( address == BMI160_ADDR )
            {
                ESP_LOGI( DRONE_TAG, "Bmi160 found at ( 0x%02x )", address );
                found = true;
            }
            else
            {
                ESP_LOGI( DRONE_TAG, "Device found at address: ( 0x%02x )", address );
                found = true;
            }
        }
        else if ( ret == ESP_ERR_TIMEOUT )
            ESP_LOGW( DRONE_TAG, "I2C bus is busy" );
    }

    return found;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize an object of Drone Class
 * @param obj: Address of Drone object
 * @retval esp_err_t
 */
static esp_err_t drone_init( drone_t * obj ) {
    ESP_LOGI( DRONE_TAG, "Initializing Drone object..." );
    
    /* Drone object is initialized */
    obj->attributes.init_ok = true;

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();

    /* Initialize Bmi160 object */
    ESP_ERROR_CHECK( obj->attributes.components.bmi->init(
        obj->attributes.components.bmi,
        DroneConfigs.imu_cfg.acc_mode,
        DroneConfigs.imu_cfg.acc_freq,
        DroneConfigs.imu_cfg.acc_range,
        DroneConfigs.imu_cfg.gyro_mode,
        DroneConfigs.imu_cfg.gyro_freq,
        DroneConfigs.imu_cfg.gyro_range,
        DroneConfigs.imu_cfg.gyro_offset.x,
        DroneConfigs.imu_cfg.gyro_offset.y,
        DroneConfigs.imu_cfg.gyro_offset.z
        )
    );

    /* Fast offset compensation for bmi sensor */
    obj->attributes.components.bmi->foc( obj->attributes.components.bmi );

    /* Initialize all Pwm objects */
    for( int i = 0; i < ( ( sizeof( obj->attributes.components.pwm ) ) / ( sizeof( obj->attributes.components.pwm[ 0 ] ) ) ); i++ ) {
        obj->attributes.components.pwm[ i ]->init( obj->attributes.components.pwm[ i ], DroneConfigs.pwm_cfg[ i ] );
    }

    /* Initialize all Pid objects */
    for(int i = 0; i < ( ( sizeof( obj->attributes.components.controllers ) ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ) ); i++) {
        obj->attributes.components.controllers[ i ]->init( obj->attributes.components.controllers[ i ], DroneConfigs.ControllersConfigs[ i ] );
    }
    
    /* Blink MCU internal LED to indicate Drone object was successfully initialized */
    gpio_set_level( GPIO_NUM_2, false );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    gpio_set_level( GPIO_NUM_2, true );

    ESP_LOGI( DRONE_TAG, "Drone object initialized" );

    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Update Drone object states
 * @param obj: Address of Drone object
 * @param ts: Sampling time in milliseconds
 * @retval none
 */
static void UpdateStates( drone_t * obj, float ts ) {
    if( !obj->attributes.init_ok ) {

        /* If Drone object isn't initialized */
        ESP_LOGE( DRONE_TAG, "Drone object must be initialized before calling it's methods!. See %s in line %d", __func__, __LINE__ );
    }
    else {

        /* Update state's velocity */
        obj->attributes.states.roll_dot  = FirstOrderIIR( obj->attributes.components.bmi->Gyro.x, obj->attributes.states.roll_dot,  DroneConfigs.IIR_coeff_roll_dot );
        obj->attributes.states.pitch_dot = FirstOrderIIR( obj->attributes.components.bmi->Gyro.y, obj->attributes.states.pitch_dot, DroneConfigs.IIR_coeff_pitch_dot );
        obj->attributes.states.yaw_dot   = FirstOrderIIR( obj->attributes.components.bmi->Gyro.z, obj->attributes.states.yaw_dot,   DroneConfigs.IIR_coeff_yaw_dot );
        
        /* Update state's position */
        Kalman( obj, ts );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


drone_t * Drone( void ) {
    ESP_LOGI( DRONE_TAG, "Making an instance of Drone Class..." );

    /* Assign memmory to Drone object */
    drone_t * drone = malloc( sizeof( drone_t ) );
    if( drone == NULL )
        return NULL;

    /* States default values */
    drone->attributes.states.z         = 0;
    drone->attributes.states.roll      = 0;
    drone->attributes.states.pitch     = 0;
    drone->attributes.states.yaw       = 0;
    drone->attributes.states.roll_dot  = 0;
    drone->attributes.states.pitch_dot = 0;
    drone->attributes.states.yaw_dot   = 0;
    
    /* Drone object isn't initialized yet */
    drone->attributes.init_ok = false;

    /* Assign memmory to Transmitter object buttons */
    drone->attributes.global_variables.tx_buttons = malloc( sizeof( tx_buttons_t ) );

    /* Transmitter buttons default values */
    drone->attributes.global_variables.tx_buttons->cross    = false;
    drone->attributes.global_variables.tx_buttons->square   = false;
    drone->attributes.global_variables.tx_buttons->triangle = false;
    drone->attributes.global_variables.tx_buttons->circle   = false;
    drone->attributes.global_variables.tx_buttons->up       = false;
    drone->attributes.global_variables.tx_buttons->down     = false;
    drone->attributes.global_variables.tx_buttons->left     = false;
    drone->attributes.global_variables.tx_buttons->right    = false;
    drone->attributes.global_variables.tx_buttons->r1       = false;
    drone->attributes.global_variables.tx_buttons->r2       = false;
    drone->attributes.global_variables.tx_buttons->l1       = false;
    drone->attributes.global_variables.tx_buttons->l2       = false;

    /* Assign memory to Bluetooth data */
    drone->attributes.global_variables.bt_data = malloc( sizeof( BluetoothData_t ) );

    /* Bluetooth data default values */
    drone->attributes.global_variables.bt_data->data = malloc( 256 * sizeof( char ) );
    drone->attributes.global_variables.bt_data->len = 0;
    drone->attributes.global_variables.bt_data->state = false;

    /* Pointer to Drone functions ( methods ) */
    drone->methods.update_states    = UpdateStates;
    drone->methods.init             = drone_init;
    drone->methods.i2c_scan         = i2c_scan;

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();

    /* Make an instance of Bmi160 Class */
    drone->attributes.components.bmi = Bmi160(
        DroneConfigs.imu_cfg.imu_i2c_cfg.address,
        DroneConfigs.imu_cfg.imu_i2c_cfg.sda,
        DroneConfigs.imu_cfg.imu_i2c_cfg.scl
    );

    /* Check if all devices are connected to i2c bus */
    if( !drone->methods.i2c_scan() ) {
        bool found = false;
        ESP_LOGI( DRONE_TAG, "Scanning i2c bus..." );
        while( !found )
        {
            if( drone->methods.i2c_scan() ) {
                found = true;
                break;
            }

            vTaskDelay( pdMS_TO_TICKS( 10 ) );
        }
    }

    /* Make an instance of Mma Class */
    drone->attributes.components.mma = Mma();

    /* Make an instance of Pid Class for all controllers */
    for(int i = 0; i < ( ( sizeof( drone->attributes.components.controllers ) ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ) ); i++) {
        drone->attributes.components.controllers[ i ] = Pid();
    }

    /* Make an instance of Pwm Class for all pwm signals */
    for(int i = 0; i < ( sizeof( drone->attributes.components.pwm ) / sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++) {
        drone->attributes.components.pwm[ i ] = Pwm( i );
    }
    
    /* Make an instance of Transmitter Class */
    drone->attributes.components.Tx = Transmitter( &( drone->attributes.global_variables ) );
    ESP_LOGI( DRONE_TAG, "Instance succesfully made" );

    /* Initialize Transmitter object */
    drone->attributes.components.Tx->init( drone->attributes.components.Tx, DroneConfigs.esp_mac_addr );

    /* Assign Transmitter buttons global variable memmory address to 'GlobalTxButtons' variable */
    GlobalTxButtons = drone->attributes.global_variables.tx_buttons;

    /* Assign Bluetooth data global variable memmory address to 'GlobalBluetoothData' variable */
    GlobalBluetoothData = drone->attributes.global_variables.bt_data;

    /* Blink MCU internal LED to indicate Transmitter object is ready to receive commands */
    gpio_set_level( GPIO_NUM_2, false );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    gpio_set_level( GPIO_NUM_2, true );

    /* Return instance of Drone Class */
    return drone;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


drone_cfg_t GetDroneConfigs( void ) {
    return DroneConfigs;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float __sin( float A, float w, float t ) {

    return A * sin( w * t );
}
