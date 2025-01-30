#include <drone.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <math.h>
#include <esp_log.h>
#include <string.h>
#include <esp_spiffs.h>
#include <freertos/FreeRTOS.h>

const char * DRONE_TAG = "DRONE";

// #define IGNORE_BMI

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


/** @details Private structs */

/**
 * @brief Details of a csv row
 * @param var_name: Variable name
 * @param var_type: Variable type
 * @param var_value: Vaiable value
 */
typedef struct csv_row {

    /* Variable name */
    char * var_name;

    /* Variable type */
    char * var_type;

    /* Vaiable value */
    float var_value;

} csv_row_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Drone Class generic configs
 */
static drone_cfg_t DroneConfigs = {
    .roll = {
        .P = 1.0f,
        .Q = 0.1f,
        .R = 0.5f,
    },
    .pitch = {
        .P = 0.0f,
        .Q = 0.0f,
        .R = 0.0f,
    },
    .yaw = {
        .P = 0.0f,
        .Q = 0.0f,
        .R = 0.0f,
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
            .tag       = z,
            .fc        = 0,
            .ts        = 10,
            .sat       = no_saturation,
            .gains     = { .kp = 0, .ki = 0, .kd = 0 },
            .intMinErr = 0.0f
        },
        {
            .tag = roll,
            .fc  = 1.0f,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f },
            .intMinErr = 0.0f
        },
        {
            .tag = pitch,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 },
            .intMinErr = 0.0f
        },
        {
            .tag = yaw,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 },
            .intMinErr = 0.0f
        },
        {
            .tag = roll_dot,
            .fc  = 1.0f,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f },
            .intMinErr = 0.0f
        },
        {
            .tag = pitch_dot,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 },
            .intMinErr = 0.0f
        },
        {
            .tag = yaw_dot,
            .fc  = 0,
            .ts  = 10,
            .sat = no_saturation,
            .gains = { .kp = 0, .ki = 0, .kd = 0 },
            .intMinErr = 0.0f
        },
    },
    .mma_out_limits = {
        .upper = 0.91f,
        .lower = 1.1f
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

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();


    /* -------------------------------------------------------------------------------- */


    /* Prediction step */

    /* Use Gyroscope as mathematical model of IMU sensor */
    float estimated_roll = obj->attributes.states.roll + ( ts / 1000.0f ) * obj->attributes.components.bmi.Gyro.x;

    /* Predict uncertainty of estimation */
    float estimated_P = DroneConfigs.roll.P + DroneConfigs.roll.Q;


    /* -------------------------------------------------------------------------------- */


    /* Update step */

    /* Update Kalman gain */
    float roll_K = estimated_P / ( estimated_P + DroneConfigs.roll.R );

    /* Use Accelerometer as sensor readings of IMU */
    float roll_sensor = atan2( obj->attributes.components.bmi.Acc.y, obj->attributes.components.bmi.Acc.z ) * ( 180.0f / M_PI );

    /* Update states */
    obj->attributes.states.roll = ( estimated_roll * ( 1 - roll_K ) ) + ( roll_K * roll_sensor );

    /* Update P uncertainty */
    DroneConfigs.roll.P = ( 1 - roll_K ) * estimated_P;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Seek devices in I2C bus
 * @param none
 * @retval bool
 */
static bool i2c_scan( void ) {

    bool found = false;
    for( uint8_t address = 1; address < 127; address++ ) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK( i2c_master_start( cmd ) );
        ESP_ERROR_CHECK( i2c_master_write_byte( cmd, ( address << 1 ) | I2C_MASTER_WRITE, true ) );
        ESP_ERROR_CHECK( i2c_master_stop( cmd ) );
        
        esp_err_t ret = i2c_master_cmd_begin( I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS );
        i2c_cmd_link_delete( cmd );

        if ( ret == ESP_OK ) {

            if( address == GetDroneConfigs().imu_cfg.imu_i2c_cfg.address ) {

                ESP_LOGI( DRONE_TAG, "Bmi160 found at ( 0x%02x )", address );
                found = true;
            }

            else {

                ESP_LOGI( DRONE_TAG, "Device found at address: ( 0x%02x )", address );
            }
        }
        
        else if ( ret == ESP_ERR_TIMEOUT ) {

            ESP_LOGW( DRONE_TAG, "I2C bus is busy" );
        }
    }

    return found;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Read drone_configs csv file and return csv_row_t object with each row data
 * @param filename: Path to csv file
 * @param n_rows: Rows count variable pre-initialized in 0
 * @retval Updated csv_row_t object with csv rows
 */
static csv_row_t * read_csv( const char * filename, int * n_rows ) {

    /* Open file in read only mode */
    FILE * fp = fopen( filename, "r" );

    /* Initialize rows to NULL */
    csv_row_t * rows = NULL;

    /* Initialize rows count to 0 */
    *n_rows = 0;

    /* Check if file was successfully opened */
    if( !fp ) {

        ESP_LOGE( DRONE_TAG, "Failed to open '%s' file... See function %s in line %d", filename, __func__, __LINE__ );
    }

    /* File was successfully opened */
    else {

        /* Buffer to store a csv row */
        char buffer[ 1024 ];

        const char * delimeter = ", ";

        /* Read csv rows until end of file */
        while( fgets( buffer, 1024, fp ) ) {


            /* ASCII integer value of # is 35 => If first character is #, then it's the header row */
            if(  ( int ) buffer[ 0 ] == 35 ) {
                
                continue;
            }

            /* Increment rows count */
            ( *n_rows )++;

            /* Re-alocate memory for a new row */
            rows = realloc( rows, ( *n_rows ) * sizeof( csv_row_t ) );

            /* Check memory re-alocation was successfull */
            if( !rows ) {

                ESP_LOGE( DRONE_TAG, "Failed to re-alocate memory for another row... See function %s in line %d", __func__, __LINE__ );
                fclose( fp );
                return NULL;
            }

            /* Get each column value */
            char * token = strtok( buffer, delimeter );

            /* Get all column values */
            for( int i = 0; token != NULL; i++ ) {

                switch( i ) {

                    case 0:
                        rows[ ( *n_rows ) - 1 ].var_name = strdup( token );
                        break;
                    
                    case 1:
                        rows[ ( *n_rows ) - 1 ].var_type = strdup( token );
                        break;

                    case 2:
                        rows[ ( *n_rows ) - 1 ].var_value = atof( token );
                        break;

                    default:
                        break;
                }

                /* Get next column value */
                token = strtok( NULL, delimeter );
            }
        }
        
        /* Close file */
        fclose( fp );
    }

    return rows;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Retrieve first occurence of 'name' in 'Name' column of drone_configs csv file
 * @param csv_rows: csv rows object retrieved by 'read_csv' function
 * @param n_rows: Total rows of drone_configs csv
 * @param name: Name to look for
 * @retval csv row of given name - NULL
 */
static csv_row_t get_csv_row( csv_row_t * csv_rows, int n_rows, const char * name ) {

    /* Set default values for a row */
    csv_row_t csv_row = {

        .var_name  = "",
        .var_type  = "",
        .var_value = 0.0f
    };

    /* Look for given name in all rows */
    for( int i = 0; i < n_rows; i++ ) {

        /* Check if actual row name matches parameter name */
        if( !strcmp( csv_rows[ i ].var_name, name ) ) {

            csv_row = csv_rows[ i ];
        }
    }

    return csv_row;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize an object of Drone Class
 * @param obj: Address of Drone object
 * @retval ESP_OK if success - ESP_FAIL
 */
static esp_err_t drone_init( drone_t * obj ) {

    ESP_LOGI( DRONE_TAG, "Initializing Drone object..." );

    /* Drone object is initialized */
    obj->attributes.init_ok = true;

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();

    #ifndef IGNORE_BMI
    /* Initialize Bmi160 object */
    ESP_ERROR_CHECK( obj->attributes.components.bmi.init(
        &( obj->attributes.components.bmi ),
        DroneConfigs.imu_cfg.acc_mode,
        DroneConfigs.imu_cfg.acc_freq,
        DroneConfigs.imu_cfg.acc_range,
        DroneConfigs.imu_cfg.gyro_mode,
        DroneConfigs.imu_cfg.gyro_freq,
        DroneConfigs.imu_cfg.gyro_range,
        0.0f,
        0.0f,
        0.0f
        )
    );
    

    obj->attributes.components.bmi.Gyro.offset.x = DroneConfigs.imu_cfg.gyro_offset.x;
    obj->attributes.components.bmi.Gyro.offset.y = DroneConfigs.imu_cfg.gyro_offset.y;
    obj->attributes.components.bmi.Gyro.offset.z = DroneConfigs.imu_cfg.gyro_offset.z;

    /* Fast offset compensation for bmi sensor */
    obj->attributes.components.bmi.foc( &( obj->attributes.components.bmi ) );
    #endif
    // obj->attributes.components.bmi->Gyro.calibrate( &( obj->attributes.components.bmi->Gyro ), 2000 );

    /* Initialize all Pwm objects */
    for( int i = 0; i < ( ( sizeof( obj->attributes.components.pwm ) ) / ( sizeof( obj->attributes.components.pwm[ 0 ] ) ) ); i++ ) {

        obj->attributes.components.pwm[ i ]->init( obj->attributes.components.pwm[ i ], DroneConfigs.pwm_cfg[ i ] );
    }

    /* Initialize all Pid objects */
    for( int i = 0; i < ( ( sizeof( obj->attributes.components.controllers ) ) / ( sizeof( obj->attributes.components.controllers[ 0 ] ) ) ); i++ ) {

        obj->attributes.components.controllers[ i ]->init( obj->attributes.components.controllers[ i ], DroneConfigs.ControllersConfigs[ i ] );
    }

    /* Initialize Mma object */
    obj->attributes.components.mma->init(
        obj->attributes.components.mma,
        DroneConfigs.mma_out_limits.upper,
        DroneConfigs.mma_out_limits.lower
    );

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

        #ifndef IGNORE_BMI
        obj->attributes.states.roll_dot  = FirstOrderIIR( obj->attributes.components.bmi.Gyro.x, obj->attributes.states.roll_dot,  DroneConfigs.IIR_coeff_roll_dot );
        obj->attributes.states.pitch_dot = FirstOrderIIR( obj->attributes.components.bmi.Gyro.y, obj->attributes.states.pitch_dot, DroneConfigs.IIR_coeff_pitch_dot );
        obj->attributes.states.yaw_dot   = FirstOrderIIR( obj->attributes.components.bmi.Gyro.z, obj->attributes.states.yaw_dot,   DroneConfigs.IIR_coeff_yaw_dot );
        
        /* Update state's position */
        Kalman( obj, ts );
        #endif
        // float roll_acc = atan2( obj->attributes.components.bmi.Acc.y, obj->attributes.components.bmi.Acc.z ) * ( 180.0f / M_PI );
        // float roll_gyro = obj->attributes.states.roll + ( obj->attributes.components.bmi.Gyro.x * ( ts / 1000.0f ) );
        // obj->attributes.states.roll = ( 0.98f * roll_acc ) + ( 0.02f * roll_gyro );
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


drone_t * Drone( void ) {

    ESP_LOGI( DRONE_TAG, "Making an instance of Drone Class..." );

    /* Assign memory to Drone object */
    drone_t * drone = ( drone_t * ) malloc( sizeof( drone_t ) );

    /* Check if memory assign was succesfull */
    if( drone == NULL ){

        return NULL;
    }

    /* States default values */
    drone->attributes.states.z         = 0.0f;
    drone->attributes.states.roll      = 0.0f;
    drone->attributes.states.pitch     = 0.0f;
    drone->attributes.states.yaw       = 0.0f;
    drone->attributes.states.roll_dot  = 0.0f;
    drone->attributes.states.pitch_dot = 0.0f;
    drone->attributes.states.yaw_dot   = 0.0f;
    
    /* sp default values */
    drone->attributes.sp.z         = 0.0f;
    drone->attributes.sp.roll      = 0.0f;
    drone->attributes.sp.pitch     = 0.0f;
    drone->attributes.sp.yaw       = 0.0f;
    drone->attributes.sp.roll_dot  = 0.0f;
    drone->attributes.sp.pitch_dot = 0.0f;
    drone->attributes.sp.yaw_dot   = 0.0f;

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
    drone->attributes.global_variables.tx_buttons->select   = false;
    drone->attributes.global_variables.tx_buttons->start    = false;
    drone->attributes.global_variables.tx_buttons->ps       = false;

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

    /* Initialize spiffs */
    esp_vfs_spiffs_conf_t config = {

        .base_path              = "/spiffs",    /* See partition table => Spiffs row => Name column */
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true
    };

    /* Mount spiffs configs */
    esp_err_t ret = esp_vfs_spiffs_register( &config );

    /* Check if mount was succesfull */
    if( ret != ESP_OK ) {

        if( ret == ESP_ERR_NOT_FOUND ) {

            ESP_LOGE( DRONE_TAG, "Failed to find spiffs partition" );
            return NULL;
        }

        else if( ret == ESP_FAIL ) {

            ESP_LOGE( DRONE_TAG, "Failed to mount spiffs partition" );
            return NULL;
        }

        else {

            ESP_LOGE( DRONE_TAG, "Failed to initialize spiffs ( %s )", esp_err_to_name( ret ) );
            return NULL;
        }
    }

    ESP_LOGI( DRONE_TAG, "Spiffs mounted successfully" );

    /* Number of csv rows */
    int n_rows = 0;

    /* Read csv, stored in flash memory of MCU, rows */
    csv_row_t * csv_rows = read_csv( "/spiffs/drone_configs.csv", &n_rows );    /* '/base_path/filename.extension' */

    /**
     * ¡IMPORTANT!
     * 
     * Any changes for Drone Class general configs must be done before calling 'GetDroneConfigs' function
     */
    DroneConfigs.imu_cfg.gyro_offset.x = get_csv_row( csv_rows, n_rows, "x" ).var_value;
    DroneConfigs.imu_cfg.gyro_offset.y = get_csv_row( csv_rows, n_rows, "y" ).var_value;
    DroneConfigs.imu_cfg.gyro_offset.z = get_csv_row( csv_rows, n_rows, "z" ).var_value;
    
    printf( "Upper limit ( before ): %.2f\r\n", DroneConfigs.mma_out_limits.upper );
    DroneConfigs.mma_out_limits.upper = get_csv_row( csv_rows, n_rows, "upper_limit" ).var_value;
    printf( "Upper limit ( after ): %.2f\r\n", DroneConfigs.mma_out_limits.upper );
    DroneConfigs.mma_out_limits.lower = get_csv_row( csv_rows, n_rows, "lower_limit" ).var_value;

    /* Get Drone Class generic configs */
    drone_cfg_t DroneConfigs = GetDroneConfigs();

    /* Make an instance of Bmi160 Class */
    #ifndef IGNORE_BMI
    ESP_ERROR_CHECK(
        Bmi160(
            &( drone->attributes.components.bmi ),
            DroneConfigs.imu_cfg.imu_i2c_cfg.address,
            DroneConfigs.imu_cfg.imu_i2c_cfg.scl,
            DroneConfigs.imu_cfg.imu_i2c_cfg.sda
        )
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
    #endif

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

    #if PLAYSTATION_TX & WEBSV_TX
        ESP_LOGE( DRONE_TAG, "Multiple transmitters can't be used simmultaneously. See transmitter_structs.h header file" );
        esp_restart();

    /* Check if playstation joystick is used as transmitter */
    #elif PLAYSTATION_TX
        /* Initialize Transmitter object */
        drone->attributes.components.Tx->init( drone->attributes.components.Tx, DroneConfigs.esp_mac_addr );
        
    /* Check if HTTP server is used as transmitter */
    #elif WEBSV_TX
        /* Initialize Transmitter object */
        drone->attributes.components.Tx->init( drone->attributes.components.Tx );

    #else
        /* No transmitter selected */
        ESP_LOGE( DRONE_TAG, "No transmitter selected. See transmitter_structs.h header file" );
        esp_restart();

    #endif

    /* Assign Transmitter buttons global variable memmory address to 'GlobalTxButtons' variable */
    GlobalTxButtons = drone->attributes.global_variables.tx_buttons;

    /* Assign Bluetooth data global variable memmory address to 'GlobalBluetoothData' variable */
    GlobalBluetoothData = drone->attributes.global_variables.bt_data;

    /* Free memory used for csv object */
    for( int i = 0; i < n_rows; i++ ) {

        free( csv_rows[ i ].var_name );
        free( csv_rows[ i ].var_type );
    }
    free( csv_rows );

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
