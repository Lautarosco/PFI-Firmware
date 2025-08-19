#include <drone.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <math.h>
#include <esp_log.h>
#include <string.h>
#include <esp_spiffs.h>
#include <freertos/FreeRTOS.h>
#include <drone_flash.h>
#include <esp_err.h>

const char * DRONE_TAG = "DRONE";

// #define IGNORE_BMI

/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public variables */

tx_buttons_t *GlobalTxButtons;          /** @brief Pointer to buttons global variable of a Drone object ( used in transmitter component | transmitter_structs.c source file ) */
SerialData_t *GlobalSerialData;         /** @brief Pointer to Bluetooth data global variable of a Drone object ( used in ps3 component | ps3_spp.c source file ) */
pid_gain_t *GlobalRollGains;            /** @brief Pointer to roll controller gains */
pid_gain_t *GlobalRoll_dGains;          /** @brief Pointer to roll controller gains */
pid_gain_t *GlobalPitchGains;           /** @brief Pointer to pitch controller gains */
pid_gain_t *GlobalPitch_dGains;         /** @brief Pointer to pitch controller gains */
pid_gain_t *GlobalYawGains;             /** @brief Pointer to yaw controller gains */
pid_gain_t *GlobalYaw_dGains;           /** @brief Pointer to yaw controller gains */
pid_gain_t *GlobalZGains;               /** @brief Pointer to Z controller gains */
pid_gain_t *GlobalZ_dGains;             /** @brief Pointer to Z_d controller gains */


/* ------------------------------------------------------------------------------------------------------------------------------------------ */
static drone_parameter_t drone_params_array[MAX_FLASH_PARAMS];

esp_err_t get_drone_params(drone_t* drone) {
    int idx = 0;  // Parameter index counter
    
    // Gyro offsets
    // drone_params_array[idx++] = (drone_parameter_t){"gyro_offset_x", &drone->attributes.components.bmi.Gyro.offset.x, sizeof(float), true};
    // drone_params_array[idx++] = (drone_parameter_t){"gyro_offset_y", &drone->attributes.components.bmi.Gyro.offset.y, sizeof(float), true};
    // drone_params_array[idx++] = (drone_parameter_t){"gyro_offset_z", &drone->attributes.components.bmi.Gyro.offset.z, sizeof(float), true};
    
    // IIR filter coefficients  
    drone_params_array[idx++] = (drone_parameter_t){"gyro_x_ema", &drone->attributes.config.IIR_coeff_roll_dot, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"gyro_y_ema", &drone->attributes.config.IIR_coeff_pitch_dot, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"gyro_z_ema", &drone->attributes.config.IIR_coeff_yaw_dot, sizeof(float), true};
    
    // Roll PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"roll/P", &drone->attributes.components.controllers[ROLL].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll/I", &drone->attributes.components.controllers[ROLL].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll/D", &drone->attributes.components.controllers[ROLL].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll/KB", &drone->attributes.components.controllers[ROLL].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll/D_IIR", &drone->attributes.components.controllers[ROLL].derivative_lpf.alpha, sizeof(float), true};
    
    // Roll_D PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"roll_d/P", &drone->attributes.components.controllers[ROLL_D].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll_d/I", &drone->attributes.components.controllers[ROLL_D].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll_d/D", &drone->attributes.components.controllers[ROLL_D].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll_d/KB", &drone->attributes.components.controllers[ROLL_D].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"roll_d/D_IIR", &drone->attributes.components.controllers[ROLL_D].derivative_lpf.alpha, sizeof(float), true};
    
    // Pitch PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"pitch/P", &drone->attributes.components.controllers[PITCH].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch/I", &drone->attributes.components.controllers[PITCH].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch/D", &drone->attributes.components.controllers[PITCH].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch/KB", &drone->attributes.components.controllers[PITCH].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch/D_IIR", &drone->attributes.components.controllers[PITCH].derivative_lpf.alpha, sizeof(float), true};
    
    // Pitch_D PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"pitch_d/P", &drone->attributes.components.controllers[PITCH_D].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch_d/I", &drone->attributes.components.controllers[PITCH_D].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch_d/D", &drone->attributes.components.controllers[PITCH_D].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch_d/KB", &drone->attributes.components.controllers[PITCH_D].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"pitch_d/D_IIR", &drone->attributes.components.controllers[PITCH_D].derivative_lpf.alpha, sizeof(float), true};
    
    // Yaw PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"yaw/P", &drone->attributes.components.controllers[YAW].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw/I", &drone->attributes.components.controllers[YAW].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw/D", &drone->attributes.components.controllers[YAW].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw/KB", &drone->attributes.components.controllers[YAW].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw/D_IIR", &drone->attributes.components.controllers[YAW].derivative_lpf.alpha, sizeof(float), true};
    
    // Yaw_D PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"yaw_d/P", &drone->attributes.components.controllers[YAW_D].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw_d/I", &drone->attributes.components.controllers[YAW_D].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw_d/D", &drone->attributes.components.controllers[YAW_D].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw_d/KB", &drone->attributes.components.controllers[YAW_D].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"yaw_d/D_IIR", &drone->attributes.components.controllers[YAW_D].derivative_lpf.alpha, sizeof(float), true};
    
    // Z PID parameters  
    drone_params_array[idx++] = (drone_parameter_t){"z/P", &drone->attributes.components.controllers[Z].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z/I", &drone->attributes.components.controllers[Z].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z/D", &drone->attributes.components.controllers[Z].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z/KB", &drone->attributes.components.controllers[Z].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z/D_IIR", &drone->attributes.components.controllers[Z].derivative_lpf.alpha, sizeof(float), true};
    
    // Z_D PID parameters
    drone_params_array[idx++] = (drone_parameter_t){"z_d/P", &drone->attributes.components.controllers[Z_D].gain.kp, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z_d/I", &drone->attributes.components.controllers[Z_D].gain.ki, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z_d/D", &drone->attributes.components.controllers[Z_D].gain.kd, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z_d/KB", &drone->attributes.components.controllers[Z_D].gain.kb, sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"z_d/D_IIR", &drone->attributes.components.controllers[Z_D].derivative_lpf.alpha, sizeof(float), true};

    // Misc. Floats
    drone_params_array[idx++] = (drone_parameter_t){"misc/0", &drone->attributes.global_variables.misc_floats[0], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/1", &drone->attributes.global_variables.misc_floats[1], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/2", &drone->attributes.global_variables.misc_floats[2], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/3", &drone->attributes.global_variables.misc_floats[3], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/4", &drone->attributes.global_variables.misc_floats[4], sizeof(float), false};
    drone_params_array[idx++] = (drone_parameter_t){"misc/5", &drone->attributes.global_variables.misc_floats[5], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/6", &drone->attributes.global_variables.misc_floats[6], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/7", &drone->attributes.global_variables.misc_floats[7], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/8", &drone->attributes.global_variables.misc_floats[8], sizeof(float), true};
    drone_params_array[idx++] = (drone_parameter_t){"misc/9", &drone->attributes.global_variables.misc_floats[9], sizeof(float), true};

    // Mark remaining slots as empty (null name indicates end)
    for (int i = idx; i < MAX_FLASH_PARAMS; i++) {
        drone_params_array[i] = (drone_parameter_t){NULL, NULL, 0, false};
    }

    // Copy the array to the drone's flash_params_arr
    drone->attributes.flash_params_arr = drone_params_array;

    return ESP_OK;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */

static drone_cfg_t GetDroneConfigs( void ) {

    return DroneConfigs;
}

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
 * @param ts_s: Sampling time in seconds
 * @param alpha: Filter coefficient (0 - 1)
 * @return float
 */
float FirstOrderIIR( float in, float out, float ts_s, float alpha ) {
    if (alpha > 1 || alpha < 0) {
        ESP_LOGE("FirstOrderIIR", "INCORRECT ALPHA SELECTED %.2f", alpha);
    }
    return ( ( 1 - alpha ) * in ) + ( alpha * out );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */

/**
 * @brief Seek devices in I2C bus
 * @param none
 * @retval bool
 */
static bool i2c_scan( void ) {
    typedef struct {
        char name[16];
        uint8_t address;
        bool found;
    } i2c_device_info_t;

    i2c_device_info_t i2c_devices[] = {
        {"BMI160", BMI160_ADDR, false},
        // {"BMP390", 0x76, false},
    };
    const int num_devices = sizeof(i2c_devices) / sizeof(i2c_devices[0]);

    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_APB,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };


    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) != ESP_OK) {
        ESP_LOGE(DRONE_TAG, "Failed to create I2C master bus");
        return false;
    }

    bool found_any = false;
    for (int i = 0; i < num_devices; ++i) {
        i2c_device_config_t dev_cfg = {
            .device_address = i2c_devices[i].address,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .scl_speed_hz = 100000,
            .flags.disable_ack_check = false,
            .scl_wait_us = 0
        };
        i2c_master_dev_handle_t dev_handle = NULL;
        esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev_handle);
        if (err == ESP_OK) {
            // Try to read a single byte to check if device responds
            uint8_t dummy = 0;
            esp_err_t probe_err = i2c_master_receive(dev_handle, &dummy, 1, 100 / portTICK_PERIOD_MS);
            if (probe_err == ESP_OK) {
                ESP_LOGI(DRONE_TAG, "I2C device '%s' found at address 0x%02X", i2c_devices[i].name, i2c_devices[i].address);
                i2c_devices[i].found = true;
                found_any = true;
            } else {
                ESP_LOGW(DRONE_TAG, "No device at address 0x%02X (%s)", i2c_devices[i].address, i2c_devices[i].name);
            }
            i2c_master_bus_rm_device(dev_handle);
        } else {
            ESP_LOGW(DRONE_TAG, "No device at address 0x%02X (%s)", i2c_devices[i].address, i2c_devices[i].name);
        }
    }
    i2c_del_master_bus(i2c_bus);


    return found_any;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Save Drone parameters to flash memory
 * @param drone: Direction of Drone object
 * @retval none
 */
static void save_to_nvs( drone_t * drone ) {

    /* Loop through all flash parameters */
    for( int i = 0; i < MAX_FLASH_PARAMS; i++ ) {

        /* If parameter direction if NULL, then all parameters have been saved */
        if( drone->attributes.flash_params_arr[ i ].ptr == NULL ) {

            break;
        }
        
        /* Continue updating NVS with Drone parameters */
        else if (drone->attributes.flash_params_arr[i].save_to_flash) {
            /* Store "i" parameter to NVS, according to drone_flash_params_t enum */
            esp_err_t err = __write_to_flash( NVS_NAMESPACE, drone->attributes.flash_params_arr[i].name, drone->attributes.flash_params_arr[ i ].ptr, drone->attributes.flash_params_arr[ i ].size );
            if (err == ESP_OK) {
            ESP_LOGI( DRONE_TAG, "Saved %s: %.2f", drone->attributes.flash_params_arr[i].name, *(float*) (drone->attributes.flash_params_arr[i].ptr) );
            } else {
            ESP_LOGE( DRONE_TAG, "Failed to save %s (err: %s)", drone->attributes.flash_params_arr[i].name, esp_err_to_name(err) );
            }

            float check_value = 0.0f;
            if (__read_from_flash(NVS_NAMESPACE, drone->attributes.flash_params_arr[i].name, &check_value, sizeof(check_value)) == ESP_OK) {
                ESP_LOGI(DRONE_TAG, "Checked %s from flash: %.2f", drone->attributes.flash_params_arr[i].name, check_value);
            } else {
                ESP_LOGW(DRONE_TAG, "Could not read back %s from flash", drone->attributes.flash_params_arr[i].name);
            }

        } else {
            ESP_LOGI(DRONE_TAG, "Skipping variable %s on save.\n", drone->attributes.flash_params_arr[i].name);
        }
    }
}

/**
 * @brief Read Drone parameters stored in flash memory and update the running parameters
 * @param drone: Direction of Drone object
 * @retval none
 */
static void read_from_nvs( drone_t * drone ) {

    float read_var = 0.0f;

    for( int i = 0; i < MAX_FLASH_PARAMS; i++ ) {

        /* If parameter direction if NULL, then all parameters have been read */
        if( drone->attributes.flash_params_arr[ i ].ptr == NULL ) {

            break;
        }

        if (!drone->attributes.flash_params_arr[i].save_to_flash) {
            ESP_LOGI(DRONE_TAG, "Skipping variable %s on read.\n", drone->attributes.flash_params_arr[i].name);
            continue;
        }

        /* Continue reading NVS */
        else {
        esp_err_t err = __read_from_flash(NVS_NAMESPACE, drone->attributes.flash_params_arr[i].name, &read_var, sizeof(read_var));
        if (err != ESP_OK) {
            ESP_LOGE(DRONE_TAG, "Error leyendo flash para %s: %s", drone->attributes.flash_params_arr[i].name, esp_err_to_name(err));
        } else {
            *(float*)(drone->attributes.flash_params_arr[i].ptr) = read_var;
            ESP_LOGI(DRONE_TAG, "Updated %s: %.2f", drone->attributes.flash_params_arr[i].name, read_var);
        }
        }
    }
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize an object of Drone Class
 * @param drone: Address of Drone object
 * @retval ESP_OK if success - ESP_FAIL
 */
static esp_err_t drone_init( drone_t * drone ) {

    ESP_LOGI( DRONE_TAG, "Initializing Drone object..." );

    #ifndef IGNORE_BMI

    i2c_master_bus_handle_t i2c_master_handler = NULL;      /* I2C master bus handler */
    i2c_master_bus_config_t i2c_master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = I2C_SCL_PIN,
        .sda_io_num                   = I2C_SDA_PIN,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C BMP390 bus handler */
    i2c_master_dev_handle_t i2c_bmi_handler = NULL;         /* I2C BMI160 bus handler */
    i2c_master_dev_handle_t i2c_imu_handler = NULL;         /* I2C LSM6DSO bus handler */

    #define I2C_BUS_FREQUENCY 400000

    /* 3.b Set I2C BMP390 and BMI160 bus configs */
    i2c_device_config_t i2c_bmp_cfg = {
        .device_address          = BMP390_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = I2C_BUS_FREQUENCY,
        .flags.disable_ack_check = false,
        .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
    };
    
    i2c_device_config_t i2c_imu_cfg = {
        .device_address          = LSM6DSO_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = I2C_BUS_FREQUENCY,
        .flags.disable_ack_check = false,
        .scl_wait_us             = 0
    };

    i2c_device_config_t i2c_bmi_cfg = {
        .device_address          = BMI160_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = I2C_BUS_FREQUENCY,
        .flags.disable_ack_check = false,
        .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
    };

    lsm6dso_params_t imu_params = {
        .i2c_lsm_handler = &i2c_imu_handler,
        .acc = {
            .fs = LSM6DSO_ACC_FS_4G,
            .odr = LSM6DSO_ACC_ODR_833_HZ,
            .lpf2_en = LSM6DSO_ACC_LPF2_DISABLE,
            .unit = LSM6DSO_FS_ACC_UNIT_DEFAULT
        },
        .gyro = {
            .fs = LSM6DSO_FS_GYRO_250_DPS,
            .odr = LSM6DSO_ODR_GYRO_833_HZ,
            .lpf1_en = LSM6DSO_GYRO_LPF1_ENABLE,
            .lpf1_mode = LSM6DSO_GYRO_LPF1_7,
            .hpf_en = LSM6DSO_GYRO_HPF_DISABLE,
            .unit = LSM6DSO_FS_GYRO_UNIT_DEFAULT,
            .offset_samples = 10000
        }
    };

    bmp390_configs_t bmp_configs = {  // TODO change with drone config handling refactor
        .i2c_wdt_en   = BMP390_IF_CONF_I2C_WDT_EN,
        .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_1250US,
        .i2c_handler  = &i2c_bmp_handler,
        .iir_coef     = BMP390_CONFIG_COEF_1,
        .odr_sel      = BMP390_ODR_SEL_12P5_HZ,
        .osr_press    = BMP390_OSR_P_X32,
        .osr_temp     = BMP390_OSR_T_X2,
        .press_en     = true,
        .temp_en      = true,
        .pwr_mode     = BMP390_PWR_CTRL_NORMAL_MODE
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_master_cfg, &i2c_master_handler);
    if(ret != ESP_OK) {
        return ret;
    }

    #define IGNORE_BMP 

    #ifndef IGNORE_BMP
    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_bmp_cfg, &i2c_bmp_handler);
    if(ret != ESP_OK) {
        return ret;
    }
    #endif

    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_imu_cfg, &i2c_imu_handler);
    if(ret != ESP_OK) {
        return ret;
    }

    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_bmi_cfg, &i2c_bmi_handler);
    if(ret != ESP_OK) {
        return ret;
    }

    drone->attributes.ts_ms = 1;   /* Overall sampling time of 1 millisecond */

    /* Initialize LSM6DSO object */
    drone->attributes.components.imu.init(&(drone->attributes.components.imu), imu_params);

    /* Initialize Bmi160 object */
    ESP_ERROR_CHECK( drone->attributes.components.bmi.init(
            &( drone->attributes.components.bmi ),
            &(i2c_bmi_handler),
            drone->attributes.config.imu_cfg.acc_mode,
            drone->attributes.config.imu_cfg.acc_freq,
            drone->attributes.config.imu_cfg.acc_range,
            drone->attributes.config.imu_cfg.gyro_mode,
            drone->attributes.config.imu_cfg.gyro_freq,
            drone->attributes.config.imu_cfg.gyro_range,
            0.0f,
            0.0f,
            0.0f
        )
    );

    #ifndef IGNORE_BMP
    /* Initiali<e BMP390 object */
    ESP_ERROR_CHECK( drone->attributes.components.bmp.init(
            &( drone->attributes.components.bmp ),
            bmp_configs,
            C,
            HPA,
            100
        )
    );
    #endif

    /* Fast offset compensation for bmi sensor */
    drone->attributes.components.bmi.foc( &( drone->attributes.components.bmi ) );
    #endif
    //drone->attributes.components.bmi.Gyro.calibrate( &( drone->attributes.components.bmi.Gyro ), 2000 );

    /* Initialize GNSS */

    const int uart_buffer_size = 1024;
    if(uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, &drone->attributes.components.gnss.uart_queue, 0) != ESP_OK) {
        printf("Failed to install UART driver\n");
        return ESP_FAIL;
    }

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    if(uart_param_config(UART_NUM_1, &uart_config) != ESP_OK) {
        printf("Failed to configure UART parameters\n");
        uart_driver_delete(UART_NUM_1);
        return ESP_FAIL;
    }

    
    if(uart_set_pin(UART_NUM_1, UART1_TX, UART1_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        printf("Failed to set UART pins\n");
        uart_driver_delete(UART_NUM_1);
        return ESP_FAIL;
    }


    gnss_params_t gnss_params = {
        .neoxm_version = GNSS_NEO_7M,
        .gnss_protocol = GNSS_PROTOCOL_DEFAULT,
        .measRate = GNSS_MEASRATE_DEFAULT,
        .uart_port = UART_NUM_1,
        .dynModel = UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN,
        .static_hold_threshold = GNSS_STATIC_HOLD_DEFAULT,
    };
    #ifndef IGNORE_GPS
    drone->attributes.components.gnss.init( // TODO change with drone config handling refactor
        &( drone->attributes.components.gnss ),
        gnss_params
    ); 
    #endif

    /* Initialize all Pwm objects */
    for( int i = 0; i < ( ( sizeof( drone->attributes.components.pwm ) ) / ( sizeof( drone->attributes.components.pwm[ 0 ] ) ) ); i++ ) {

        drone->attributes.components.pwm[ i ].init( &drone->attributes.components.pwm[ i ], drone->attributes.config.pwm_cfg[ i ] );
    }

    pid_gain_t emtpy_gains = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .kb = 0.0f };

    /* Initialize all Pid objects */
    for( int i = 0; i < ( ( sizeof( drone->attributes.components.controllers ) ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ) ); i++ ) {

        drone->attributes.components.controllers[ i ].init(
            &drone->attributes.components.controllers[ i ],
            i,
            drone->attributes.ts_ms,
            1.0f,
            emtpy_gains,
            drone->attributes.config.pid_cfgs[ i ].integral_limits,
            drone->attributes.config.pid_cfgs[ i ].pid_output_limits
        );
    }

    /* Initialize Mma object */
    drone->attributes.components.mma.init(&drone->attributes.components.mma);

    /* Blink MCU internal LED to indicate Drone object was successfully initialized */
    //gpio_set_level( GPIO_NUM_2, false );
    //vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    //gpio_set_level( GPIO_NUM_2, true );

    ESP_LOGI( DRONE_TAG, "Drone object initialized" );

    drone->methods.read_from_flash(drone);

    /* Drone object is initialized */
    drone->attributes.init_ok = true;
    
    return ESP_OK;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */
// Función para mantener el ángulo en [0, 2pi)
float wrapAngle360(float angle) {
    #define MAX 360.0f
    angle = fmodf(angle, MAX);
    if (angle < 0) angle += MAX;
    return angle;
}

/**
 * @brief Update Drone object states
 * @param drone: Address of Drone object
 * @param ts: Sampling time in milliseconds
 * @retval none
 */
float acc_x_filtered = 0;
float acc_y_filtered = 0;
float acc_z_filtered = 0;
float roll_filtered = 0;
static void UpdateStates( drone_t * drone, float ts ) {

    if( !drone->attributes.init_ok ) {

        /* If Drone object isn't initialized */
        ESP_LOGE( DRONE_TAG, "Drone object must be initialized before calling it's methods!. See %s in line %d", __func__, __LINE__ );
    }

    else {
        float acc_x = drone->attributes.components.imu.acc.x;
        float acc_y = drone->attributes.components.imu.acc.y;
        float acc_z = drone->attributes.components.imu.acc.z;

        float acc_filter_coeff = drone->attributes.global_variables.misc_floats[3];
        acc_x_filtered = FirstOrderIIR(drone->attributes.components.imu.acc.x, acc_x_filtered, ts / 1000.0f, acc_filter_coeff);
        acc_y_filtered = FirstOrderIIR(drone->attributes.components.imu.acc.y, acc_y_filtered, ts / 1000.0f, acc_filter_coeff);
        acc_z_filtered = FirstOrderIIR(drone->attributes.components.imu.acc.z, acc_z_filtered, ts / 1000.0f, acc_filter_coeff);


        float gyro_x = FirstOrderIIR( drone->attributes.components.imu.gyro.x, drone->attributes.states.roll_dot, ts / 1000.0f, drone->attributes.config.IIR_coeff_roll_dot );
        float gyro_y = FirstOrderIIR( drone->attributes.components.imu.gyro.y, drone->attributes.states.pitch_dot, ts / 1000.0f, drone->attributes.config.IIR_coeff_pitch_dot );
        float gyro_z = FirstOrderIIR( drone->attributes.components.imu.gyro.z*2, drone->attributes.states.yaw_dot, ts / 1000.0f, drone->attributes.config.IIR_coeff_yaw_dot );

        /* Apply first order IIR filter to gyroscope data */
        drone->attributes.states.roll_dot = gyro_x;
        drone->attributes.states.pitch_dot = gyro_y;
        drone->attributes.states.yaw_dot = gyro_z;
        
        /* Update state's position */
        float ALPHA = 0.95f;  // TODO: make this a parameter 

        float roll_acc = atan2( acc_y, acc_z ) * ( 180.0f / M_PI );
        float roll_acc_filtered = atan2( acc_y_filtered, acc_z_filtered ) * ( 180.0f / M_PI );

        float roll_gyro = drone->attributes.states.roll + ( gyro_x * ( ts / 1000.0f ) );

        roll_filtered = (1-ALPHA)*roll_acc_filtered + ALPHA*roll_gyro;
        drone->attributes.global_variables.misc_floats[4] = roll_filtered;

        drone->attributes.states.roll = (1-ALPHA)*roll_acc + ALPHA*roll_gyro;

        float pitch_acc = atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z)) * (180.0f / M_PI);
        float pitch_gyro = drone->attributes.states.pitch + (gyro_y * ( ts / 1000.0f) );
        drone->attributes.states.pitch = (1-ALPHA)*pitch_acc + ALPHA*pitch_gyro;

        drone->attributes.states.yaw = wrapAngle360(drone->attributes.states.yaw + (gyro_z * (ts / 1000.0f) ));

        drone->attributes.states.z_dot = drone->attributes.states.z_dot + ts*drone->attributes.components.imu.acc.z;
    }
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */
void Drone( drone_t * drone ) {

    ESP_LOGI( DRONE_TAG, "Making an instance of Drone Class..." );

    memset( drone, 0, sizeof( drone_t ) );
    drone->attributes.ts_ms = 1;
    drone->attributes.init_ok = false;
    
    /* Set Drone Class generic configs */
    drone->attributes.config = GetDroneConfigs();  // este GetDroneConfig está bien porque es el único que se tiene que usar

    /* Pointer to Drone functions ( methods ) */
    drone->methods.update_states    = UpdateStates;
    drone->methods.init             = drone_init;
    drone->methods.i2c_scan         = i2c_scan;
    drone->methods.read_from_flash  = read_from_nvs;
    drone->methods.save_to_nvs      = save_to_nvs;

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
            return;
        }

        else if( ret == ESP_FAIL ) {

            ESP_LOGE( DRONE_TAG, "Failed to mount spiffs partition" );
            return;
        }

        else {

            ESP_LOGE( DRONE_TAG, "Failed to initialize spiffs ( %s )", esp_err_to_name( ret ) );
            return;
        }
    }

    ESP_LOGI( DRONE_TAG, "Spiffs mounted successfully" );
    
    /* Make an instance of Bmi160 Class */
    #ifndef IGNORE_BMI
    Bmi160(&(drone->attributes.components.bmi));

    ESP_LOGI( DRONE_TAG, "BMI160 Init successful\n");

    Lsm6dso(&(drone->attributes.components.imu));

    Bmp390(&(drone->attributes.components.bmp));

    Gnss(&(drone->attributes.components.gnss));

    /* Check if all devices are connected to i2c bus */
    int retries = 3;
    bool i2c_ok = false;
    while (retries > 0){
        if (drone->methods.i2c_scan()) {
            retries = 0;
            i2c_ok = true;
        } else {
            retries--;
        }
        
    }
    if (!i2c_ok) {
        ESP_LOGE(DRONE_TAG, "Failed to find all devices in the I2C bus.");
        while (1) {
            drone->attributes.components.indicators.power.state = LED_BLINKING_FAST;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    #endif

    /* Make an instance of PrintManager*/
    PrintManager(&(drone->attributes.print_manager));

    extern print_output_t serial_output;
    extern print_output_t bluetooth_output;

    drone->attributes.print_manager.methods.register_output(
        &(drone->attributes.print_manager),
        &serial_output
    );
    
    drone->attributes.print_manager.methods.enable_output(
        &(drone->attributes.print_manager),
        "Serial"
    );

    drone->attributes.print_manager.methods.set_as_system_output(&(drone->attributes.print_manager));

    /* Make an instance of Battery */
    Battery(&(drone->attributes.components.battery));

    /* Make an instance of Indicators */
    Indicators(&(drone->attributes.components.indicators), 5000);

    /* Make an instance of Mma Class */

    Mma(&drone->attributes.components.mma);
    ESP_LOGI( DRONE_TAG, "MMA Init successful\n");

    /* Make an instance of Pid Class for all controllers */
    for(int i = 0; i < ( ( sizeof( drone->attributes.components.controllers ) ) / ( sizeof( drone->attributes.components.controllers[ 0 ] ) ) ); i++) {
        Pid( &drone->attributes.components.controllers[ i ], P_Basic, I_Clamping, D_LPF );
    }
    ESP_LOGI( DRONE_TAG, "PID controllers Init successful\n");
    /* Make an instance of Pwm Class for all pwm signals */
    for(int i = 0; i < ( sizeof( drone->attributes.components.pwm ) / sizeof( drone->attributes.components.pwm[ 0 ] ) ); i++) {
        Pwm(&drone->attributes.components.pwm[ i ], i);
    }
    ESP_LOGI( DRONE_TAG, "PWM signals Init successful\n");

    /* Make an instance of Transmitter Class */
    Transmitter( &drone->attributes.components.Tx, &( drone->attributes.global_variables ) );

    drone->attributes.components.Tx.methods.init( &drone->attributes.components.Tx, drone->attributes.config.esp_mac_addr );

    bluetooth_output.context = &drone->attributes.components.Tx;

    drone->attributes.print_manager.methods.register_output(
        &(drone->attributes.print_manager),
        &bluetooth_output
    );

    drone->attributes.print_manager.methods.enable_output(
        &(drone->attributes.print_manager),
        "Bluetooth"
    );

    /* =============== START Global variables assignment =============== */

    /* Assign Transmitter buttons global variable memmory address to 'GlobalTxButtons' variable */
    GlobalTxButtons = &drone->attributes.global_variables.tx_buttons;

    /* Assign Bluetooth data global variable memmory address to 'GlobalSerialData' variable */
    GlobalSerialData = &drone->attributes.global_variables.serial_data;

    /* Point 'GlobalRollGains' global variable to roll controller */
    GlobalRollGains = &(drone->attributes.components.controllers[ROLL].gain);

    /* Point 'GlobalRoll_dGains' global variable to roll_d controller */
    GlobalRoll_dGains = &(drone->attributes.components.controllers[ROLL_D].gain);

    /* Point 'GlobalPitchGains' global variable to pitch controller */
    GlobalPitchGains = &(drone->attributes.components.controllers[PITCH].gain);

    /* Point 'GlobalPitch_dGains' global variable to pitch_d controller */
    GlobalPitch_dGains = &(drone->attributes.components.controllers[PITCH_D].gain);

    /* Point 'GlobalYawGains' global variable to yaw controller */
    GlobalYawGains = &(drone->attributes.components.controllers[YAW].gain);

    /* Point 'GlobalYaw_dGains' global variable to yaw_d controller */
    GlobalYaw_dGains = &(drone->attributes.components.controllers[YAW_D].gain);

    /* Point 'GlobalZGains' global variable to z controller */
    // GlobalZGains = &(drone->attributes.components.controllers[Z].gain);

    /* Point 'GlobalZ_dGains' global variable to z_d controller */
    // GlobalZ_dGains = &(drone->attributes.components.controllers[Z_D].gain);

    /* =============== END Global variables assignment =============== */

    /* Initialize flash variable pointers to NULL */
    ret = get_drone_params(drone);
    if (ret != ESP_OK) {
        ESP_LOGE("Drone", "Error with params");
    }
    /* Blink MCU internal LED to indicate Transmitter object is ready to receive commands */
    gpio_set_level( GPIO_NUM_2, false );
    vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    gpio_set_level( GPIO_NUM_2, true );

    ESP_LOGI( DRONE_TAG, "Instance succesfully made" );

}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */

/**
 * @brief Initialize battery parameters
 * @param battery: Pointer to battery structure
 * @retval none
 */
void Battery( battery_t * battery ) {
    /* Initialize battery parameters */
    memset( battery, 0, sizeof( battery_t ) );
    battery->cells = 3;
}

/* ------------------------------------------------------------------------------------------------------------------------------------------ */
