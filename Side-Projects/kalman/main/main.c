#include <stdio.h>

#include <application_layer/bmp390_app_layer.h>
#include <application_layer/gnss_app_layer.h>
#include <bmi160.h>
#include <kalman.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <math.h>
#include <string.h>


/* =========== Defines =========== */

#define BMP390_ADDR                         0x76            /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ                 100000          /* I2C SCL (clock) line frequency in Hz */
#define VIRTUAL_TEMP_K                      293.87          /* Virtual temperature See (https://www.weather.gov/epz/wxcalc_virtualtemperature) */
#define GAS_CONSTANT_R                      287.05          /* Gas constant for dry air in J/(kg·K) */
#define GRAVITY_ACCELERATION_G              9.80665         /* Standard gravity in m/s² */

#define BMP390_SAMPLING_RATE                85              /* BMP390 sampling rate in ms */
#define BMI160_SAMPLING_RATE                10              /* BMI160 sampling rate in ms */

#define GPIO_SDA                            21              /* I2C SDA data line */
#define GPIO_SCL                            22              /* I2C SCL clock line */
#define UART_NUM                            UART_NUM_1      /* UART port */
#define UART_TX_PIN                         17              /* UART Tx GPIO */
#define UART_RX_PIN                         16              /* UART Rx GPIO */


const char *main_tag = "[MAIN]";


/* =========== Structs =========== */

typedef struct {
    float z;                /* State: Altitude [m] */
    float vz;               /* State: Altitude velocity [m/s] */
} states_t;

typedef struct {
    bmp390_t bmp;           /* BMP390 Barometric pressure sensor */
    bmi160_t bmi;           /* BMI160 Inertial Measurements Unit */
    gnss_t gnss;            /* NEO-7M Global Navigation Satellite System */
} sensors_t;

typedef struct {
    float z0_bmp;               /* Relative altitude measurement [m] for BMP390 sensor */
    float z0_gnss;              /* Relative altitude measurement [m] for NEO-7M sensor */
} relative_meas_t;

typedef struct {
    sensors_t sensors;                      /* Sensors */
    states_t states;                        /* States */
    relative_meas_t rel_meas;               /* Relative measurements for each sensor */
    kalman_altitude_t kf_alt;               /* Kalman Filter for altitude */
} drone_t;


/* =========== Functions =========== */

/**
 * @brief Calculate altitude (meters) using the hypsometric equation
 * 
 * @param press0: Reference pressure (hPa)
 * @param press: Current pressure (hPa)
 * @param z: Pointer altitude variable to store result
 * 
 * @retval
 *      - ESP_ERR_INVALID_ARG: press0 is less or equal to 0
 *      - ESP_OK: Success
 */
static esp_err_t hypsometric_eqn(double press0, double press, double *z) {
    if(press0 <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Calculate altitude in meters */
    *z = ((GAS_CONSTANT_R * VIRTUAL_TEMP_K) / GRAVITY_ACCELERATION_G) * log(press0 / press);

    return ESP_OK;
}

void xTask_process_bmp(void *pvParameters) {
    drone_t *drone = (drone_t *) pvParameters;
    double z_tmp = 0.0;

    while(1){
        /* Calculate altitude using Hypsometric formulae */
        hypsometric_eqn(drone->sensors.bmp.press0, drone->sensors.bmp.press, &z_tmp);

        /* Calculate relative altitude with respect to first measurement, z0 */
        float z_bmp = ((float) z_tmp) - drone->rel_meas.z0_bmp;

        /* Update Kalman Filter with BMP390 estimated altitude */
        kalman_update(&(drone->kf_alt), z_bmp, 0.5f, &(drone->states.z), &(drone->states.vz));

        vTaskDelay(pdMS_TO_TICKS(BMP390_SAMPLING_RATE));
    }
}

void xTask_process_bmi(void *pvParameters) {
    drone_t *drone = (drone_t *) pvParameters;

    while(1) {
        /* Predict altitude based on accelerometer measure */
        kalman_predict(&(drone->kf_alt), drone->sensors.bmi.Acc.z, BMI160_SAMPLING_RATE / 1000.0f, &(drone->states.z), &(drone->states.vz));

        vTaskDelay(pdMS_TO_TICKS(BMI160_SAMPLING_RATE));
    }
}

void bmp_callback(TimerHandle_t xTimer) {
    void *pv = pvTimerGetTimerID(xTimer);
    drone_t *drone = (drone_t *) pv;

    /* Read updated BMP390 registers and update pressure and temperature values */
    drone->sensors.bmp.measure(&(drone->sensors.bmp));
}

void bmi_callback(TimerHandle_t xTimer) {
    void *pv = pvTimerGetTimerID(xTimer);
    drone_t *drone = (drone_t *) pv;

    /* Read updated BMI160 registers and update accelerations and velocity values */
    drone->sensors.bmi.measure(&(drone->sensors.bmi));
}

static void xTask_uart_event(void *pvParameters) {
    drone_t *drone = (drone_t *) pvParameters;
    uart_event_t event;

    esp_err_t ret = ESP_OK;
    float z_gnss = 0.0;

    /* Calculate z0 for NEO-7M sensor */
    do
    {
        drone->sensors.gnss.measure(&(drone->sensors.gnss));

        vTaskDelay(pdMS_TO_TICKS(GNSS_MEASRATE_DEFAULT));
    } while ((!drone->sensors.gnss.data.flags.valid_gnss_fix) && (!drone->sensors.gnss.data.flags.valid_gnss_fix_type));
    drone->rel_meas.z0_gnss = drone->sensors.gnss.data.position.hMSL;

    while(1) {
        if(xQueueReceive(drone->sensors.gnss.uart_queue, &event, portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    if(event.size > 0) {
                        ret = drone->sensors.gnss.measure(&(drone->sensors.gnss));
                        if((ret == ESP_OK) && (drone->sensors.gnss.data.flags.valid_gnss_fix) && (drone->sensors.gnss.data.flags.valid_gnss_fix_type)) {
                            /* Calculate relative altitude with respect to first measurement, z0 */
                            z_gnss = ((float) drone->sensors.gnss.data.position.hMSL) - drone->rel_meas.z0_gnss;

                            /* Update Kalman Filter with NEO-7M measured altitude */
                            kalman_update(&(drone->kf_alt), z_gnss, 3.0f, &(drone->states.z), &(drone->states.vz));
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}


/* =========== Main =========== */

void app_main(void) {
    /* ========== I2C initialization ========== */

    /* 2.a Set I2C master bus handler */
    i2c_master_bus_handle_t i2c_master_handler = NULL;      /* I2C master bus handler */

    /* 2.b Set I2C master bus configs */
    i2c_master_bus_config_t i2c_master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = GPIO_SCL,
        .sda_io_num                   = GPIO_SDA,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    /* 3.a Set I2C devices bus handlers */
    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C BMP390 bus handler */
    i2c_master_dev_handle_t i2c_bmi_handler = NULL;         /* I2C BMI160 bus handler */

    /* 3.b Set I2C BMP390 and BMI160 bus configs */
    i2c_device_config_t i2c_bmp_cfg = {
        .device_address          = BMP390_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = BMP390_I2C_SCL_F_HZ,
        .flags.disable_ack_check = false,
        .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
    };

    i2c_device_config_t i2c_bmi_cfg = {
        .device_address          = BMI160_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = BMP390_I2C_SCL_F_HZ,
        .flags.disable_ack_check = false,
        .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
    };

    /* 4. Define BMP390 modes of operation */
    bmp390_configs_t bmp_configs = {
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

    esp_err_t ret = ESP_OK;

    /* 5. Initialize I2C master bus */
    ret = i2c_new_master_bus(&i2c_master_cfg, &i2c_master_handler);
    if(ret != ESP_OK) {
        return;
    }

    /* 6. Add BMP390 and BMI160 to I2C bus */
    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_bmp_cfg, &i2c_bmp_handler);
    if(ret != ESP_OK) {
        return;
    }

    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_bmi_cfg, &i2c_bmi_handler);
    if(ret != ESP_OK) {
        return;
    }

    /* ========== UART initialization ========== */

    /* Define a drone instance and initialize it to 0 */
    drone_t drone;
    memset(&drone, 0, sizeof(drone_t));
    
    const int uart_buffer_size = 1024;
    // QueueHandle_t uart_queue;
    if(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &(drone.sensors.gnss.uart_queue), 0) != ESP_OK) {
        printf("Failed to install UART driver\n");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    if(uart_param_config(UART_NUM, &uart_config) != ESP_OK) {
        printf("Failed to configure UART parameters\n");
        uart_driver_delete(UART_NUM);
        return;
    }

    
    if(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        printf("Failed to set UART pins\n");
        uart_driver_delete(UART_NUM);
        return;
    }

    printf("UART driver installed and configured successfully\n");

    /* ========== GNSS NEO-7M ========== */

    gnss_params_t gnss_params = {
        .neoxm_version = GNSS_NEO_7M,
        .gnss_protocol = GNSS_PROTOCOL_DEFAULT,
        .measRate = GNSS_MEASRATE_DEFAULT,
        .uart_port = UART_NUM,
        .dynModel = UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN,
        .static_hold_threshold = GNSS_STATIC_HOLD_DEFAULT,
    };

    /* Make an instance of Gnss Class */
    Gnss(&(drone.sensors.gnss));

    ret = drone.sensors.gnss.init(&(drone.sensors.gnss), gnss_params);
    if(ret != ESP_OK){
        return;
    }

    /* ========== BMP390 ========== */

    /* 1. Make an instance of Bmp390 Class */
    Bmp390(&(drone.sensors.bmp));
    
    /* 7. Initialize BMP390 sensor */
    ret = drone.sensors.bmp.init(&(drone.sensors.bmp), bmp_configs, C, HPA, 100);
    if(ret != ESP_OK) {
        return;
    }

    /* ========== BMI160 ========== */

    Bmi160(&(drone.sensors.bmi));

    ret = drone.sensors.bmi.init(
        &(drone.sensors.bmi), &i2c_bmi_handler,
        BMI160_CMD_ACC_NORMAL_MODE, BMI160_ACC_CONF_100HZ_NORMAL, BMI160_ACC_RANGE_4G,
        BMI160_CMD_GYRO_NORMAL_MODE, BMI160_GYRO_CONF_100HZ_NORMAL, BMI160_GYRO_RANGE_250DPS,
        0, 0, 0
    );
    if(ret != ESP_OK) {
        return;
    }
    ret = drone.sensors.bmi.foc(&(drone.sensors.bmi));
    if(ret != ESP_OK) {
        return;
    }
    
    /* ========== Initialize Kalman ========== */

    /* Calculate z0 for BMP390 sensor */
    hypsometric_eqn(drone.sensors.bmp.press0, drone.sensors.bmp.press, (double *) &(drone.rel_meas.z0_bmp));
    ESP_LOGI(main_tag, "Initial BMP390 z0: %f", drone.rel_meas.z0_bmp);

    /* Initialize Kalman Filter for altitude */
    kalman_alt_init(&(drone.kf_alt), 0.1f, 0.5f, 1.0f, 1.0f);

    /* ========== Sensor processing task ========== */

    xTaskCreate(xTask_process_bmp, "xTask_process_bmp", 1024 * 4, &drone, 0, NULL);
    xTaskCreate(xTask_process_bmi, "xTask_process_bmi", 1024 * 4, &drone, 0, NULL);

    /* ========== Timer/Task BMP390 ========== */

    TimerHandle_t bmp_timer = xTimerCreate("bmp_timer", pdMS_TO_TICKS(BMP390_SAMPLING_RATE), pdTRUE, &drone, bmp_callback);
    xTimerStart(bmp_timer, 0);

    /* ========== Task NEO-7M ========== */

    xTaskCreate(xTask_uart_event, "xTask_uart_event", 1024 * 5, &drone, 10, NULL);

    /* ========== Timer/Task BMI160 ========== */

    TimerHandle_t bmi_timer = xTimerCreate("bmi_timer", pdMS_TO_TICKS(BMI160_SAMPLING_RATE), pdTRUE, &drone, bmi_callback);
    xTimerStart(bmi_timer, 0);

    while(1) {
        printf(
            "printer:z,%f|vel_z,%f\n",
            drone.states.z, drone.states.vz
        );

        vTaskDelay(pdMS_TO_TICKS(BMI160_SAMPLING_RATE));
    }

    return;
}
