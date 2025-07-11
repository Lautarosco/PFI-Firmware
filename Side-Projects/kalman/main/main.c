#include <stdio.h>

#include <application_layer/bmp390_app_layer.h>
#include <application_layer/gnss_app_layer.h>
#include <bmi160.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <math.h>


/* =========== Defines =========== */

#define BMP390_ADDR                         0x76            /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ                 100000          /* I2C SCL (clock) line frequency in Hz */
#define VIRTUAL_TEMP_K                      293.87          /* Virtual temperature See (https://www.weather.gov/epz/wxcalc_virtualtemperature) */
#define GAS_CONSTANT_R                      287.05          /* Gas constant for dry air in J/(kg·K) */
#define GRAVITY_ACCELERATION_G              9.80665         /* Standard gravity in m/s² */

#define GPIO_SDA                            21              /* I2C SDA data line */
#define GPIO_SCL                            22              /* I2C SCL clock line */
#define UART_NUM                            UART_NUM_1      /* UART port */
#define UART_TX_PIN                         17              /* UART Tx GPIO */
#define UART_RX_PIN                         16              /* UART Rx GPIO */


/* =========== Structs =========== */

typedef struct {
    bmp390_t *bmp;
    double *z;
    double *z0;
} bmp_wrapper_t;

typedef struct {
    gnss_t *gnss;
    QueueHandle_t queue;
} xGnss_wrapper_t;


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

void bmp_callback(TimerHandle_t xTimer) {
    void *pv = pvTimerGetTimerID(xTimer);
    bmp_wrapper_t *bmp_wrapper = (bmp_wrapper_t *) pv;

    bmp_wrapper->bmp->measure(bmp_wrapper->bmp);
    hypsometric_eqn(bmp_wrapper->bmp->press0, bmp_wrapper->bmp->press, bmp_wrapper->z);
}

void bmi_callback(TimerHandle_t xTimer) {
    void *pv = pvTimerGetTimerID(xTimer);
    bmi160_t *bmi = (bmi160_t *) pv;

    bmi->measure(bmi);
}

void vTaskPrintAltitude(void *any) {
    bmp_wrapper_t *bmp_wrapper = (bmp_wrapper_t *) any;

    while(1) {
        printf("T: %lf °C, P: %lf hPa, Altitude: %lf cm\n", bmp_wrapper->bmp->temp, bmp_wrapper->bmp->press, (*(bmp_wrapper->z) - *(bmp_wrapper->z0)) * 100.0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskPrintAttitude(void *_bmi) {
    bmi160_t *bmi = (bmi160_t *) _bmi;

    while(1) {
        printf("z': %f m/s\t z'': %f m/s2\n", bmi->Gyro.z, bmi->Acc.z);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void xTask_uart_event(void *pvParameters) {
    xGnss_wrapper_t *wrapper = (xGnss_wrapper_t *) pvParameters;
    uart_event_t event;

    while(1) {
        if(xQueueReceive(wrapper->queue, &event, portMAX_DELAY)) {
            switch(event.type) {
                case UART_DATA:
                    if(event.size > 0) {
                        wrapper->gnss->measure(wrapper->gnss);
                    }
                    break;
                default:
                    break;
            }
        }
    }

    vTaskDelete(NULL);
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

    const int uart_buffer_size = 1024;
    QueueHandle_t uart_queue;
    if(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0) != ESP_OK) {
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

    gnss_t gnss;
    gnss_params_t gnss_params = {
        .neoxm_version = GNSS_NEO_7M,
        .gnss_protocol = GNSS_PROTOCOL_DEFAULT,
        .measRate = GNSS_MEASRATE_DEFAULT,
        .uart_port = UART_NUM,
        .dynModel = UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN,
        .static_hold_threshold = GNSS_STATIC_HOLD_DEFAULT,
    };

    Gnss(&gnss);
    ret = gnss.init(&gnss, gnss_params);
    if(ret != ESP_OK){
        return;
    }

    /* ========== BMP390 ========== */

    /* 1. Make an instance of Bmp390 Class */
    static bmp390_t bmp;
    Bmp390(&bmp);
    
    /* 7. Initialize BMP390 sensor */
    ret = bmp.init(&bmp, bmp_configs, C, HPA, 100);
    if(ret != ESP_OK) {
        return;
    }

    /* Altitude */
    static double z = 0.0;
    static double z0 = 0.0;

    hypsometric_eqn(bmp.press0, bmp.press, &z0);
    printf("z0: %lf cm\n", z0 * 100.0);

    /* ========== BMI160 ========== */

    static bmi160_t bmi;
    Bmi160(&bmi);

    ret = bmi.init(
        &bmi, &i2c_bmi_handler,
        BMI160_CMD_ACC_NORMAL_MODE, BMI160_ACC_CONF_100HZ_NORMAL, BMI160_ACC_RANGE_4G,
        BMI160_CMD_GYRO_NORMAL_MODE, BMI160_GYRO_CONF_100HZ_NORMAL, BMI160_GYRO_RANGE_250DPS,
        0, 0, 0
    );
    if(ret != ESP_OK) {
        return;
    }
    ret = bmi.foc(&bmi);
    if(ret != ESP_OK) {
        return;
    }
    
    /* ========== Timer/Task BMP390 ========== */

    static bmp_wrapper_t bmp_wrapper = {
        .bmp = &bmp,
        .z = &z,
        .z0 = &z0
    };

    TimerHandle_t bmp_timer = xTimerCreate("bmp_timer", pdMS_TO_TICKS(80), pdTRUE, &bmp_wrapper, bmp_callback);
    xTimerStart(bmp_timer, 0);
    xTaskCreatePinnedToCore(vTaskPrintAltitude, "Task1", 1024 * 4, &bmp_wrapper, 0, NULL, 0);

    /* ========== Task NEO-7M ========== */

    xGnss_wrapper_t uart_wrapper = {
        .gnss = &gnss,
        .queue = uart_queue
    };

    xTaskCreate(xTask_uart_event, "uart_task_event", 1024 * 4, &uart_wrapper, 10, NULL);

    /* ========== Timer/Task BMI160 ========== */

    TimerHandle_t bmi_timer = xTimerCreate("bmi_timer", pdMS_TO_TICKS(10), pdTRUE, &bmi, bmi_callback);
    xTimerStart(bmi_timer, 0);
    xTaskCreatePinnedToCore(vTaskPrintAttitude, "Task2", 1024 * 4, &bmi, 0, NULL, 0);


    while(1) {
        printf(
            "%s\t numSV: %u\t hEl: %lf cm\t hMSL: %lf cm\t vDOP: %lf\t Latitude: %lf°\t Longitude: %lf°\n",
            gnss.data.time.utc_timestamp, gnss.data.svs_data.numSV, gnss.data.position.height * 100, gnss.data.position.hMSL * 100, gnss.data.position.vDOP, gnss.data.position.lat, gnss.data.position.lon
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return;
}
