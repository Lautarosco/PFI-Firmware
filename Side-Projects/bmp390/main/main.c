#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <math.h>



/* =========== Defines =========== */

#define BMP390_ADDR                         0x76            /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ                 100000          /* I2C SCL (clock) line frequency in Hz */
#define GPIO_SDA                            21              /* I2C SDA data line */
#define GPIO_SCL                            22              /* I2C SCL clock line */
#define VIRTUAL_TEMP_K                      293.87          /* Virtual temperature See (https://www.weather.gov/epz/wxcalc_virtualtemperature) */
#define GAS_CONSTANT_R                      287.05          /* Gas constant for dry air in J/(kg·K) */
#define GRAVITY_ACCELERATION_G              9.80665         /* Standard gravity in m/s² */


/* =========== Functions prototypes =========== */

static esp_err_t estimate_altitude(double press0, double press, double *z);
static esp_err_t hypsometric_eqn(double press0, double press, double *z);
void vTaskPrintAltitude(void *any);


typedef struct {
    bmp390_t *bmp;
    double *z;
    double *z0;
} bmp_wrapper_t;

void bmp_callback(TimerHandle_t xTimer) {
    void *pv = pvTimerGetTimerID(xTimer);
    bmp_wrapper_t *bmp_wrapper = (bmp_wrapper_t *) pv;

    bmp_wrapper->bmp->measure(bmp_wrapper->bmp);
    hypsometric_eqn(bmp_wrapper->bmp->press0, bmp_wrapper->bmp->press, bmp_wrapper->z);
}



/* =========== Main app =========== */

void app_main(void)
{
    /* 1. Make an instance of Bmp390 Class */
    static bmp390_t bmp;
    Bmp390(&bmp);

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

    /* 3.a Set I2C device bus handler */
    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C BMP390 bus handler */

    /* 3.b Set I2C BMP390 bus configs */
    i2c_device_config_t i2c_bmp_cfg = {
        .device_address          = BMP390_ADDR,
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

    /* 6. Add BMP390 to I2C bus */
    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_bmp_cfg, &i2c_bmp_handler);
    if(ret != ESP_OK) {
        return;
    }

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

    static bmp_wrapper_t bmp_wrapper = {
        .bmp = &bmp,
        .z = &z,
        .z0 = &z0
    };

    TimerHandle_t bmp_timer = xTimerCreate("bmp_timer", pdMS_TO_TICKS(80), pdTRUE, (void *) &bmp_wrapper, bmp_callback);
    xTimerStart(bmp_timer, 0);

    xTaskCreatePinnedToCore(vTaskPrintAltitude, "Task1", 1024 * 4, &bmp_wrapper, 0, NULL, 0);

    return;
}



/* =========== Functions implementations =========== */

/**
 * @brief Estimate altitude (meters) based on relative pressure (hPa) and last pressure measurement
 * 
 * @note Equation was taken from BOSCH BMP180 datasheet. See (https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf, p. 16, Sec. 3.6)
 * 
 * @param press0: Relative pressure
 * @param press: Last pressure measurement
 * @param z: Pointer altitude variable to store result
 * 
 * @retval
 *      - ESP_ERR_INVALID_ARG: press0 is less or equal to 0
 *      - ESP_OK: Success
 */
static esp_err_t estimate_altitude(double press0, double press, double *z) {
    if(press0 <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *z = 44330.0 * (1 - pow(press / press0, 1.0 / 2.255));

    return ESP_OK;
}

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

void vTaskPrintAltitude(void *any) {
    bmp_wrapper_t *bmp_wrapper = (bmp_wrapper_t *) any;

    while(1) {
        printf("T: %lf °C, P: %lf hPa, Altitude: %lf cm\n", bmp_wrapper->bmp->temp, bmp_wrapper->bmp->press, (*(bmp_wrapper->z) - *(bmp_wrapper->z0)) * 100.0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
