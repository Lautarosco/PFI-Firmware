#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <i2c/interface_i2c.h>      /* I2C custom driver */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <math.h>

/* =========== Defines =========== */

#define BMP390_ADDR                         0x76            /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ                 100000          /* I2C SCL (clock) line frequency in Hz */
#define GPIO_SDA                            21              /* I2C SDA data line */
#define GPIO_SCL                            22              /* I2C SCL clock line */
#define BMP390_TEMP_UNIT                    C               /* Unit of measured temperature */
#define BMP390_PRESS_UNIT                   HPA             /* Unit of measured pressure */
#define BMP390_REL_PRESS_SAMPLES            100             /* Total samples to compute relative pressure */

/* =========== Functions prototypes =========== */

static esp_err_t estimate_altitude(double press0, double press, double *z);
void vTaskPrintAltitude(void *any);

/* =========== Main app =========== */

void app_main(void)
{
    /* 1. Make an instance of Bmp390 Class */
    bmp390_t bmp;
    Bmp390(&bmp);

    /* 2.a Set I2C master bus handler */
    i2c_master_bus_handle_t i2c_master_handler = NULL;      /* I2C master bus handler */

    /* 2.b Set I2C master bus configs */
    i2c_master_custom_t i2c_master = {
        .i2c_master_handler = &i2c_master_handler,
        .i2c_master_configs = {
            .clk_source                   = I2C_CLK_SRC_APB,
            .i2c_port                     = I2C_NUM_0,
            .scl_io_num                   = GPIO_SCL,
            .sda_io_num                   = GPIO_SDA,
            .glitch_ignore_cnt            = 7,
            .flags.enable_internal_pullup = true
        }
    };

    /* 3.a Set I2C device bus handler */
    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C BMP390 bus handler */

    /* 3.b Set I2C BMP390 bus configs */
    i2c_dev_custom_t i2c_bmp = {
        .i2c_dev_handler = &i2c_bmp_handler,
        .i2c_dev_configs = {
            .device_address          = BMP390_ADDR,
            .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
            .scl_speed_hz            = BMP390_I2C_SCL_F_HZ,
            .flags.disable_ack_check = false,
            .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
        }
    };

    /* 3.c Define I2C BMP390 interface settings */
    device_interface_t bmp_iface = {
        .dev_cfg     = &i2c_bmp,
        .master_cfg  = &i2c_master,
        .iface_sel   = I2C,
        .read_bytes  = i2c_read_bytes,
        .write_bytes = i2c_write_byte
    };

    /* 4. Define BMP390 modes of operation */
    bmp390_configs_t bmp_modes = {
        .i2c_wdt_en   = BMP390_IF_CONF_I2C_WDT_EN,
        .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_1250US,
        .iir_coef     = BMP390_CONFIG_COEF_3,
        .odr_sel      = BMP390_ODR_SEL_12P5_HZ,
        .osr_press    = BMP390_OSR_P_X32,
        .osr_temp     = BMP390_OSR_T_X1,
        .press_en     = true,
        .temp_en      = true,
        .pwr_mode     = BMP390_PWR_CTRL_NORMAL_MODE
    };

    /* 5. Initialize I2C master bus */
    i2c_init_master_bus(&(i2c_master.i2c_master_configs), &i2c_master_handler);

    /* 6. Add BMP390 to I2C bus */
    i2c_add_new_device(i2c_master_handler, &(i2c_bmp.i2c_dev_configs), &i2c_bmp_handler, bmp_iface.iface_sel);

    /* 7. Initialize BMP390 sensor */
    if(bmp.init(&bmp, &bmp_iface, bmp_modes, BMP390_TEMP_UNIT, BMP390_PRESS_UNIT, BMP390_REL_PRESS_SAMPLES) != ESP_OK) {
        return;
    }

    /* Altitude */
    double z = 0.0;
    double z0 = 0.0;
    double delta_z = 0.0;

    // estimate_altitude(bmp.press0, bmp.press, &z0);

    xTaskCreatePinnedToCore(vTaskPrintAltitude, "Task1", 1024 * 4, (void *) &bmp, 0, NULL, 0);

    while(1) {
        bmp.measure(&bmp);
        // estimate_altitude(bmp.press0, bmp.press, &z);
        // delta_z = z - z0;
        // printf("T: %lf °C\t P: %lf hPa\t Z: %lf cm\n", bmp.temp, bmp.press, z * 100.0);

        //printf("Altitude (Relative to z0): %lf cm\n", z * 100.0);

        vTaskDelay(pdMS_TO_TICKS(80));
    }
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

void vTaskPrintAltitude(void *any) {
    bmp390_t *bmp = (bmp390_t *) any;

    while(1) {
        // printf("Altitude (relative to z0): %lf cm\n", (*_z) * 100.0);

        printf("P: %lf hPa\n", bmp->press);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
