/* General headers */
#include <stdio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

/* Components headers */
#include <bmp280.h>     /* bmp280 header */
#include <serial.h>     /* Serial interfaces driver */

#define BMP280_ADDR             0x76    /* Sensor address - 0x76 if SDO = 0 or 0x77 if SDO = 1 */
#define BMP280_I2C_SCL_F_HZ     100000  /* I2C SCL (clock) line frequency in Hz */
#define GPIO_SDA                21      /* I2C SDA data */
#define GPIO_SCL                22      /* I2C SCL clock */

static const char *main_tag = "MAIN APP";  /* Main tag */

void vTaskBmp280Measure(void *bmp);


void app_main(void) {
    esp_err_t ret;

    /* ======== I2C master setup ======== */
    i2c_master_bus_handle_t master_i2c_handler = NULL;  /* I2C bus handler for master device (MCU) */
    ret = i2c_init(&master_i2c_handler, GPIO_SDA, GPIO_SCL);   /* Initialize I2C interface */
    if(ret != ESP_OK) {
        ESP_LOGE(main_tag, "%s in line %d: Failed to initialice I2C interface", __func__, __LINE__);
        return;
    }

    /* ======== bmp I2C setup ======== */
    i2c_master_dev_handle_t bmp280_i2c_handler = NULL; /* I2C bus bmp280 handler */

    /* bmp serial configs */
    dev_serial_iface_t bmp_serial_iface = {
        .i2c_cfg = {
            .device_address          = BMP280_ADDR,             /* bmp serial address */
            .dev_addr_length         = I2C_ADDR_BIT_LEN_7,      /* bmp address length */
            .scl_speed_hz            = BMP280_I2C_SCL_F_HZ,     /* SCL line frequency */
            .flags.disable_ack_check = false,                   /* Enable ACK detection */
            .scl_wait_us             = 0                        /* Default response timeout */  
        },
        .type                        = IFACE_I2C,               /* Type of serial interface used */
        .handler                     = &bmp280_i2c_handler,     /* bmp serial (I2C) handler */
        .read_func                   = i2c_read_bytes,          /* Serial (I2C) read function */
        .write_func                  = i2c_write_bytes          /* Serial (I2C) write function */
    };

    ret = i2c_add_new_device(master_i2c_handler, &(bmp_serial_iface.i2c_cfg), bmp_serial_iface.handler, bmp_serial_iface.type);
    if(ret != ESP_OK) {
        return;
    }
    
    bmp280_t bmp;
    Bmp280(&bmp);   /* Make an instance of Bmp280 Class */

    if(bmp.init(&bmp, &bmp_serial_iface) != ESP_OK) {
        return;
    }

    double p0 = bmp.get_avg_pressure(bmp, 1500);    /* n = 1500 samples ~ 1 minute */
    // double p0 = 1015.867004; /* Relative pressure (depends on location) */
    double z0 = bmp.get_avg_altitude(bmp, p0, 1500);
    // double t = 0.0;
    double p = 0.0;
    double z = 0.0;

    xTaskCreatePinnedToCore(vTaskBmp280Measure, "task1", 1024 * 2, (void *) &bmp, 1, NULL, 1);

    char buff[1024];
    
    while(1) {

        // t = bmp.get_temperature();
        p = bmp.get_pressure();
        z = bmp.get_altitude(p, p0) - z0;

        printf("printer:z,%f\n", z);

        // printf("Temperature: %lf °C\tPressure: %lf hPa\tAltitude: %lf m\r\n", t, p, z);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskBmp280Measure(void *_bmp) {
    bmp280_t *bmp = (bmp280_t *) _bmp;

    while(1) {
        bmp->measure(bmp->serial_iface);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
