#include <stdio.h>
#include <application_layer/bmp390_app_layer.h>

#include <i2c/interface_i2c.h>      /* I2C custom driver */

#define BMP390_ADDR             0x76        /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ     100000      /* I2C SCL (clock) line frequency in Hz */
#define GPIO_SDA                21          /* I2C SDA data line */
#define GPIO_SCL                22          /* I2C SCL clock line */

void app_main(void)
{
    /* 1. Make an instance of Bmp390 Class */
    bmp390_t bmp;
    Bmp390(&bmp);

    /* 2. Set I2C master configs */
    i2c_master_bus_handle_t i2c_master_handler = NULL;      /* I2C master bus handler */

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

    /* 3. Set I2C device configs */
    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C BMP390 bus handler */

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

    device_interface_t bmp_iface = {
        .dev_cfg     = &i2c_bmp,
        .master_cfg  = &i2c_master,
        .iface_sel   = I2C,
        .read_bytes  = i2c_read_bytes,
        .write_bytes = i2c_write_byte
    };


    bmp390_configs_t bmp_configs = {
        .i2c_wdt_en   = BMP390_IF_CONF_I2C_WDT_EN,
        .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_1250US,
        .iir_coef     = BMP390_CONFIG_COEF_3,
        .odr_sel      = BMP390_ODR_SEL_50_HZ,
        .osr_press    = BMP390_OSR_P_X8,
        .osr_temp     = BMP390_OSR_T_X1,
        .press_en     = true,
        .temp_en      = true,
        .pwr_mode     = BMP390_PWR_CTRL_NORMAL_MODE
    };

    /* 3. Initialize I2C master bus */
    i2c_init_master_bus(&(i2c_master.i2c_master_configs), &i2c_master_handler);

    /* 4. Add BMP390 to I2C bus */
    i2c_add_new_device(i2c_master_handler, &(i2c_bmp.i2c_dev_configs), &i2c_bmp_handler, bmp_iface.iface_sel);

    /* 5. Initialize BMP390 sensor */
    bmp.init(&bmp, &bmp_iface, bmp_configs);
}
