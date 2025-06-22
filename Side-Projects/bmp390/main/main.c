#include <stdio.h>
#include <bmp390.h>

#include <i2c/interface_i2c.h>      /* I2C custom driver */

#define BMP390_ADDR             0x76        /* SDO = 0 --> Device address is 0b1110110 = 0x76 */
#define BMP390_I2C_SCL_F_HZ     100000      /* I2C SCL (clock) line frequency in Hz */
#define GPIO_SDA                21          /* I2C SDA data line */
#define GPIO_SCL                22          /* I2C SCL clock line */

void app_main(void)
{
    bmp390_t bmp;
    Bmp390(&bmp);       /* Make an instance of Bmp390 Class */

    i2c_master_bus_handle_t i2c_master_handler = NULL;      /* I2C master bus handler */
    i2c_master_dev_handle_t i2c_bmp_handler = NULL;         /* I2C bmp390 bus handler */

    i2c_custom_t i2c_configs = {
        .i2c_dev_handler = i2c_bmp_handler,
        .i2c_dev_configs = {
            .device_address          = BMP390_ADDR,
            .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
            .scl_speed_hz            = BMP390_I2C_SCL_F_HZ,
            .flags.disable_ack_check = false,
            .scl_wait_us             = BMP390_IF_CONF_I2C_WDT_SEL_1250US
        },
        .i2c_master_handler = i2c_master_handler,
        .i2c_master_configs = {
            .clk_source                   = I2C_CLK_SRC_APB,
            .i2c_port                     = I2C_NUM_0,
            .scl_io_num                   = GPIO_SCL,
            .sda_io_num                   = GPIO_SDA,
            .glitch_ignore_cnt            = 7,
            .flags.enable_internal_pullup = true
        }
    };

    device_interface_t bmp_iface = {
        .iface_cfg = &i2c_configs,
        .iface_sel = I2C,
        .read_bytes = i2c_read_bytes,
        .write_bytes = i2c_write_byte
    };

    bmp390_configs_t bmp_configs = {
        .i2c_wdt_en = BMP390_IF_CONF_I2C_WDT_EN,
        .i2c_wdt_tout = BMP390_IF_CONF_I2C_WDT_SEL_1250US,
        .iir_coef = 
    };

    bmp.bmp390_hal_init(&bmp, bmp_iface, );
}