#ifndef I2C_CUSTOM_H
#define I2C_CUSTOM_H

#include <driver/i2c_master.h>  /* ESP-IDF I2C driver */

typedef struct i2c_dev_custom {
    // i2c_master_bus_handle_t *i2c_master_handler;        /* I2C master handler */
    // i2c_master_bus_config_t i2c_master_configs;         /* I2C master configs */
    i2c_device_config_t i2c_dev_configs;                /* I2C device configs */
    i2c_master_dev_handle_t *i2c_dev_handler;           /* I2C device handler */
} i2c_dev_custom_t;

typedef struct i2c_master_custom {
    i2c_master_bus_handle_t *i2c_master_handler;        /* I2C master handler */
    i2c_master_bus_config_t i2c_master_configs;         /* I2C master configs */
    // i2c_device_config_t i2c_dev_configs;                /* I2C device configs */
    // i2c_master_dev_handle_t *i2c_dev_handler;           /* I2C device handler */
} i2c_master_custom_t;

#endif
