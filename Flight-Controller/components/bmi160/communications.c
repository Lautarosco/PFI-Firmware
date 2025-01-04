#include "registers.h"
#include "communication.h"

#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_NUM       I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   200000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master do not need buffer */

#define WRITE_BIT            I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT             I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN         0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS        0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL              0x0              /*!< I2C ack value */
#define NACK_VAL             0x1              /*!< I2C nack value */

static const char * TAG = "BMI160/communications"; 

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_init(int sda, int scl) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief Write a byte to the given register
 */
esp_err_t bmi160_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read a sequence of bytes from a sensor register
 */
 esp_err_t bmi160_read_bytes(uint8_t sensor_addr, uint8_t reg_addr, uint8_t *data, uint8_t len) {
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret;
    i2c_cmd_handle_t cmd;

    // Start a command link transaction for the address setting phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | WRITE_BIT, ACK_CHECK_EN);  // Send the device address with the write option
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);                     // Send the register address
    i2c_master_stop(cmd);                                                   // Stop after writing the register address
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    if (ret != ESP_OK) {
        return ret;  // Return error if address setting phase failed
    }

    // Start another command link transaction for the reading phase
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensor_addr << 1 | READ_BIT, ACK_CHECK_EN);  // Send the device address with the read option
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, ACK_VAL);                       // Read data bytes with ACK
    }
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);                    // Read the last byte with NACK
    i2c_master_stop(cmd);                                                   // Stop after reading the data
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);  // Execute the command
    i2c_cmd_link_delete(cmd);                                               // Clean up command link

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: Couldn't read data. %d\n", ret);
    }

    return ret;  // Return the result of the reading phase
}
