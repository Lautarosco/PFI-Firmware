#include <bmp280_ll_drivers/communications/com.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define I2C_MASTER_FREQ_HZ      100000      /* I2C clock speed */
#define ACK_EN                  0x01        /* Enable ACK */

static const char * i2c_tag = "I2C";

esp_err_t i2c_init(int sda, int scl) {
    ESP_LOGI(i2c_tag, "Initializing I2C interface...");

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    if(i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) {
        ESP_LOGE(i2c_tag, "Failed to set I2C configs");
        return ESP_FAIL;
    }
    
    if(i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(i2c_tag, "Failed to install I2C drivers");
        return ESP_FAIL;
    }

    ESP_LOGI(i2c_tag, "I2C interface successfully initialized.");

    return ESP_OK;
}

esp_err_t i2c_read_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t * buff, int len) {

    /**
     * I2C Read
     * 
     * 1. Send 111011x0 (write mode)
     * 2. Send register address (0xF6)
     * 3. Wait for a stop or repeated start condition
     * 4. Send 111011x1 (read mode)
     * 5. Read bytes 0xF6 and 0xF7 simmultaneously
     * 6. Wait until a stop condition occurs
     */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(i2c_master_start(cmd) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to start I2C communication.", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* 1. */
    if(i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_WRITE, ACK_EN) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to write sensor address.", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* 2. */
    if(i2c_master_write_byte(cmd, reg_addr, ACK_EN) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to write register address.", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* 3. */
    if(i2c_master_start(cmd) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to stop I2C communication.", __func__, __LINE__);
        return ESP_FAIL;
    }

/*
    if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to send queued commands.", __func__, __LINE__);
        return ESP_FAIL;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    if(i2c_master_start(cmd) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to start I2C communication.", __func__, __LINE__);
        return ESP_FAIL;
    }
*/

    /* 4. */
    if(i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_READ, ACK_EN) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to write sensor address.", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* 5. */
    if(len > 1) {
        if(i2c_master_read(cmd, buff, len - 1, I2C_MASTER_ACK) != ESP_OK) {
            ESP_LOGE(i2c_tag, "%s in line %d --> Failed to read <0x%X> address with a length of <%d>", __func__, __LINE__, reg_addr, len);
        }
    }

    if(i2c_master_read_byte(cmd, buff + len - 1, I2C_MASTER_NACK) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to read byte.", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* 6. */
    if(i2c_master_stop(cmd)) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to stop I2C communication.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS) != ESP_OK) {
        ESP_LOGE(i2c_tag, "%s in line %d --> Failed to send queued commands.", __func__, __LINE__);
        return ESP_FAIL;
    }
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t i2c_write_byte(uint8_t sensor_addr, uint8_t reg_addr, uint8_t data) {

    /**
     * I2C Write
     * 
     * 1. Send 111011x0 (write mode)
     * 2. Send register address (0xA0)
     * 3. First, send 8-bit data to 0xA0 register, then send again 8-bit data to 0xA1
     * 4. Wait for a stop condition
     */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    /* 1. */
    i2c_master_write_byte(cmd, (sensor_addr << 1) | I2C_MASTER_WRITE, ACK_EN);

    /* 2. */
    i2c_master_write_byte(cmd, reg_addr, ACK_EN);

    /* 3. */
    i2c_master_write_byte(cmd, data, ACK_EN);

    /* 4. */
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

void i2c_scan(sensors_addr_t * sensors, int len) {
    ESP_LOGI(i2c_tag, "Scanning I2C bus...");

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            for (int i = 0; i < len; i++) {
                if(addr == sensors[i].addr) {
                    sensors[i].__found = true;
                    ESP_LOGI(i2c_tag, "%s found at <0x%X>", sensors[i].name, sensors[i].addr);
                }
            }
        }
    }
}
