#include <bmp390_hal_api.h>
#include <bmp390_registers.h>

#include <esp_err.h>
#include <esp_log.h>
#include <stdbool.h>

#include <interface.h>

const char *bmp390_hal_tag = "[BMP390_HAL]";

esp_err_t bmp390_hal_get_chip_id(device_interface_t dev_iface, uint8_t *read_data) {
    /* Read CHIP_ID register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_CHIP_ID_RO_REG, read_data, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        return ESP_OK;
    }
}

esp_err_t bmp390_hal_get_rev_id(device_interface_t dev_iface, uint8_t *read_data) {
    /* Read REV_ID register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_REV_ID_RO_REG, read_data, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        return ESP_OK;
    }
}

esp_err_t bmp390_hal_err(device_interface_t dev_iface) {
    uint8_t err_reg;

    /* Read ERR_REG register and check for errors */
    if(dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_ERR_REG_RO_REG, &err_reg, 1) != ESP_OK) {
        return ESP_FAIL;
    } else {
        if(err_reg & (1U << BMP390_ERR_REG_FATAL_ERR_BIT)) {
            ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: Fatal error", __func__, __LINE__);
            return ESP_FAIL;
        } else if(err_reg & (1U << BMP390_ERR_REG_CMD_ERR_BIT)) {
            ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: Command execution failed", __func__, __LINE__);
            return ESP_FAIL;
        } else if(err_reg & (1U << BMP390_ERR_REG_CONF_ERR_BIT)) {
            ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: Sensor configuration error detected", __func__, __LINE__);
            return ESP_FAIL;
        } else {
            return ESP_OK;
        }
    }
}

esp_err_t bmp390_hal_cmd_rdy_status(device_interface_t dev_iface) {
    uint8_t cmd_status;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_STATUS_RO_REG, &cmd_status, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        if(cmd_status & (1U << BMP390_STATUS_CMD_RDY_BIT)) {
            return ESP_OK;
        } else {
            return ESP_ERR_NOT_FINISHED;
        }
    }
}

esp_err_t bmp390_hal_drdy_press_status(device_interface_t dev_iface) {
    uint8_t press_status;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_STATUS_RO_REG, &press_status, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        if(press_status & (1U << BMP390_STATUS_DRDY_PRESS_BIT)) {
            return ESP_OK;
        } else {
            return ESP_ERR_NOT_FINISHED;
        }
    }
}

esp_err_t bmp390_hal_drdy_temp_status(device_interface_t dev_iface) {
    uint8_t temp_status;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_STATUS_RO_REG, &temp_status, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        if(temp_status & (1U << BMP390_STATUS_DRDY_TEMP_BIT)) {
            return ESP_OK;
        } else {
            return ESP_ERR_NOT_FINISHED;
        }
    }
}

int bmp390_hal_detect_soft_reset(device_interface_t dev_iface) {
    uint8_t event;

    /* Read EVENT register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_EVENT_RO_REG, &event, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    } else {
        if(event & (1U << BMP390_EVENT_POR_DETECTED_BIT)) {
            return true;
        } else {
            return false;
        }
    }
}

esp_err_t bmp390_hal_spi_en(device_interface_t dev_iface, bmp390_if_conf_reg_spi_t spi_mode) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else if(dev_iface.iface_sel == SPI) {
        /* SPI interface was selected */
        
        /* Read IF_CONF register content and check for errors */
        uint8_t if_conf;
        if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &if_conf, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
            return ESP_FAIL;
        } else {
            /* Read operation success */

            /* Clear bit in position <BMP390_IF_CONF_SPI3_BIT> of <if_conf>. Then, set <spi_mode> bit as its new value */
            uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_SPI3_BIT)) | (spi_mode << BMP390_IF_CONF_SPI3_BIT);

            /* Write new register value to IF_CONF register and check for errors */
            if((dev_iface.write_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &new_if_conf, 1)) || (bmp390_hal_err(dev_iface))) {
                return ESP_FAIL;
            } else {
                /* Write operation success */

                ESP_LOGI(bmp390_hal_tag, "Set SPI mode --> OK");
                return ESP_OK;
            }
        }
    } else {
        /* SPI interface was not selected */
        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: SPI interface must be selected in order to use this function", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t bmp390_hal_i2c_en_wdt(device_interface_t dev_iface, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else if(dev_iface.iface_sel == I2C) {
        /* I2C interface was selected */

        /* Read IF_CONF register content and check for errors */
        uint8_t if_conf;
        if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &if_conf, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
            return ESP_FAIL;
        } else {
            /* Read operation success */

            /* Clear i2c_wdt_en and i2c_wdt_sel bits, then enable I2C wdt timeout and define its period <i2c_wdt_tout> */
            uint8_t new_if_conf = (if_conf & ~((1U << BMP390_IF_CONF_I2C_WDT_EN_BIT) | (1U << BMP390_IF_CONF_I2C_WDT_SEL_BIT))) | ((1U << BMP390_IF_CONF_I2C_WDT_EN_BIT) | (i2c_wdt_tout << BMP390_IF_CONF_I2C_WDT_SEL_BIT));

            /* Write IF_CONF register and check for errors */
            if((dev_iface.write_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &new_if_conf, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
                return ESP_FAIL;
            } else {
                /* Write operation success */

                ESP_LOGI(bmp390_hal_tag, "Enable and configure I2C watchdog timeout --> OK");
                return ESP_OK;
            }
        }
    } else {
        /* I2C interface was not selected */

        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: I2C interface must be selected in order to use this function", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t bmp390_hal_i2c_dis_wdt(device_interface_t dev_iface) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    } else if(dev_iface.iface_sel == I2C) {
        /* I2C interface was selected */

        /* Read IF_CONF register content and check for errors */
        uint8_t if_conf;
        if((dev_iface.read_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &if_conf, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
            return ESP_FAIL;
        } else {
            /* Read operation success */

            /* Clear <BMP390_IF_CONF_I2C_WDT_EN_BIT> bit of IF_CONF register and disable watchdog timeout */
            uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_I2C_WDT_EN_BIT)) | (BMP390_IF_CONF_I2C_WDT_DIS << BMP390_IF_CONF_I2C_WDT_EN_BIT);

            /* Write IF_CONF register and check for errors */
            if((dev_iface.write_bytes(dev_iface.iface_cfg, dev_iface.iface_sel, BMP390_IF_CONF_RW_REG, &new_if_conf, 1) != ESP_OK) || (bmp390_hal_err(dev_iface) != ESP_OK)) {
                return ESP_FAIL;
            } else {
                /* Write operation success */

                ESP_LOGI(bmp390_hal_tag, "Enable and configure I2C watchdog timeout --> OK");
                return ESP_OK;
            }
        }
    } else {
        /* I2C interface was not selected */

        ESP_LOGE(bmp390_hal_tag, "{Function <%s> in line %d}: SPI interface must be selected in order to use this function", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
}
