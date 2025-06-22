#include <ll/bmp390_ll_api.h>
#include <ll/bmp390_registers.h>

#include <ll/bmp390_modes_names.h>

#include <esp_err.h>
#include <esp_log.h>
#include <stdbool.h>

#include <interface.h>

const char *bmp390_ll_tag = "[BMP390_LL]";

esp_err_t bmp390_ll_get_chip_id(device_interface_t dev_iface, uint8_t *read_data) {
    /* Check if <read_data> is a valid pointer */
    if(read_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    /* Clear <read_data> parameter for security */
    *read_data = 0;

    /* Read CHIP_ID register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_CHIP_ID_RO_REG, read_data, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmp390_ll_get_rev_id(device_interface_t dev_iface, uint8_t *read_data) {
    /* Check if <read_data> is a valid pointer */
    if(read_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Clear <read_data> parameter for security */
    *read_data = 0;

    /* Read REV_ID register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_REV_ID_RO_REG, read_data, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmp390_ll_err(device_interface_t dev_iface) {
    uint8_t err_reg = 0;

    /* Read ERR_REG register and check for errors */
    if(dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_ERR_REG_RO_REG, &err_reg, 1, dev_iface.iface_sel) != ESP_OK) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(err_reg & (1U << BMP390_ERR_REG_FATAL_ERR_BIT)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Fatal error", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    if(err_reg & (1U << BMP390_ERR_REG_CMD_ERR_BIT)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Command execution failed", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    if(err_reg & (1U << BMP390_ERR_REG_CONF_ERR_BIT)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Sensor configuration error detected", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bmp390_ll_cmd_rdy_status(device_interface_t dev_iface) {
    uint8_t status = 0;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_STATUS_RO_REG, &status, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(status & (1U << BMP390_STATUS_CMD_RDY_BIT)) {
        return ESP_OK;
    }

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t bmp390_ll_drdy_press_status(device_interface_t dev_iface) {
    uint8_t status = 0;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_STATUS_RO_REG, &status, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(status & (1U << BMP390_STATUS_DRDY_PRESS_BIT)) {
        return ESP_OK;
    }

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t bmp390_ll_drdy_temp_status(device_interface_t dev_iface) {
    uint8_t status = 0;

    /* Read STATUS register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_STATUS_RO_REG, &status, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    if(status & (1U << BMP390_STATUS_DRDY_TEMP_BIT)) {
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FINISHED;
}

int bmp390_ll_detect_soft_reset(device_interface_t dev_iface) {
    uint8_t event = 0;

    /* Read EVENT register and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_EVENT_RO_REG, &event, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(event & (1U << BMP390_EVENT_POR_DETECTED_BIT)) {
        return true;
    }

    return false;
}

esp_err_t bmp390_ll_spi_en(device_interface_t dev_iface, bmp390_if_conf_reg_spi_t spi_mode) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* SPI interface was selected */
    if(dev_iface.iface_sel == SPI) {
        uint8_t if_conf = 0;

        /* Read IF_CONF register content and check for errors */
        if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, &if_conf, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }

        /* Clear bit in position <BMP390_IF_CONF_SPI3_BIT> of <if_conf>. Then, set <spi_mode> bit as its new value */
        uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_SPI3_BIT)) | (spi_mode << BMP390_IF_CONF_SPI3_BIT);
        
        /* Write new register value to IF_CONF register and check for errors */
        if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, new_if_conf, dev_iface.iface_sel)) || (bmp390_ll_err(dev_iface))) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set SPI mode --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }

        ESP_LOGI(bmp390_ll_tag, "Set SPI mode <%s> --> OK", bmp390_get_spi_mode_name(spi_mode));
        return ESP_OK;
    }

    /* SPI interface was not selected */
    ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: SPI interface must be selected in order to use this function", __func__, __LINE__);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t bmp390_ll_i2c_en_wdt(device_interface_t dev_iface, bmp390_if_conf_reg_i2c_wdt_tout_t i2c_wdt_tout) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* I2C interface was selected */
    if(dev_iface.iface_sel == I2C) {
        uint8_t if_conf = 0;

        /* Read IF_CONF register content and check for errors */
        if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, &if_conf, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }
        
        /* Clear i2c_wdt_en and i2c_wdt_sel bits, then enable I2C wdt timeout and define its period <i2c_wdt_tout> */
        uint8_t new_if_conf = (if_conf & ~((1U << BMP390_IF_CONF_I2C_WDT_EN_BIT) | (1U << BMP390_IF_CONF_I2C_WDT_SEL_BIT))) | ((1U << BMP390_IF_CONF_I2C_WDT_EN_BIT) | (i2c_wdt_tout << BMP390_IF_CONF_I2C_WDT_SEL_BIT));
        
        /* Write IF_CONF register and check for errors */
        if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, new_if_conf, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Enable and configure I2C watchdog timeout --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }

        ESP_LOGI(bmp390_ll_tag, "Enable and configure I2C watchdog timeout <%s> --> OK", bmp390_get_i2c_wdt_tout_name(i2c_wdt_tout));
        return ESP_OK;
    }

    /* I2C interface was not selected */
    ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: I2C interface must be selected in order to use this function", __func__, __LINE__);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t bmp390_ll_i2c_dis_wdt(device_interface_t dev_iface) {
    /* Check if any interface was selected */
    if(dev_iface.iface_sel == NONE) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: No Interface was selected", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* I2C interface was selected */
    if(dev_iface.iface_sel == I2C) {

        /* Read IF_CONF register content and check for errors */
        uint8_t if_conf = 0;
        if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, &if_conf, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }
        
        /* Clear <BMP390_IF_CONF_I2C_WDT_EN_BIT> bit of IF_CONF register and disable watchdog timeout */
        uint8_t new_if_conf = (if_conf & ~(1U << BMP390_IF_CONF_I2C_WDT_EN_BIT)) | (BMP390_IF_CONF_I2C_WDT_DIS << BMP390_IF_CONF_I2C_WDT_EN_BIT);
        
        /* Write IF_CONF register and check for errors */
        if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_IF_CONF_RW_REG, new_if_conf, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
            ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Disable I2C watchdog timeout --> FAILED", __func__, __LINE__);
            return ESP_FAIL;
        }

        ESP_LOGI(bmp390_ll_tag, "Disable I2C watchdog timeout --> OK");
        return ESP_OK;
    }
    
    ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: SPI interface must be selected in order to use this function", __func__, __LINE__);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t bmp390_ll_set_pwr_mode(device_interface_t dev_iface, bmp390_pwr_ctrl_mode_t pwr_mode) {
    uint8_t pwr_ctrl = 0;

    /* Read PWR_CTRL register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, &pwr_ctrl, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits 4 and 5 of PWR_CTRL register, then set the power mode */
    uint8_t mask = (pwr_ctrl & ~(0b11 << BMP390_PWR_CTRL_MODE_BITS)) | (pwr_mode << BMP390_PWR_CTRL_MODE_BITS);

    /* Write mask to PWR_CTRL register and check for errors */
    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set power mode --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Set power mode <%s> --> OK", bmp390_get_pwr_mode_name(pwr_mode));
    return ESP_OK; 
}

esp_err_t bmp390_ll_press_en(device_interface_t dev_iface) {
    uint8_t pwr_ctrl = 0;

    /* Read PWR_CTRL register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, &pwr_ctrl, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits 4 and 5 of PWR_CTRL register, then enable pressure sensor */
    uint8_t mask = (pwr_ctrl & ~(1U << BMP390_PWR_CTRL_PRESS_EN_BIT)) | (BMP390_PWR_CTRL_PRESS_EN << BMP390_PWR_CTRL_PRESS_EN_BIT);

    /* Write mask to PWR_CTRL register and check for errors */
    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Enable pressure sensor --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Enable pressure sensor --> OK");
    return ESP_OK;
}

esp_err_t bmp390_ll_temp_en(device_interface_t dev_iface) {
    uint8_t pwr_ctrl = 0;

    /* Read PWR_CTRL register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, &pwr_ctrl, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits[5:4] of PWR_CTRL register, then enable temperature sensor */
    uint8_t mask = (pwr_ctrl & ~(1U << BMP390_PWR_CTRL_TEMP_EN_BIT)) | (BMP390_PWR_CTRL_TEMP_EN << BMP390_PWR_CTRL_TEMP_EN_BIT);

    /* Write mask to PWR_CTRL register and check for errors */
    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_PWR_CTRL_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Enable temperature sensor --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Enable temperature sensor --> OK");
    return ESP_OK;
}

esp_err_t bmp390_ll_set_osr_press(device_interface_t dev_iface, bmp390_osr_press_t osr_press) {
    if((osr_press < BMP390_OSR_P_X1) || (osr_press > BMP390_OSR_P_X32)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Invalid <osr_press> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t osr = 0;

    /* Read OSR register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_OSR_RW_REG, &osr, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits[2:0] of OSR register and set oversampling rate for pressure measurements */
    uint8_t mask = (osr & ~(0b111 << BMP390_OSR_P_BITS)) | (osr_press << BMP390_OSR_P_BITS);

    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_OSR_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set pressure oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Set pressure oversampling rate <%s> --> OK", bmp390_get_osr_press_name(osr_press));
    return ESP_OK;
}

esp_err_t bmp390_ll_set_osr_temp(device_interface_t dev_iface, bmp390_osr_temp_t osr_temp) {
    if((osr_temp < BMP390_OSR_T_X1) || (osr_temp > BMP390_OSR_T_X32)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Invalid <osr_temp> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t osr = 0;

    /* Read OSR register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_OSR_RW_REG, &osr, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits[2:0] of OSR register and set oversampling rate for temperature measurements */
    uint8_t mask = (osr & ~(0b111 << BMP390_OSR_T_BITS)) | (osr_temp << BMP390_OSR_T_BITS);

    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_OSR_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set temperature oversampling rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Set temperature oversampling rate <%s> --> OK", bmp390_get_osr_temp_name(osr_temp));
    return ESP_OK;
}

esp_err_t bmp390_ll_set_odr(device_interface_t dev_iface, bmp390_odr_sel_t odr_sel) {
    if((odr_sel < BMP390_ODR_SEL_200_HZ) || (odr_sel > BMP390_ODR_SEL_0P0015_HZ)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Invalid <osr_temp> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t odr = 0;

    /* Read ODR register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_ODR_RW_REG, &odr, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits[4:0] of ODR register and set output data rates */
    uint8_t mask = (odr & ~(0b1111 << BMP390_ODR_ODR_SEL_BITS)) | (odr_sel << BMP390_ODR_ODR_SEL_BITS);

    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_ODR_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set output data rate --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Set output data rate <%s> --> OK", bmp390_get_odr_sel_name(odr_sel));
    return ESP_OK;
}

esp_err_t bmp390_ll_set_iir_coef(device_interface_t dev_iface, bmp390_config_coef_t iir_coef) {
    if((iir_coef < BMP390_CONFIG_COEF_0) || (iir_coef > BMP390_CONFIG_COEF_127)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Invalid <iir_coef> parameter", __func__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t config = 0;

    /* Read CONFIG register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_CONFIG_RW_REG, &config, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear bits[3:1] of CONFIG register and set IIR filter coefficient */
    uint8_t mask = (config & ~(0b111 << BMP390_CONFIG_IIR_BITS)) | (iir_coef << BMP390_CONFIG_IIR_BITS);

    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_CONFIG_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set IIR filter coefficient --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Set IIR filter coefficient <%s> --> OK", bmp390_get_iir_coef_name(iir_coef));
    return ESP_OK;
}

esp_err_t bmp390_ll_exec_cmd(device_interface_t dev_iface, bmp390_cmd_t cmd_sel) {
    uint8_t cmd = 0;

    /* Read CMD register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_CMD_RW_REG, &cmd, 1, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    /* Clear all bits[7:0] of CONFIG register and set cmd to be executed */
    uint8_t mask = (cmd & ~0b11111111) | cmd_sel;

    if((dev_iface.write_bytes(dev_iface.iface_cfg, BMP390_CMD_RW_REG, mask, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        ESP_LOGE(bmp390_ll_tag, "{Function <%s> in line %d}: Set CMD --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp390_ll_tag, "Execute CMD <%s> --> OK", bmp390_get_cmd_name(cmd_sel));
    return ESP_OK;
}

esp_err_t bmp390_ll_read_raw_data(device_interface_t dev_iface, uint32_t *adc_temp, uint32_t *adc_press) {
    uint8_t data[6] = {0};

    /* Check if adc_temp and adc_press are valid pointers */
    if((adc_temp == NULL) || (adc_press == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Clear both pointers for security */
    *adc_temp = 0;
    *adc_press = 0;

    /* Read DATA register content and check for errors */
    if((dev_iface.read_bytes(dev_iface.iface_cfg, BMP390_DATA_0_RO_REG, data, 6, dev_iface.iface_sel) != ESP_OK) || (bmp390_ll_err(dev_iface) != ESP_OK)) {
        return ESP_FAIL;
    }
    
    /* Raw temperature = MSB[23:16] | LSB[15:8] | XLSB[7:0] */
    *adc_temp = ((uint32_t) data[5] << 16) | ((uint32_t) data[4] << 8) | data[3];

    /* Raw pressure = MSB[23:16] | LSB[15:8] | XLSB[7:0] */
    *adc_press = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | data[0];

    return ESP_OK;
}
