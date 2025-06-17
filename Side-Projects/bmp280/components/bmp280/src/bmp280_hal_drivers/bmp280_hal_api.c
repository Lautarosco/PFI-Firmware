#include <bmp280_hal_drivers/bmp280_hal_api.h>

#include <bmp280_registers.h>
#include <bmp280_structs.h>
#include <bmp280_data_types.h>
#include <get_modes_names.h>

#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/* #################### CONSTANTS #################### */

static const char * bmp280_hal_tag = "[BMP280_HAL]";


/* #################### STRUCTS #################### */

typedef struct compensation_words {
    union {
        bmp280_u16_t _unsigned;
        bmp280_s16_t _signed;    
    };
    uint8_t addr;
} compensation_words_t;

static compensation_words_t comp_words_arr[12] = {
    {.addr = BMP280_DIG_T1_LSB_REG},
    {.addr = BMP280_DIG_T2_LSB_REG},
    {.addr = BMP280_DIG_T3_LSB_REG},
    {.addr = BMP280_DIG_P1_LSB_REG},
    {.addr = BMP280_DIG_P2_LSB_REG},
    {.addr = BMP280_DIG_P3_LSB_REG},
    {.addr = BMP280_DIG_P4_LSB_REG},
    {.addr = BMP280_DIG_P5_LSB_REG},
    {.addr = BMP280_DIG_P6_LSB_REG},
    {.addr = BMP280_DIG_P7_LSB_REG},
    {.addr = BMP280_DIG_P8_LSB_REG},
    {.addr = BMP280_DIG_P9_LSB_REG}
};


/* #################### DEFINITIONS #################### */

esp_err_t bmp280_hal_Reset(dev_serial_iface_t *dev_iface) {
    const uint8_t reset_cmd = 0xB6;

    esp_err_t ret = dev_iface->write_func(dev_iface->handler, BMP280_RESET_REG, reset_cmd, sizeof(reset_cmd), IFACE_I2C);

    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Reset device --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else {
        ESP_LOGI(bmp280_hal_tag, "Reset device --> OK");
        return ESP_OK;
    }
}


esp_err_t bmp280_hal_GetChipID(dev_serial_iface_t *dev_iface, uint8_t *buff) {
    if(bmp280_hal_ReadSerial(dev_iface, BMP280_ID_REG, buff, 1) != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Get chip ID --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else {
        ESP_LOGI(bmp280_hal_tag, "Get chip ID --> OK ---- ID is <0x%X>", *buff);
        return ESP_OK;
    }
}


esp_err_t bmp280_hal_SetPowerMode(dev_serial_iface_t *dev_iface, bmp280_PowerMode_t power_mode) {
    uint8_t ctrl_meas;

    /* Read current value of ctrl_meas register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xFC;    /* Clear bits <1:0> of ctrl_meas <0xF4> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | power_mode, sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    ESP_LOGI(bmp280_hal_tag, "Set power mode --> OK ---- power mode is %d", power_mode);

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsT(dev_serial_iface_t *dev_iface, bmp280_OsT_t temp_os) {
    uint8_t ctrl_meas;
    
    /* Read current value of ctrl_meas register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of ctrl_meas <0xF4> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (temp_os << 5), sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &ctrl_meas, 0, sizeof(ctrl_meas));
    ret = dev_iface->read_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ESP_LOGI(bmp280_hal_tag, "Set oversampling temperature --> OK ---- os. temp is '%s'", bmp280_GetOsTName((ctrl_meas & (0b111 << 5)) >> 5));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsP(dev_serial_iface_t *dev_iface, bmp280_OsP_t press_os) {
    uint8_t ctrl_meas;

    /* Read current value of ctrl_meas register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of ctrl_meas <0xF4> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (press_os << 2), sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &ctrl_meas, 0, sizeof(ctrl_meas));
    ret = dev_iface->read_func(dev_iface->handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ESP_LOGI(bmp280_hal_tag, "Set oversampling pressure --> OK ---- os. press is '%s'", bmp280_GetOsPName((ctrl_meas & (0b111 << 2)) >> 2));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetTsb(dev_serial_iface_t *dev_iface, bmp280_TStandby_t t_sb) {
    uint8_t config;

    /* Read current value of config register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of config <0xF5> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CONFIG_REG, (config & mask) | (t_sb << 5), sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    memset((void *) &config, 0, sizeof(config));
    ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ESP_LOGI(bmp280_hal_tag, "Set standby time --> OK ---- t_sb is '%s'", bmp280_GetTsbName((config & (0b111 << 5)) >> 5));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetIIR(dev_serial_iface_t *dev_iface, bmp280_IIRCoeff_t coeff) {
    uint8_t config;

    /* Read current value of config register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of config <0xF5> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CONFIG_REG, (config & mask) | (coeff << 2), sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &config, 0, sizeof(config));
    ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ESP_LOGI(bmp280_hal_tag, "Set IIR coefficient --> OK ---- IIR coeff. is '%s'", bmp280_GetIIRName((config & (0b111 << 2)) >> 2));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetSerial(dev_serial_iface_t *dev_iface, bmp280_SerialInterface_t serial) {
    /* Read current value of config register */
    uint8_t config;

    /* Read current value of config register */
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xFE;    /* Clear bit 0 of config <0xF4> register */

    ret = dev_iface->write_func(dev_iface->handler, BMP280_CONFIG_REG, (config & mask) | serial, sizeof(mask), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    memset((void *) &config, 0, sizeof(config));
    ret = dev_iface->read_func(dev_iface->handler, BMP280_CONFIG_REG, &config, 1, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    ESP_LOGI(bmp280_hal_tag, "Set serial interface --> OK ---- serial interface is '%s'", bmp280_GetSerialName(config & 0b1));

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadRawTP(dev_serial_iface_t *dev_iface, bmp280_s32_t * adc_t, bmp280_s32_t * adc_p) {
    esp_err_t ret;
    uint8_t buff[6];

    uint8_t status;
    uint8_t mask = 0x08;    /* Clear all bits except bit 3 */

    /* Wait until im_update <0> bit of status register is set to 0 */
    do {
        ret = dev_iface->read_func(dev_iface->handler, BMP280_STATUS_REG, &status, 1, IFACE_I2C);
        if(ret != ESP_OK) {
            return ESP_FAIL;
        }
    } while (mask & status);

    /* Perform a burst read from press_msb <0xF7> to temp_xlsb <0XFC> */

    ret = dev_iface->read_func(dev_iface->handler, BMP280_PRESS_MSB_REG, buff, 6, IFACE_I2C);
    if(ret != ESP_OK) {
        return ESP_FAIL;
    }

    /* buff[6] = {press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb} */


/*
    printf("<0x%X>: 0x%X\r\n", BMP280_TEMP_MSB_REG, buff[0]);
    printf("<0x%X>: 0x%X\r\n", BMP280_TEMP_LSB_REG, buff[1]);
    printf("<0x%X>: 0x%X\r\n\n", BMP280_TEMP_XLSB_REG, buff[2]);
*/


    /* 20-bit temperature and pressure */

    *adc_p = (bmp280_u32_t) ((buff[0] << 12) | (buff[1] << 4) | (buff[2] >> 4));
    *adc_t = (bmp280_u32_t) ((buff[3] << 12) | (buff[4] << 4) | (buff[5] >> 4));

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadSerial(dev_serial_iface_t *dev_iface, uint8_t reg_addr, uint8_t * buff, int n_bytes) {
    esp_err_t ret = dev_iface->read_func(dev_iface->handler, reg_addr, buff, n_bytes, IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Read bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_WriteSerial(dev_serial_iface_t *dev_iface, uint8_t reg_addr, uint8_t data) {
    esp_err_t ret = dev_iface->write_func(dev_iface->handler, reg_addr, data, sizeof(data), IFACE_I2C);
    if(ret != ESP_OK) {
        ESP_LOGE(bmp280_hal_tag, "%s in line %d: Write bytes --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadCompWords(dev_serial_iface_t *dev_iface, comp_words_t * comp_words) {
    uint8_t buff[2];

    /* Loop through all compensation words */
    for(int word_index = 0; word_index < ((sizeof(comp_words_arr) / (sizeof(comp_words_arr[0])))); word_index++) {

        /* Read each compensation word register (LSB/MSB) */
        bmp280_hal_ReadSerial(dev_iface, comp_words_arr[word_index].addr, buff, 2);

        /* dig_T1 and dig_P1 are unsigned while the rest are signed */
        if((!word_index) || (word_index == 3)) {
            comp_words_arr[word_index]._unsigned = (bmp280_u16_t) (buff[1] << 8) | (bmp280_u16_t) buff[0];
        } else {
            comp_words_arr[word_index]._signed = (bmp280_s16_t) (buff[1] << 8) | (bmp280_s16_t) buff[0];
        }
    }

    comp_words->dig_T1 = comp_words_arr[0]._unsigned;
    comp_words->dig_T2 = comp_words_arr[1]._signed;
    comp_words->dig_T3 = comp_words_arr[2]._signed;
    comp_words->dig_P1 = comp_words_arr[3]._unsigned;
    comp_words->dig_P2 = comp_words_arr[4]._signed;
    comp_words->dig_P3 = comp_words_arr[5]._signed;
    comp_words->dig_P4 = comp_words_arr[6]._signed;
    comp_words->dig_P5 = comp_words_arr[7]._signed;
    comp_words->dig_P6 = comp_words_arr[8]._signed;
    comp_words->dig_P7 = comp_words_arr[9]._signed;
    comp_words->dig_P8 = comp_words_arr[10]._signed;
    comp_words->dig_P9 = comp_words_arr[11]._signed;

    return ESP_OK;
}
