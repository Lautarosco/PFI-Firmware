#include <bmp280_hal_drivers/bmp280_hal_api.h>

#include <com.h>
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

static const char * bmp280_tag = "BMP280";


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

esp_err_t bmp280_hal_Reset(i2c_master_dev_handle_t bmp280_i2c_bus_handler) {
    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_RESET_REG, 0xB6) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Reset device --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else {
        ESP_LOGI(bmp280_tag, "Reset device --> OK");
        return ESP_OK;
    }
}


esp_err_t bmp280_hal_GetChipID(bmp280_t * bmp) {
    if(bmp280_hal_ReadSerial(BMP280_ID_REG, bmp->i2c.bmp280_i2c_bus_handler, &(bmp->id), 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Get chip ID --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    } else {
        ESP_LOGI(bmp280_tag, "Get chip ID --> OK ---- ID is <0x%X>", bmp->id);
        return ESP_OK;
    }
}


esp_err_t bmp280_hal_SetPowerMode(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_PowerMode_t power_mode) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xFC;    /* Clear bits <1:0> of ctrl_meas <0xF4> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | power_mode) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    // ESP_LOGI(bmp280_tag, "Set power mode --> OK ---- power mode is %d", power_mode);

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsT(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_OsT_t temp_os) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of ctrl_meas <0xF4> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (temp_os << 5)) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &ctrl_meas, 0, sizeof(ctrl_meas));
    i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1);
    ESP_LOGI(bmp280_tag, "Set oversampling temperature --> OK ---- os. temp is '%s'", bmp280_GetOsTName((ctrl_meas & (0b111 << 5)) >> 5));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsP(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_OsP_t press_os) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of ctrl_meas <0xF4> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (press_os << 2)) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to ctrl_meas register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &ctrl_meas, 0, sizeof(ctrl_meas));
    i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1);
    ESP_LOGI(bmp280_tag, "Set oversampling pressure --> OK ---- os. press is '%s'", bmp280_GetOsPName((ctrl_meas & (0b111 << 2)) >> 2));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetTsb(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_TStandby_t t_sb) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of config <0xF5> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, (config & mask) | (t_sb << 5)) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    
    memset((void *) &config, 0, sizeof(config));
    i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1);
    ESP_LOGI(bmp280_tag, "Set standby time --> OK ---- t_sb is '%s'", bmp280_GetTsbName((config & (0b111 << 5)) >> 5));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetIIR(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_IIRCoeff_t coeff) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of config <0xF5> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, (config & mask) | (coeff << 2)) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    memset((void *) &config, 0, sizeof(config));
    i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1);
    ESP_LOGI(bmp280_tag, "Set IIR coefficient --> OK ---- IIR coeff. is '%s'", bmp280_GetIIRName((config & (0b111 << 2)) >> 2));

    return ESP_OK;
}


esp_err_t bmp280_hal_SetSerial(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_SerialInterface_t serial) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Read from config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }

    uint8_t mask = 0xFE;    /* Clear bit 0 of config <0xF4> register */

    if(i2c_write_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, (config & mask) | serial) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d: Write to config register --> FAILED", __func__, __LINE__);
        return ESP_FAIL;
    }
    
    memset((void *) &config, 0, sizeof(config));
    i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_CONFIG_REG, &config, 1);
    ESP_LOGI(bmp280_tag, "Set serial interface --> OK ---- serial interface is '%s'", bmp280_GetSerialName(config & 0b1));

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadRawTP(i2c_master_dev_handle_t bmp280_i2c_bus_handler, bmp280_s32_t * adc_t, bmp280_s32_t * adc_p) {
    uint8_t buff[6];

    uint8_t status;
    uint8_t mask = 0x08;    /* Clear all bits except bit 3 */

    /* Wait until im_update <0> bit of status register is set to 0 */
    do {
        if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_STATUS_REG, &status, 1) != ESP_OK) {
            return ESP_FAIL;
        }
    } while (mask & status);

    /* Perform a burst read from press_msb <0xF7> to temp_xlsb <0XFC> */

    if(i2c_read_bytes(bmp280_i2c_bus_handler, BMP280_PRESS_MSB_REG, buff, 6) != ESP_OK) {
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


esp_err_t bmp280_hal_i2cInit(i2c_master_bus_handle_t * i2c_master_bus_handler, i2c_master_dev_handle_t * bmp280_i2c_bus_handler,
                             uint8_t slave_addr, int sda, int scl) {
    if(i2c_init(i2c_master_bus_handler, sda, scl) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to initialized I2C interface");
        return ESP_FAIL;
    }
    if(i2c_add_device(*i2c_master_bus_handler, bmp280_i2c_bus_handler, slave_addr, I2C_ADDR_BIT_7, 100000) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to add bmp280 to I2C bus");
        return ESP_FAIL;
    }
    if(!i2c_scan(*i2c_master_bus_handler, slave_addr)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadSerial(uint8_t reg_addr, i2c_master_dev_handle_t bmp280_i2c_bus_handler, uint8_t * buff, int n_bytes) {
    if(i2c_read_bytes(bmp280_i2c_bus_handler, reg_addr, buff, n_bytes) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to read bytes");
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_WriteSerial(uint8_t reg_addr, i2c_master_dev_handle_t bmp280_i2c_bus_handler, uint8_t data) {
    if(i2c_write_bytes(bmp280_i2c_bus_handler, reg_addr, data) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "Failed to write bytes");
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadCompWords(comp_words_t * comp_words, i2c_master_dev_handle_t bmp280_i2c_bus_handler) {
    uint8_t buff[2];

    /* Loop through all compensation words */
    for(int word_index = 0; word_index < ((sizeof(comp_words_arr) / (sizeof(comp_words_arr[0])))); word_index++) {

        /* Read each compensation word register (LSB/MSB) */
        bmp280_hal_ReadSerial(comp_words_arr[word_index].addr, bmp280_i2c_bus_handler, buff, 2);

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


double bmp280_hal_GetRelativeP(bmp280_t bmp, int n) {
    double p0 = 0.0;
    for(int i = 0; i < n; i++) {
        bmp.measure(bmp.i2c.bmp280_i2c_bus_handler);
        p0 += bmp.get_press();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    p0 /= n;

    ESP_LOGI(bmp280_tag, "Average pressure: %lf", p0);

    return p0;
}
