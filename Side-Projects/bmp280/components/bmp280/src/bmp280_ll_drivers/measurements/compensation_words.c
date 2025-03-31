#include <bmp280_ll_drivers/measurements/compensation_words.h>
#include <bmp280_ll_drivers/communications/com.h>
#include <registers_map/bmp280_registers.h>
#include <esp_log.h>

static const char * bmp280_tag = "BMP280";


/**
 * @brief Read MSB and LSB from compensation 16-bit word registers
 * 
 * @param slave_addr: Sensor (bmp280) address
 * @param reg_addr: Address of compensation word register
 * @param comp_word: Array to store compensation 16-bit word
 * @param _signed: 1 if compensation word is signed else 0
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
static esp_err_t bmp280_ReadCompWord(uint8_t slave_addr, uint8_t reg_addr, void * comp_word, _signed_t _signed) {
    uint8_t buff[2];

    /* Read LSB and MSB from calibration data variable */
    if(i2c_read_byte(slave_addr, reg_addr, buff, 2) != ESP_OK) {
        return ESP_FAIL;
    }

    /* dig_(T/P)1 are unsigned while the rest of them are signed */
    if(_signed) {
        *(bmp280_s16_t *) comp_word = (bmp280_s16_t) (buff[1] << 8) | (bmp280_s16_t) buff[0];    /* MSB | LSB */
    } else {
        *(bmp280_u16_t *) comp_word = (bmp280_u16_t) (buff[1] << 8) | (bmp280_u16_t) buff[0];  /* MSB | LSB */
    }

    return ESP_OK;
}


/**
 * @brief Read MSB and LSB from compensation 16-bit word registers and convert it from 2 complement into decimal
 * 
 * @param slave_addr: Sensor (bmp280) address
 * @param reg_addr: Address of compensation word register
 * @param comp_word: Array to store compensation 16-bit word
 * @param _signed: 1 if compensation word is signed else 0
 * 
 * @return ESP_OK if success - ESP_FAIL
 */
static esp_err_t bmp280_GetCompWord(uint8_t slave_addr, uint8_t reg_addr, void * comp_word, _signed_t _signed) {
    bmp280_ReadCompWord(slave_addr, reg_addr, comp_word, _signed); /* Read 16-bit compensation word */

    short unsigned int raw_word = *(short unsigned int *) comp_word;    /* Get raw word */
    printf("(raw) 0x%X --> Hex: 0x%X = %u\r\n", reg_addr, raw_word, raw_word);

    if(_signed) {
        *(bmp280_s16_t *) comp_word = *(bmp280_s16_t *) comp_word;  /* Convert to signed decimal */
    } else {
        *(bmp280_u16_t *) comp_word = *(bmp280_u16_t *) comp_word;  /* Convert to unsigned decimal */
    }

    return ESP_OK;
}


esp_err_t bmp280_InitCompWords(uint8_t slave_addr, compensation_words_t * comp_words) {
    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_T1_LSB_REG, (void *) &(comp_words->dig_T1), UNSIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_T2_LSB_REG, (void *) &(comp_words->dig_T2), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_T3_LSB_REG, (void *) &(comp_words->dig_T3), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P1_LSB_REG, (void *) &(comp_words->dig_P1), UNSIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P2_LSB_REG, (void *) &(comp_words->dig_P2), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P3_LSB_REG, (void *) &(comp_words->dig_P3), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P4_LSB_REG, (void *) &(comp_words->dig_P4), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P5_LSB_REG, (void *) &(comp_words->dig_P5), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P6_LSB_REG, (void *) &(comp_words->dig_P6), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P7_LSB_REG, (void *) &(comp_words->dig_P7), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P8_LSB_REG, (void *) &(comp_words->dig_P8), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    if(bmp280_GetCompWord(slave_addr, BMP280_DIG_P9_LSB_REG, (void *) &(comp_words->dig_P9), SIGNED) != ESP_OK) {
        ESP_LOGE(bmp280_tag, "%s in line %d --> Failed to read compensation words.", __func__, __LINE__);
        return ESP_FAIL;
    }

    return ESP_OK;
}
