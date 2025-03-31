#include <bmp280_hal_drivers/bmp280_hal_api.h>
#include <registers_map/bmp280_registers.h>
#include <bmp280_ll_drivers/communications/com.h>

#include <bmp280_structs.h>

#include <esp_err.h>
#include <string.h>


esp_err_t bmp280_hal_Reset(bmp280_t bmp) {
    if(i2c_write_byte(bmp.addr, BMP280_RESET_REG, 0xB6) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetPowerMode(bmp280_t bmp, bmp280_PowerMode_t power_mode) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_byte(bmp.addr, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0xFC;    /* Clear bits <1:0> of ctrl_meas <0xF4> register */

    if(i2c_write_byte(bmp.addr, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | power_mode) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsT(bmp280_t bmp, bmp280_OsT_t temp_os ) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_byte(bmp.addr, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of ctrl_meas <0xF4> register */

    if(i2c_write_byte(bmp.addr, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (temp_os << 5)) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetOsP(bmp280_t bmp, bmp280_OsP_t press_os ) {
    /* Read current value of ctrl_meas register */
    uint8_t ctrl_meas;
    if(i2c_read_byte(bmp.addr, BMP280_CTRL_MEAS_REG, &ctrl_meas, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of ctrl_meas <0xF4> register */

    if(i2c_write_byte(bmp.addr, BMP280_CTRL_MEAS_REG, (ctrl_meas & mask) | (press_os << 2)) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetTsb(bmp280_t bmp, bmp280_TStandby_t t_sb ) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_byte(bmp.addr, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0x1F;    /* Clear bits <7:5> of config <0xF5> register */

    if(i2c_write_byte(bmp.addr, BMP280_CONFIG_REG, (config & mask) | (t_sb << 5)) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetIIR(bmp280_t bmp, bmp280_IIRCoeff_t coeff ) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_byte(bmp.addr, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0xE3;    /* Clear bits <4:2> of config <0xF5> register */

    if(i2c_write_byte(bmp.addr, BMP280_CONFIG_REG, (config & mask) | (coeff << 2)) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_SetSerial(bmp280_t bmp, bmp280_SerialInterface_t serial ) {
    /* Read current value of config register */
    uint8_t config;
    if(i2c_read_byte(bmp.addr, BMP280_CONFIG_REG, &config, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t mask = 0xFE;    /* Clear bit 0 of config <0xF4> register */

    if(i2c_write_byte(bmp.addr, BMP280_CONFIG_REG, (config & mask) | serial) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t bmp280_hal_ReadRawTP(bmp280_t bmp, bmp280_s32_t * adc_t) {
    uint8_t buff[3];

    uint8_t status;
    uint8_t mask = 0x08;    /* Clear all bits except bit 3 */

    /* Wait until im_update<0> bit of status register is set to 0 */
    do {
        if(i2c_read_byte(bmp.addr, BMP280_STATUS_REG, &status, 1) != ESP_OK) {
            return ESP_FAIL;
        }
    } while (mask & status);

    if(i2c_read_byte(bmp.addr, BMP280_TEMP_MSB_REG, buff, 3) != ESP_OK) {
        return ESP_FAIL;
    }
    printf("<0x%X>: 0x%X\r\n", BMP280_TEMP_MSB_REG, buff[0]);
    printf("<0x%X>: 0x%X\r\n", BMP280_TEMP_LSB_REG, buff[1]);
    printf("<0x%X>: 0x%X\r\n\n", BMP280_TEMP_XLSB_REG, buff[2]);

/*
    memset(buff, 0, sizeof(buff));
    if(i2c_read_byte(bmp.addr, BMP280_TEMP_LSB_REG, buff, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    printf("<0x%X>: 0x%X\r\n", BMP280_TEMP_LSB_REG, buff[0]);
*/
    // *adc_t = (buff[3] << 12) | (buff[4] << 4) | (buff[5] >> 4);

    return ESP_OK;
}
