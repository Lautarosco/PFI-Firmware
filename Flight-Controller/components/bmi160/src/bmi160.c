#include <stdint.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "communication.h"
#include "registers.h"
#include "bmi160.h"

#include "string.h"

static const char *TAG = "BMI160";

// Device Methods
/*Takes an empty bmi160_t struct and inits its functions and parameters*/
esp_err_t Bmi160( bmi160_t* bmi, int i2c_address_param, int i2c_scl_param, int i2c_sda_param ) {

    // Function pointers assignment
    bmi->init                 = bmi_init;
    bmi->measure              = bmi160_measure;
    bmi->foc                  = bmi160_foc;
    bmi->Acc.get_sensitivity  = acc_get_sensitivity;
    bmi->Acc.set_range        = acc_set_range;
    bmi->Gyro.get_sensitivity = gyro_get_sensitivity;
    bmi->Gyro.set_range       = gyro_set_range;
    bmi->Gyro.calibrate       = gyro_calibrate;

    esp_err_t i2c_ret = i2c_init(i2c_sda_param, i2c_scl_param);

    if (i2c_ret == ESP_OK){
        bmi->i2c.address = i2c_address_param;
        bmi->i2c.scl = i2c_scl_param;
        bmi->i2c.sda = i2c_sda_param;
    } else {
        ESP_LOGI(TAG, "INIT->I2C ERROR: %d", i2c_ret);
    }

    bmi->Acc.offset.x = 0.0f;
    bmi->Acc.offset.y = 0.0f;
    bmi->Acc.offset.z = 0.0f;

    bmi->Gyro.offset.x = 0.0f;
    bmi->Gyro.offset.y = 0.0f;
    bmi->Gyro.offset.z = 0.0f;

    bmi->Acc.i2c = bmi->i2c;
    bmi->Gyro.i2c = bmi->i2c;

    return ESP_OK;
}

esp_err_t bmi_init( bmi160_t * self,
    int acc_mode,  int acc_freq,  int acc_range,
    int gyro_mode, int gyro_freq, int gyro_range,
    int gyro_offset_x, int gyro_offset_y, int gyro_offset_z ) {

    /* Initialize Acelerometer */
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_CMD_REG,   acc_mode ) );       
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_ACC_CONF,  acc_freq ) );
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_ACC_RANGE, acc_range ) );
    
    /* Initialize Gyroscope */
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_CMD_REG,   gyro_mode ) );      
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_ACC_CONF,  gyro_freq ) );
    ESP_ERROR_CHECK( bmi160_write_byte( self->i2c.address, BMI160_ACC_RANGE, gyro_range ) );

    /* Set initials offsets*/
    self->Gyro.offset.x = gyro_offset_x;
    self->Gyro.offset.y = gyro_offset_y;
    self->Gyro.offset.z = gyro_offset_z;

    return ESP_OK;
}


esp_err_t bmi160_measure(bmi160_t* bmi) {

    uint8_t sensor_data[12];    // Buffer for accelerometer and gyroscope
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    // Read accelerometer, gyroscope, and magnetometer data
    bmi160_read_bytes(bmi->i2c.address, BMI160_GYRO_REGISTER, sensor_data, 12);

    // Parse and calibrate data from sensors

    gyro_x = (int16_t)((sensor_data[1] << 8) | sensor_data[0]) - bmi->Gyro.offset.x;
    gyro_y = (int16_t)((sensor_data[3] << 8) | sensor_data[2]) - bmi->Gyro.offset.y;
    gyro_z = (int16_t)((sensor_data[5] << 8) | sensor_data[4]) - bmi->Gyro.offset.z;

    accel_x = (int16_t)((sensor_data[7] << 8) | sensor_data[6]) - bmi->Acc.offset.x;
    accel_y = (int16_t)((sensor_data[9] << 8) | sensor_data[8]) - bmi->Acc.offset.y;
    accel_z = (int16_t)((sensor_data[11] << 8) | sensor_data[10]) - bmi->Acc.offset.z;


    // Convert raw data to logical values with units
    float acc_sensitivity = (bmi->Acc.get_sensitivity) (bmi->Acc);
    float gyro_sensitivity = (bmi->Gyro.get_sensitivity) (bmi->Gyro);

    float accel_x_g = accel_x * acc_sensitivity;
    float accel_y_g = accel_y * acc_sensitivity;
    float accel_z_g = accel_z * acc_sensitivity;

    float gyro_x_dps = gyro_x * gyro_sensitivity;
    float gyro_y_dps = gyro_y * gyro_sensitivity;
    float gyro_z_dps = gyro_z * gyro_sensitivity;

    // Accelerometer
    bmi->Acc.x = accel_x_g;
    bmi->Acc.y = accel_y_g;
    bmi->Acc.z = accel_z_g;

    // Gyroscopoe
    bmi->Gyro.x = gyro_x_dps;
    bmi->Gyro.y = gyro_y_dps;
    bmi->Gyro.z = gyro_z_dps;

    return ESP_OK;

}


// Sensor Methods
esp_err_t acc_set_range(acc_t* acc, uint8_t fs_sel) {
    if (fs_sel > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = bmi160_write_byte(acc->i2c.address, BMI160_ACC_RANGE, fs_sel & 0b00001111);
    if (ret != ESP_OK) {
        printf("Failed to set_accel_range: %s\n", esp_err_to_name(ret));
    }
    return ret;
}


void print_binary(unsigned int num);

float acc_get_sensitivity(acc_t acc) {
    uint8_t accel_sensitivity_setting;
    bmi160_read_bytes(acc.i2c.address, BMI160_ACC_RANGE, &accel_sensitivity_setting, 1);
/*
    uint8_t txBuff[1024];
    sprintf((char *)txBuff, "acc_sens: %d\n", accel_sensitivity_setting);
    uart_write_bytes(0, (char *)txBuff, strlen((char *)txBuff));
*/

    switch (accel_sensitivity_setting & 0b1111) {
        case 0b0011: return 2.0 / 32768.0;
        case 0b0101: return 4.0 / 32768.0;
        case 0b1000: return 8.0 / 32768.0;
        case 0b1100: return 16.0 / 32768.0;
        default:   return 2.0 / 32768.0;  // Default sensitivity
    }


};


void print_binary(unsigned int num) {
    // Size of an unsigned int in bits
    int bits = sizeof(num) * 8;
    int leading_zero = 1; // Flag to skip leading zeros

    // Iterate through each bit from the most significant to the least significant
    for (int i = bits - 1; i >= 0; i--) {
        // Get the bit at position i
        int bit = (num >> i) & 1;

        // Print the bit only if it is 1 or if we have already printed a 1 (to avoid leading zeros)
        if (bit) {
            leading_zero = 0; // We have printed at least one 1
            printf("%d", bit);
        } else if (!leading_zero) {
            // If we have started printing 1s, we can print 0s now
            printf("0");
        }
    }

    // If the number is 0, we need to print "0"
    if (leading_zero) {
        printf("0");
    }
}


esp_err_t gyro_set_range(gyro_t* gyro, uint8_t fs_sel){
    if (fs_sel > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = bmi160_write_byte(gyro->i2c.address, BMI160_GYRO_CONF, fs_sel << 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set_gyro_range: %s\n", esp_err_to_name(ret));
    }
    return ret;

}
float gyro_get_sensitivity(gyro_t gyro){
    uint8_t gyro_sensitivity_setting;
    bmi160_read_bytes(gyro.i2c.address, BMI160_GYRO_RANGE, &gyro_sensitivity_setting, 1);

    switch (gyro_sensitivity_setting & 0b111) {
        case 0b000:  // ±2000 °/s
            return 2000 / 32768.0;
        case 0b001:  // ±1000 °/s
            return 1000.0 / 32768.0;
        case 0b010:  // ±500 °/s
            return 500.0 / 32768.0;
        case 0b011:  // ±250 °/s
            return 250.0 / 32768.0;
        case 0b100:  // ±125 °/s
            return 125.0 / 32768.0;
        default:
            return 250.0 / 32768.0;  // Default sensitivity CHECK
    }

}

esp_err_t gyro_calibrate(gyro_t* gyro, uint32_t samples) {

    int cum_value_x = 0;
    int cum_value_y = 0;
    int cum_value_z = 0;

    uint8_t sensor_data[14];    // Buffer for accelerometer and gyroscope
    int16_t gyro_x, gyro_y, gyro_z;


    ESP_LOGI(TAG, "Gyro calibration: Keep the device in the operation position until calibration is finished.");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Starting in 3");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Starting in 2");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Starting in 1");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Gyro calibration starting...");

    for (int i = 0; i < samples; i++) {

        bmi160_read_bytes(gyro->i2c.address, BMI160_GYRO_REGISTER, sensor_data, 6);

        gyro_x = ((sensor_data[1] << 8) | sensor_data[0]);
        gyro_y = ((sensor_data[3] << 8) | sensor_data[2]);
        gyro_z = ((sensor_data[5] << 8) | sensor_data[4]);

        cum_value_x += gyro_x; 
        cum_value_y += gyro_y; 
        cum_value_z += gyro_z;

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    gyro->offset.x = cum_value_x/(float)samples;
    gyro->offset.y = cum_value_y/(float)samples;
    gyro->offset.z = cum_value_z/(float)samples;

    ESP_LOGI(TAG, "Gyro calibration completed.\nOffsets set:\nx: %.2f\ny: %.2f\nz: %.2f", gyro->offset.x, gyro->offset.y, gyro->offset.z);
    vTaskDelay(pdMS_TO_TICKS(1000));

    return ESP_OK;
}

esp_err_t bmi160_foc(bmi160_t* bmi){

    // read offsets before FOC
    int8_t previous_offset[6];
    bmi160_read_bytes(bmi->i2c.address, 0x71, &previous_offset, 6);

    int8_t prev_acc_x = previous_offset[0];
    int8_t prev_acc_y = previous_offset[1];
    int8_t prev_acc_z = previous_offset[2];

    int16_t prev_gyro_x = (int16_t)((previous_offset[6] & 0b00000011) << 8) | previous_offset[3];
    int16_t prev_gyro_y = (int16_t)((previous_offset[6] & 0b00001100) << 6) | previous_offset[4];
    int16_t prev_gyro_z = (int16_t)((previous_offset[6] & 0b00110000) << 4) | previous_offset[5];

    // handle sign extension for 10-bit values (two's complement)
    if (prev_gyro_x & 0x0200) prev_gyro_x |= 0xFC00;  // Sign-extend if negative
    if (prev_gyro_y & 0x0200) prev_gyro_y |= 0xFC00;  // Sign-extend if negative
    if (prev_gyro_z & 0x0200) prev_gyro_z |= 0xFC00;  // Sign-extend if negative

    float prev_gyro_x_dps = prev_gyro_x * 0.061;
    float prev_gyro_y_dps = prev_gyro_y * 0.061;
    float prev_gyro_z_dps = prev_gyro_z * 0.061;

    float prev_acc_x_g = prev_acc_x * 0.00391;
    float prev_acc_y_g = prev_acc_y * 0.00391;
    float prev_acc_z_g = prev_acc_z * 0.00391;


    ESP_LOGI(TAG, "acc_x_off: %.2fG - acc_y_off: %.2fG - acc_z_off: %.2fG\n", prev_acc_x_g, prev_acc_y_g, prev_acc_z_g);

    ESP_LOGI(TAG, "gyro_x_off: %.2f°/s - gyro_y_off: %.2f°/s - gyro_z_off: %.2f°/s", prev_gyro_x_dps, prev_gyro_y_dps, prev_gyro_z_dps);

    // configure FOC
    uint8_t foc_config = BMI160_FOC_GYRO_ENABLE || (BMI160_FOC_ACC_DISABLED << BMI160_FOC_ACC_X_SHIFT) || (BMI160_FOC_ACC_DISABLED << BMI160_FOC_ACC_Y_SHIFT) || (BMI160_FOC_ACC_DISABLED << BMI160_FOC_ACC_Z_SHIFT)  ;

    bmi160_write_byte(bmi->i2c.address, BMI160_FOC_CONF_REG, foc_config);

    // trigger FOC by cmd
    bmi160_write_byte(bmi->i2c.address, BMI160_CMD_REG, BMI160_FOC_START_CMD);

    // wait for FOC to complete
    uint8_t status = 0;
    do {
        bmi160_read_bytes(bmi->i2c.address, BMI160_STATUS_REG, &status, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    } while (status & 0x08);  // FOC running bit

    // read data to clear data ready bit
    uint8_t dummy_sensor_data[12];
    bmi160_read_bytes(bmi->i2c.address, BMI160_GYRO_REGISTER, &dummy_sensor_data, 12);

    ESP_LOGI(TAG, "Fast Offset Compensation completed.");

    // enable offset
    uint8_t current_offset_bit;
    bmi160_read_bytes(bmi->i2c.address, 0x77, &current_offset_bit, 1);
    bmi160_write_byte(bmi->i2c.address, 0x77, 0b11000000 | current_offset_bit);

    // get and print offsets
    uint8_t offset_data[6];
    bmi160_read_bytes(bmi->i2c.address, 0x71, &offset_data, 6);

    int8_t acc_x = offset_data[0];
    int8_t acc_y = offset_data[1];
    int8_t acc_z = offset_data[2];

    int16_t gyro_x = (int16_t)((offset_data[6] & 0b00000011) << 8) | offset_data[3];
    int16_t gyro_y = (int16_t)((offset_data[6] & 0b00001100) << 6) | offset_data[4];
    int16_t gyro_z = (int16_t)((offset_data[6] & 0b00110000) << 4) | offset_data[5];

    // Handle sign extension for 10-bit values (two's complement)
    if (gyro_x & 0x0200) gyro_x |= 0xFC00;  // Sign-extend if negative
    if (gyro_y & 0x0200) gyro_y |= 0xFC00;  // Sign-extend if negative
    if (gyro_z & 0x0200) gyro_z |= 0xFC00;  // Sign-extend if negative

    float gyro_x_dps = gyro_x * 0.061;
    float gyro_y_dps = gyro_y * 0.061;
    float gyro_z_dps = gyro_z * 0.061;

    float acc_x_g = acc_x * 0.00391;
    float acc_y_g = acc_y * 0.00391;
    float acc_z_g = acc_z * 0.00391;

    ESP_LOGI(TAG, "acc_x_off: %.2fG - acc_y_off: %.2fG - acc_z_off: %.2fG\n", acc_x_g, acc_y_g, acc_z_g);

    ESP_LOGI(TAG, "gyro_x_off: %.2f°/s - gyro_y_off: %.2f°/s - gyro_z_off: %.2f°/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);

    return ESP_OK;
}
