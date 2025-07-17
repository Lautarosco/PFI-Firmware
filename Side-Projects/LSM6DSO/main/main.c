#include <stdio.h>
#include <application_layer/lsm6dso_app_layer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <string.h>
#include <math.h>

#define LSM6DSO_ADDR                    0x6B
#define LSM6DSO_I2C_SCL_FREQ_HZ         100000
#define GPIO_SDA                        GPIO_NUM_21
#define GPIO_SCL                        GPIO_NUM_22

/**
 * Notes:
 * 
 * 1. Accelerometer LPF1 cut off frequency is ODR/2 when high-performance is enabled, else LPF1 is equal to 700 Hz.
 * 2. Accelerometer LPF2 can be enabled and configured
 * 3. Gyroscope LPF2 cannot be configured by the user and it value is round ODR/3
 * 4. Gyroscope LPF1 can be enabled and configured
 */

typedef struct angles {
    float roll;
    float pitch;
    float yaw;
} angles_t;

esp_err_t convert_to_angles(angles_t *angles, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z) {
    float dt = 1.0f / 104.0f;
    float alpha = 0.95f;

    /* Calculate roll and pitch with accelerometer measurements (trigonometry) */
    float pitch_acc = atan2(acc_y, sqrt((acc_x * acc_x) + (acc_z * acc_z)));
    float roll_acc  = atan2(-acc_x, sqrt((acc_y * acc_y) + (acc_z * acc_z)));


    /* Update yaw with gyroscope measurements (integration) */
    angles->yaw += gyro_z * dt;

    /* Update roll and pitch by accelerometer trigonometric equations */
    angles->pitch = alpha * (angles->pitch + gyro_x * dt) + (1 - alpha) * pitch_acc;
    angles->roll  = alpha * (angles->roll  + gyro_y * dt) + (1 - alpha) * roll_acc;

    return ESP_OK;
}

void app_main(void)
{
    i2c_master_bus_handle_t i2c_master_handler = NULL;
    i2c_master_dev_handle_t i2c_lsm_handler = NULL;

    i2c_master_bus_config_t i2c_master_cfg = {
        .clk_source                   = I2C_CLK_SRC_APB,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = GPIO_SCL,
        .sda_io_num                   = GPIO_SDA,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_device_config_t i2c_lsm_cfg = {
        .device_address          = LSM6DSO_ADDR,
        .dev_addr_length         = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz            = LSM6DSO_I2C_SCL_FREQ_HZ,
        .flags.disable_ack_check = false,
        .scl_wait_us             = 0
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_master_cfg, &i2c_master_handler);
    if(ret != ESP_OK) {
        printf("[%s]: Error in line %d\n", __func__, __LINE__);
        return;
    }
    printf("I2C master bus created\n");
    
    ret = i2c_master_bus_add_device(i2c_master_handler, &i2c_lsm_cfg, &i2c_lsm_handler);
    if(ret != ESP_OK) {
        printf("[%s]: Error in line %d\n", __func__, __LINE__);
        return;
    }
    printf("0x%X added to I2C bus\n", i2c_lsm_cfg.device_address);

    lsm6dso_t imu;
    Lsm6dso(&imu);

    lsm6dso_params_t imu_params = {
        .i2c_lsm_handler = &i2c_lsm_handler,
        .acc = {
            .fs = LSM6DSO_ACC_FS_4G,
            .odr = LSM6DSO_ACC_ODR_104_HZ,
            .lpf2_en = LSM6DSO_ACC_LPF2_DISABLE,
            .unit = LSM6DSO_ACC_UNIT_MS2
        },
        .gyro = {
            .fs = LSM6DSO_FS_GYRO_250_DPS,
            .odr = LSM6DSO_ODR_GYRO_104_HZ,
            .lpf1_en = LSM6DSO_GYRO_LPF1_DISABLE,
            .hpf_en = LSM6DSO_GYRO_HPF_DISABLE,
            .unit = LSM6DSO_FS_GYRO_UNIT_DEFAULT
        }
    };
    
    uint8_t dummy = 0;
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        ret = i2c_master_transmit(i2c_lsm_handler, &dummy, sizeof(dummy), 50);
        if((ret == ESP_OK) && (addr == LSM6DSO_ADDR)) {
            printf("Found LSM6DSO at address 0x%X\n", addr);
            break;
        }
    }

    ret = imu.init(&imu, imu_params);
    angles_t angles;
    memset(&angles, 0, sizeof(angles_t));
    
    // float avg = 0.0f;
    // for(int i = 0; i < 100000; i++) {
    //     imu.measure(&imu);
    //     avg += imu.acc.z;
    // }
    // printf("Promedio: %f\n", avg / 100000.0f);
    // return;

    while(1) {
        imu.measure(&imu);
        convert_to_angles(&angles, imu.acc.x, imu.acc.y, imu.acc.z, imu.gyro.x, imu.gyro.y, imu.gyro.z);
        
        /* Print results */
        // printf(
        //     "Acc_x: %f m/s^2, Acc_y: %f m/s^2, Acc_z: %f m/s^2\n",
        //     imu.acc.x, imu.acc.y, imu.acc.z
        // );

        printf(
            "Roll: %f °, Pitch: %f °, Yaw: %f °\n",
            angles.roll, angles.pitch, angles.yaw
        );

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return;
}
