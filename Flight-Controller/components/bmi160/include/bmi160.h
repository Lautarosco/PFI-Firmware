#include <stdint.h>
#include "esp_system.h"
#include "registers.h"

// Structs

typedef struct offset {
    float x;
    float y;
    float z;
} offset_t;

typedef struct i2c_config {
    int address;
    int scl;
    int sda;

} i2c_params_t;

typedef struct magnetometer {
    float x;
    float y;
    float z;

    i2c_params_t i2c;

    offset_t offset;

    float declination;
    float resolution;
    float mode;

    esp_err_t (*set_mode_res)(uint8_t mag_mode, uint8_t mag_res);
    esp_err_t (*enable_bypass)(struct magnetometer* self);

} mag_t;

typedef struct accelerometer {
    float x;
    float y;
    float z;

    i2c_params_t i2c;

    offset_t offset;

    float range;

    float (*get_sensitivity)(struct accelerometer self);
    esp_err_t (*set_range)(struct accelerometer* self, uint8_t range);

} acc_t;

typedef struct gyroscope {
    float x;
    float y;
    float z;

    i2c_params_t i2c;

    offset_t offset;

    float range;

    float (*get_sensitivity)(struct gyroscope self);
    esp_err_t (*set_range)(struct gyroscope* self, uint8_t range);
    esp_err_t (*calibrate)(struct gyroscope* self, uint32_t samples);

} gyro_t;

typedef struct temperature {
    float temperature;
    i2c_params_t i2c;


} temp_t;

typedef struct bmi160_t {
    // Device config
    i2c_params_t i2c;

    esp_err_t (*init)(struct bmi160_t* self, int bmi_address, int acc_mode, int acc_freq, int acc_range, int gyro_mode, int gyro_freq, int gyro_range, int gyro_offset_x, int gyro_offset_y, int gyro_offset_z);
    esp_err_t (*measure)(struct bmi160_t* self);
    esp_err_t (*foc)(struct bmi160_t* self);
    
    // Sensors
    acc_t Acc;
    mag_t Mag;
    gyro_t Gyro;
    temp_t Temp;



} bmi160_t;



//Methods
esp_err_t acc_set_range(acc_t* acc, uint8_t fs_sel);
float acc_get_sensitivity(acc_t acc);

esp_err_t gyro_set_range(gyro_t* gyro, uint8_t fs_sel);
float gyro_get_sensitivity(gyro_t gyro);
esp_err_t gyro_calibrate(gyro_t* gyro, uint32_t samples);

esp_err_t mag_enable_bypass(mag_t* mag);
esp_err_t mag_set_mode_resolution(uint8_t mode, uint8_t resolution);
float get_mag_sensitivity();

/** 
 * @brief: Reads each sensor and updates its values
 */
esp_err_t bmi160_measure(bmi160_t* bmi);

esp_err_t bmi160_foc(bmi160_t* bmi);

/**
 * @brief Make an instance of Bmi160 Class.
 * @param bmi: Pointer to bmi160_t struct.
 * @param i2c_address_param: bmi address.
 * @param i2c_scl_param: scl GPIO.
 * @param i2c_sda_param: sda GPIO.
 * @retval esp_err_t
 */
esp_err_t Bmi160( bmi160_t* bmi,
                        int i2c_address_param,
                        int i2c_scl_param,
                        int i2c_sda_param
                );

/**
 * @brief Initialize Acelerometer and Gyroscope
 * @param self: Pointer to bmi160_t struct.
 * @param bmi_address: Address of bmi register.
 * @param acc_mode:    Accelerometer mode.
 * @param acc_freq:    Accelerometer operation frequency.
 * @param acc_range:   Accelerometer range.
 * @param gyro_mode:   Gyroscope mode.
 * @param gyro_freq:   Gyroscope operation frequency.
 * @param gyro_range:  Gyroscope range.
 * @param gyro_offset_x: Gyroscope x offset.
 * @param gyro_offset_y: Gyroscope y offset.
 * @param gyro_offset_z: Gyroscope z offset.
 * @retval esp_err_t
 */
esp_err_t bmi_init( bmi160_t * self, int bmi_address,
    int acc_mode,  int acc_freq,  int acc_range,
    int gyro_mode, int gyro_freq, int gyro_range,
    int gyro_offset_x, int gyro_offset_y, int gyro_offset_z
);
        