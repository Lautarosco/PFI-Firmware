#ifndef DRONE_STRUCTS_H
#define DRONE_STRUCTS_H

#include <drone_globals.h>
#include <bmi160.h>
#include <transmitter.h>
#include <pwm.h>
#include <controllers.h>
#include <mma.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone states */

typedef struct drone_states {
    /* z linear position */
    float z;

    /* roll angular position */
    float roll;

    /* pitch angular position */
    float pitch;

    /* yaw angular position */
    float yaw;

    /* roll angular velocity */
    float roll_dot;

    /* pitch angular velocity */
    float pitch_dot;

    /* yaw angular velocity */
    float yaw_dot;

} drone_states_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone's components */

typedef struct drone_components {
    /* bmi160 component */
    bmi160_t * bmi;

    /* Transmitter component */
    transmitter_t * Tx;

    /* Pwm component */
    pwm_t * pwm[ 4 ];

    /* Mma component */
    mma_t * mma;

    /* Controller component */
    pid_controller_t * controllers[ 7 ];
    
} drone_components_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Kalman parameters of drone's attitude */

typedef struct drone_kalman {
    float error;
    float Q;
    float R;

} drone_kalman_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief IMU configs */

typedef struct drone_imu_cfg {
    /* IMU i2c configs */
    i2c_params_t imu_i2c_cfg;
    
    /* Accelerometer mode */
    uint32_t acc_mode;
    
    /* Accelerometer operatring frequency */
    uint32_t acc_freq;
    
    /* Accelerometer range */
    uint32_t acc_range;
    
    /* Gyroscope mode */
    uint32_t gyro_mode;
    
    /* Gyroscope operatring frequency */
    uint32_t gyro_freq;
    
    /* Gyroscope range */
    uint32_t gyro_range;
    
    /* Gyroscope offsets */
    offset_t gyro_offset;

} drone_imu_cfg_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone's cfgs */

typedef struct drone_cfg {
    /* Kalman parameters for roll */
    drone_kalman_t roll;
    
    /* Kalman parameters for pitch */
    drone_kalman_t pitch;
    
    /* Kalman parameters for yaw */
    drone_kalman_t yaw;
    
    /* First order IIR coefficient */
    float IIR_coeff_roll_dot;
    
    /* First order IIR coefficient */
    float IIR_coeff_pitch_dot;
    
    /* First order IIR coefficient */
    float IIR_coeff_yaw_dot;

    /* IMU configs */
    drone_imu_cfg_t imu_cfg;

    /* Transmitter Mac address */
    uint8_t esp_mac_addr[ MAC_ADDR_SIZE ];

    /* Pwm configs */
    pwm_cfg_t pwm_cfg[ 4 ];

    /* Controllers configs */
    ControllerCfgs_t ControllersConfigs[ 7 ];

} drone_cfg_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone's attributes */

typedef struct drone_attributes {
    /* Drone's states */
    drone_states_t states;

    /* Drone's components */
    drone_components_t components;

    /* Drone's global variables */
    drone_globals_t global_variables;

    /* Drone's cfgs */
    // drone_cfg_t cfg;

    bool init_ok;

} drone_attributes_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/* Forward declaration to avoid compiler warning in pointer functions */
typedef struct drone drone_t;

/** @brief Drone's methods */

typedef struct drone_methods {
    /** @brief Update Drone object states @param obj: Address of Drone object @param ts: Sampling time in milliseconds @retval none */
    void ( * update_states )( drone_t * obj, float ts );

    /** @brief Initialize an object of Drone Class @param obj: Address of Drone object @param drone_cfg: Drone's configs @retval esp_err_t */
    esp_err_t ( * init )( drone_t * obj );

    /** @brief Seek devices in I2C bus @param none @retval bool */
    bool ( * i2c_scan )( void );

} drone_methods_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone Class */

typedef struct drone {
    /* Drone's attributes */
    drone_attributes_t attributes;

    /* Drone's methods */
    drone_methods_t methods;

} drone_t;


#endif
