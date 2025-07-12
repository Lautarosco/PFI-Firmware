#ifndef DRONE_STRUCTS_H
#define DRONE_STRUCTS_H

#include <drone_globals.h>
#include "state_machine.h"
#include <bmi160.h>
#include <transmitter.h>
#include <pwm.h>
#include <controllers.h>
#include <mma.h>
#include <state_machine.h>

#define FLASH_PARAMS 40 /* Total Drone parameters to be stored in flash memory */
#define NVS_NAMESPACE "storage"


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Drone parameters stored and updated from flash memory */
typedef enum drone_flash_params {

    /* Gyro offset x */
    GYRO_OFFSET_X,

    /* Gyro offset y */
    GYRO_OFFSET_Y,

    /* Gyro offset z */
    GYRO_OFFSET_Z,

    /* Roll Kp gain */
    PID_ROLL_KP,

    /* Roll Ki gain */
    PID_ROLL_KI,

    /* Roll Kd gain */
    PID_ROLL_KD,

    /* Roll KB gain */
    PID_ROLL_KB,

    /* Roll_d Kp gain */
    PID_ROLL_D_KP,

    /* Roll_d Ki gain */
    PID_ROLL_D_KI,

    /* Roll_d Kd gain */
    PID_ROLL_D_KD,

    /* Roll_d KB gain */
    PID_ROLL_D_KB,

    /* Roll_d IIR filter coefficient */
    ROLL_D_IIR_COEFF,

    /* Pitch Kp gain */
    PID_PITCH_KP,

    /* Pitch Ki gain */
    PID_PITCH_KI,

    /* Pitch Kd gain */
    PID_PITCH_KD,

    /* Pitch KB gain */
    PID_PITCH_KB,

    /* Pitch_d Kp gain */
    PID_PITCH_D_KP,

    /* Pitch_d Ki gain */
    PID_PITCH_D_KI,

    /* Pitch_d Kd gain */
    PID_PITCH_D_KD,

    /* Pitch_d KB gain */
    PID_PITCH_D_KB,

    /* Pitch_d IIR filter coefficient */
    PITCH_D_IIR_COEFF,

    /* Yaw Kp gain */
    PID_YAW_KP,

    /* Yaw Ki gain */
    PID_YAW_KI,

    /* Yaw Kd gain */
    PID_YAW_KD,

    /* Yaw KB gain */
    PID_YAW_KB,

    /* Yaw_d Kp gain */
    PID_YAW_D_KP,

    /* Yaw_d Ki gain */
    PID_YAW_D_KI,

    /* Yaw_d Kd gain */
    PID_YAW_D_KD,

    /* Yaw_d KB gain */
    PID_YAW_D_KB,

    /* Yaw_d IIR filter coefficient */
    YAW_D_IIR_COEFF,

    /* Z Kp gain */
    PID_Z_KP,

    /* Z Ki gain */
    PID_Z_KI,

    /* Z Kd gain */
    PID_Z_KD,

    /* Z KB gain */
    PID_Z_KB,

    /* Z_d Kp gain */
    PID_Z_D_KP,

    /* Z_d Ki gain */
    PID_Z_D_KI,

    /* Z_d Kd gain */
    PID_Z_D_KD,

    /* Z_d KB gain */
    PID_Z_D_KB,

    /* Z_d IIR filter coefficient */
    Z_D_IIR_COEFF,

} drone_flash_params_t;

static const char* key_names[] = {
    "gyro_offset_x",      // GYRO_OFFSET_X = 0
    "gyro_offset_y",      // GYRO_OFFSET_Y = 1
    "gyro_offset_z",      // GYRO_OFFSET_Z = 2
    "pid_roll_kp",        // PID_ROLL_KP = 3
    "pid_roll_ki",        // PID_ROLL_KI = 4
    "pid_roll_kd",        // PID_ROLL_KD = 5
    "pid_roll_kb",        // PID_ROLL_KB = 6
    "pid_roll_d_kp",      // PID_ROLL_D_KP = 7
    "pid_roll_d_ki",      // PID_ROLL_D_KI = 8
    "pid_roll_d_kd",      // PID_ROLL_D_KD = 9
    "pid_roll_d_kb",      // PID_ROLL_D_KB = 10
    "gyro_x_ema",         // ROLL_D_IIR_COEFF = 11
    "pid_pitch_kp",       // PID_PITCH_KP = 12
    "pid_pitch_ki",       // PID_PITCH_KI = 13
    "pid_pitch_kd",       // PID_PITCH_KD = 14
    "pid_pitch_kb",       // PID_PITCH_KB = 15
    "pid_pitch_d_kp",     // PID_PITCH_D_KP = 16
    "pid_pitch_d_ki",     // PID_PITCH_D_KI = 17
    "pid_pitch_d_kd",     // PID_PITCH_D_KD = 18
    "pid_pitch_d_kb",     // PID_PITCH_D_KB = 19
    "gyro_y_ema",         // PITCH_D_IIR_COEFF = 20
    "pid_yaw_kp",         // PID_YAW_KP = 21
    "pid_yaw_ki",         // PID_YAW_KI = 22
    "pid_yaw_kd",         // PID_YAW_KD = 23
    "pid_yaw_kb",         // PID_YAW_KB = 24
    "pid_yaw_d_kp",       // PID_YAW_D_KP = 25
    "pid_yaw_d_ki",       // PID_YAW_D_KI = 26
    "pid_yaw_d_kd",       // PID_YAW_D_KD = 27
    "pid_yaw_d_kb",       // PID_YAW_D_KB = 28
    "gyro_z_ema",         // YAW_D_IIR_COEFF = 29
    "pid_z_kp",           // PID_Z_KP = 30
    "pid_z_ki",           // PID_Z_KI = 31
    "pid_z_kd",           // PID_Z_KD = 32
    "pid_z_kb",           // PID_Z_KB = 33
    "pid_z_d_kp",         // PID_Z_D_KP = 34
    "pid_z_d_ki",         // PID_Z_D_KI = 35
    "pid_z_d_kd",         // PID_Z_D_KD = 36
    "pid_z_d_kb",         // PID_Z_D_KB = 37
    "z_d_iir_coeff"       // Z_D_IIR_COEFF = 38
};

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

    /* z linear velocity */
    float z_dot;

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
    
    /* bmi160 component  */
    bmi160_t bmi;

    /* Transmitter component */
    transmitter_t Tx;

    /* Pwm component */
    pwm_t pwm[ 4 ];

    /* Mma component */
    mma_t mma;

    /* Controller component */
    pid_controller_t controllers[ 8 ];
    
} drone_components_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @brief Kalman parameters of drone's attitude */
typedef struct drone_kalman {

    /* Estimated uncertainty related to the estimation */
    float P;

    /* Process noise covariance */
    float Q;

    /* Sensor noise covariance */
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


typedef struct PidCfgs {

    /* Integral saturation limits */
    pid_limits_t integral_limits;

    /* PID output saturation limits */
    pid_limits_t pid_output_limits;

    states_t tag;

} PidCfgs_t;


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

    PidCfgs_t pid_cfgs[ 8 ];

    /* Upper and lower limits of mma output */
    limits_t mma_out_limits;

} drone_cfg_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */
typedef struct synced_tx_buttons {
    tx_buttons_t current;
    tx_buttons_t previous;
} synced_tx_buttons_t;

/** @brief Drone's attributes */
typedef struct drone_attributes {

    /* Drone's states */
    drone_states_t states;

    /* Drone's sp */
    drone_states_t sp;

    /* Drone's components */
    drone_components_t components;

    /* Buttons */
    synced_tx_buttons_t buttons;

    /* Drone's global variables */
    drone_globals_t global_variables;

    /* Drone's configuration parameters */
    drone_cfg_t config;

    /* Array of Drone parameters stored in flash memory */
    void * flash_params_arr[ FLASH_PARAMS ];

    /* Drone's init flag */
    bool init_ok;

    /* State machine of Drone Class */
    sm_state_machine_t state_machine;
    bool request_state_transition;
    sm_event_t requested_transition_event;


} drone_attributes_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/* Forward declaration to avoid compiler warning in pointer functions */
typedef struct drone drone_t;

/** @brief Drone's methods */
typedef struct drone_methods {
    /** @brief Update Drone object states @param drone: Address of Drone object @param ts: Sampling time in milliseconds @retval none */
    void ( * update_states )( drone_t * drone, float ts );

    /** @brief Update Drone set points @param drone: Address of Drone object @retval none */
    void (*update_sp)(drone_t * drone);

    /** @brief Initialize an object of Drone Class @param drone: Address of Drone object @param drone_cfg: Drone's configs @retval esp_err_t */
    esp_err_t ( * init )( drone_t * drone );

    /** @brief Seek devices in I2C bus @param none @retval bool */
    bool ( * i2c_scan )( void );

    /** @brief Save Drone parameters to flash memory @param drone: Direction of Drone object @retval none */
    void ( * save_to_nvs )( drone_t * drone );

    /** @brief Read Drone parameters stored in flash memory @param drone: Direction of Drone object @retval none */
    void ( * read_from_flash )( drone_t * drone );

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
