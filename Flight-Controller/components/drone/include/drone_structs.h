#pragma once
#include <string.h>

#include <drone_globals.h>
#include "state_machine.h"
#include <bmi160.h>
#include "indicators.h"
#include "application_layer/bmp390_app_layer.h"
#include "application_layer/gnss_app_layer.h"
#include <transmitter.h>
#include <pwm.h>
#include <controllers.h>
#include <mma.h>
#include <state_machine.h>
#include "print_manager.h"
#include <application_layer/lsm6dso_app_layer.h>

#define MAX_FLASH_PARAMS 60 /* Total Drone parameters to be stored in flash memory */
#define NVS_NAMESPACE "storage"


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


typedef struct {
    const char* name;
    void* ptr;
    size_t size;
    bool save_to_flash;
} drone_parameter_t;

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

// TODO - Make a component for battery management
/**
 * @struct battery
 * @brief Represents the state and measurements of a battery.
 */
typedef struct battery {
    /* Battery level as a percentage (0.0 - 100.0). */
    float level;

    /* Battery voltage in volts */
    float voltage;

    /* Number of cells */
    int cells;

    /* Battery current (not implemented yet) */
    // float current;

    /* Battery temperature (not implemented yet) */
    // float temperature;

} battery_t;

// Function declaration only - definition should be in a .c file
void Battery( battery_t * battery );


/** @brief Drone's components */
typedef struct drone_components {
    
    /* bmi160 component  */
    bmi160_t bmi;

    /* LSM6DSO component */
    lsm6dso_t imu;

    /* bmp390 component */
    bmp390_t bmp;

    /* gnss component */
    gnss_t gnss;

    /* Transmitter component */
    transmitter_t Tx;

    /* Pwm component */
    pwm_t pwm[ 4 ];

    /* Mma component */
    mma_t mma;

    /* Battery component */
    battery_t battery;

    /* Indicators */
    indicators_t indicators;

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

    /* Drone's print manager */
    print_manager_t print_manager;

    /* Array of Drone parameters stored in flash memory */
    drone_parameter_t* flash_params_arr;

    /* Drone's init flag */
    bool init_ok;

    /* Sampling time in milliseconds */
    unsigned int ts_ms;

    /* State machine of Drone Class */
    sm_state_machine_t state_machine;
    bool request_state_transition;
    sm_event_t requested_transition_event;
    int64_t sm_cycle_time;
    int64_t measure_cycle_time;

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
