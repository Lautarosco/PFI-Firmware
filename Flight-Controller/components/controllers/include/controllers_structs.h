#ifndef STRUCT_QUADCOPTER_CONTROLLER
#define STRUCT_QUADCOPTER_CONTROLLER

#include <math.h>
#include <stdbool.h>


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Drone Class states enumeration
 */
typedef enum states {

    /* z index used in Arrays */
    z,
    
    /* Roll index used in Arrays */
    roll,
    
    /* Pitch index used in Arrays */
    pitch,
    
    /* Yaw index used in Arrays */
    yaw,
    
    /* Roll_dot index used in Arrays */
    roll_dot,
    
    /* Pitch_dot index used in Arrays */
    pitch_dot,
    
    /* Yaw_dot index used in Arrays */
    yaw_dot,
    
} states_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Filter derivative action options
 */
typedef enum filter_derivative {

    /* Do not filter derivative action */
    no_filter,

    /* Filter derivative action */
    filter,

} filter_derivative_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Integral action saturation options
 */
typedef enum int_sat {

    /* No integral saturation */
    no_saturation,
    
    /* Anti-Windup saturation */
    anti_windup,
    
    /* Back-Propagation saturation */
    back_propagation,

} int_sat_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief PID Gains
 */
typedef struct pid_gain {

    /* Proportional gain */
    float kp;
    
    /* Integral gain */
    float ki;
    
    /* Derivative gain */
    float kd;

} pid_gain_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Struct of an integral
 */
typedef struct derivative {

    /* Accumulation of the integral */
    float sum;

    /* Actual value of the integral */
    float out;

} derivative_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief PID Actions
 */
typedef struct pid_action {

    /* Proportional action */
    float p;
    
    /* Integral action */
    float i;
    
    /* Derivative action */
    derivative_t d;
    
} pid_action_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief PID configs
 */
typedef struct ControllerCfgs {

    /* Controller TAG */
    states_t tag;

    /* LPF cut-off frequency ( D action ) */
    float fc;

    /* Sampling time in milliseconds for discrete blocks */
    float ts;

    /* Integral saturation */
    int_sat_t sat;

    /* Derivative filter */
    filter_derivative_t der_filter;

    /* PID gains */
    pid_gain_t gains;

    /* Minimum error to start using integral action */
    float intMinErr;

} ControllerCfgs_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Forward declaration to avoid warning in function pointers */

typedef struct pid_controller pid_controller_t;

/**
 * @brief Complete definition of Pid Class
 */
typedef struct pid_controller {

    /* [ A ] Error */
    float error;

    /* [ A ] Controller gains struct */
    pid_gain_t gain;

    /* [ A ] Controller actions struct */
    pid_action_t action;

    /* [ A ] Controller cfgs */
    ControllerCfgs_t cfg;

    /* [ A ] Flag to check if object was initialized */
    bool init_ok;

    /** @brief [ M ] Initialize Pid object @param obj: Address of Pid object @param cfg: Controller configs @retval none */
    void ( * init )( pid_controller_t * obj, ControllerCfgs_t cfg );

    /** @brief [ M ] Calculate Controller action @param obj: Address of Pid object @param pv: Process value @param sp: Set point @retval PID calculation */
    float ( * pid )( pid_controller_t * obj, float pv, float sp );

    /** @brief Calculate Controller PI-D action @param obj: Address of Pid object @param pv: Process value @param sp: Set point @retval PID calculation */
    float ( * manual_pi_d )( pid_controller_t * obj, float pv, float sp, float d_state );

} pid_controller_t;


#endif
