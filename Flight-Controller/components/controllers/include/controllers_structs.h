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
    Z,
    
    /* Roll index used in Arrays */
    ROLL,
    
    /* Pitch index used in Arrays */
    PITCH,
    
    /* Yaw index used in Arrays */
    YAW,
    
    /* Roll_d index used in Arrays */
    ROLL_D,
    
    /* Pitch_d index used in Arrays */
    PITCH_D,
    
    /* Yaw_d index used in Arrays */
    YAW_D,
    
} states_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Filter derivative action options
 */
typedef enum filter_derivative {

    /* Do not filter derivative action */
    NO_FILTER,

    /* Filter derivative action */
    FILTER,

} filter_derivative_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Integral action saturation options
 */
typedef enum int_sat {

    /* No integral saturation */
    NO_SATURATION,
    
    /* Anti-Windup saturation */
    ANTI_WINDUP,
    
    /* Back-Propagation saturation */
    BACK_PROPAGATION,

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

    /** @brief Calculate Controller PI-D action
     * @param obj: Address of Pid object 
     * @param pv: Process value 
     * @param sp: Set point
     * @param d_state: D value  
     * @retval PID calculation */
    float ( * manual_pi_d )( pid_controller_t * obj, float pv, float sp, float d_state );

} pid_controller_t;


#endif
