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
 * @brief PID Gains
 */
typedef struct pid_gain {

    /* Proportional gain */
    float kp;
    
    /* Integral gain */
    float ki;
    
    /* Derivative gain */
    float kd;

    /* Back Calculation gain */
    float Kb;

} pid_gain_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


typedef struct pid_limits {

    /* Minimum value */
    float min;

    /* Maximum value */
    float max;

} pid_limits_t;


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
    float d;
    
} pid_action_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Forward declaration to avoid warning in function pointers */

typedef struct pid_controller pid_controller_t;
typedef float ControllerFunction( pid_controller_t * obj, float error );

/**
 * @brief Complete definition of Pid Class
 */
typedef struct pid_controller {

    /* [ A ] Error */
    float error;

    /* [ A ] Previous error */
    float prev_error;

    /* [ A ] Controller TAG */
    states_t tag;

    /* [ A ] Sampling time in milliseconds */
    float ts_ms;

    /* [ A ] Controller gains struct */
    pid_gain_t gain;

    /* [ A ] Integrator state */
    float integrator;

    /* [ A ] Integral limits */
    pid_limits_t integral_limits;

    /* [ A ] PID Output limits */
    pid_limits_t pid_out_limits;

    /* [ A ] Flag to check if object was initialized */
    bool init_ok;

    /** @brief [ M ] Initialize Pid object @param obj: Address of Pid object @retval none */
    void ( * init )( pid_controller_t * obj, states_t tag, float ts_ms, pid_gain_t pid_gains, pid_limits_t integral_limits, pid_limits_t pid_limits );

    float ( * pidUpdate )( pid_controller_t * obj, float pv, float sp );

    ControllerFunction* pFunc;
    ControllerFunction* iFunc;
    ControllerFunction* dFunc;

    void ( * PidSetActionP )( pid_controller_t* obj, float ( * pFunc )( pid_controller_t * obj, float error ) );
    void ( * PidSetActionI )( pid_controller_t* obj, float ( * pFunc )( pid_controller_t * obj, float error ) );
    void ( * PidSetActionD )( pid_controller_t* obj, float ( * pFunc )( pid_controller_t * obj, float error ) );

} pid_controller_t;


#endif
