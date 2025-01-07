#include <stdio.h>
#include <controllers.h>
#include <math.h>
#include <esp_system.h>
#include <esp_log.h>

#define N       1       /* Actual sample [ n ] of buffers used in discrete algorithms */
#define N_1     0       /* 1 sample delay [ n - 1 ] of buffers used in discrete algorithms */

const char * CONTROLLER_TAG = "CONTROLLER";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Calculate Controller action
 * @param obj: Address of Pid object
 * @param pv: Process value
 * @param sp: Set point
 * @retval PID calculation
 */
static float pid( pid_controller_t * obj, float pv, float sp ) {

    if( !obj->init_ok ) {

        ESP_LOGE( CONTROLLER_TAG, "Object is not initialized!" );
        esp_restart();
    }
    
    float w0 = 2 * M_PI * obj->cfg.fc;   /* Calculate Low Pass Filter coefficient in rad/s */

    obj->error_buffer[ N_1 ] = obj->error_buffer[ N ];          /* Update last Error */
    obj->error_buffer[ N ]   = sp - pv;                         /* Update new Error */

    /* Note: Roll, Pitch and Yaw SetPoints and measures are in angles but the model of the drone is in radians, so we need to convert them to radians */
    if( obj->cfg.tag != z ) {

        obj->error_buffer[ 1 ] *= ( M_PI / 180.0f );
    }

    if( obj->gain.kp != 0 ) {

        obj->action.p = obj->error_buffer[ N ];                   /* Update Proportional value */
    }




    /**
     * *********************************************************************************
     * 
     *         Controller Discretization with Bilinear ( Tustin ) Approximation
     * 
     * *********************************************************************************
     */

    /** Integral Discretization:
     * 
     *                         Ts
     * y[ n ] = y[ n - 1 ]  + ---- * ( x[ n ] + x[ n - 1 ] ) = a + ( b * c ) where,
     *                         2
     * x is the error and
     * y is the integral value
     */

    if( obj->gain.ki != 0 ) {

        obj->action.i_buffer[ N_1 ] = obj->action.i_buffer[ N ];                /* Update last Integral value */

        float ai = obj->action.i_buffer[ N_1 ];
        float bi = obj->cfg.ts / 2;
        float ci = obj->error_buffer[ N ] + obj->error_buffer[ N_1 ];

        obj->action.i_buffer[ N ] = ai + ( bi * ci );                           /* Update new Integral value */
    }
    
    


    /**
     * Derivative Discretization:
     * 
     *            [ 2 * w0 * ( x[ n ] - x[ n - 1 ] ) ] + [ ( 2 - ( w0 * Ts ) ) * y[ n - 1 ] ]    a + b
     * y[ n ] = ----------------------------------------------------------------------------- = ------- where,
     *                                        2 + ( w0 * Ts )                                      c
     * 
     * If PI-D control, then the input to D action is te state nor the error
     * However, if PID control, the input to D action is as usual, the error
     * 
     */

    if( obj->gain.kd != 0 ) {

        obj->action.d_buffer[ N_1 ] = obj->action.d_buffer[ N ];                      /* Update last Derivative value */

        float ad = 2 * w0 * ( obj->error_buffer[ N ] - obj->error_buffer[ N_1 ] );
        float bd = ( 2 - ( w0 * obj->cfg.ts ) ) * obj->action.d_buffer[ N_1 ];
        float cd = 2 + ( w0 * obj->cfg.ts );

        obj->action.d_buffer[ N ] = ( ad + bd ) / cd;                                  /* Update new Derivative value */
    }

    float c_p = 0;
    float c_i = 0;
    float c_d = 0;

    /**
     * ***********************************
     * 
     *         Proportional Action
     * 
     * ***********************************
     */
    
    if( obj->gain.kp != 0 ) {

        c_p = obj->action.p * obj->gain.kp;
    }

    /**
     * ***********************************
     * 
     *         Integral Action
     * 
     * ***********************************
     */
    
    if( obj->gain.ki != 0 ) {

        if( obj->cfg.sat == anti_windup ) {

            if( ( obj->error_buffer[ N ] < 0 ) && ( obj->action.i_buffer[ N ] > 0 ) ) {

                c_i = 0;
            }

            else {

                c_i = obj->action.i_buffer[ N ] * obj->gain.ki;
            }
        }

        else if( obj->cfg.sat == back_propagation ) {

            /* Back-Propagation Algorithm */
        }

        else {

            c_i = obj->action.i_buffer[ N ] * obj->gain.ki;
        }
    }

    /**
     * ***********************************
     * 
     *         Derivative Action
     * 
     * ***********************************
     */

    if( obj->gain.kd != 0 ) {

        c_d = obj->action.d_buffer[ N ] * obj->gain.kd;
    }

    return c_p + c_i + c_d;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize Pid object
 * @param obj: Address of Pid object
 * @param cfg: Controller configs
 * @retval none
 */
static void pid_init( pid_controller_t * obj, ControllerCfgs_t cfg ) {
    ESP_LOGI( CONTROLLER_TAG, "Initializing Pid object..." );
    
    /* Set configs */
    obj->cfg = cfg;

    /* Set gains */
    obj->gain = obj->cfg.gains;

    /* Pid object initialized */
    obj->init_ok = true;

    ESP_LOGI( CONTROLLER_TAG, "Pid object initialized" );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


pid_controller_t * Pid( void ) {

    ESP_LOGI( CONTROLLER_TAG, "Making an instance of Pid Class..." );

    /* Assign memmory to Pid object */
    pid_controller_t * controller = malloc( sizeof( pid_controller_t ) );

    /* Default values */
    controller->gain.kp            = 0;
    controller->gain.ki            = 0;
    controller->gain.kd            = 0;
    controller->cfg.fc             = 0;
    controller->cfg.ts             = 0;
    controller->cfg.sat            = no_saturation;
    controller->action.p           = 0;
    controller->init_ok            = false;

    /* Default values */
    for(int i = 0; i < ( ( sizeof( controller->action.i_buffer ) ) / ( sizeof( controller->action.i_buffer[ 0 ] ) ) ); i++) {

        controller->action.i_buffer[ i ] = 0;
    }

    for(int i = 0; i < ( ( sizeof( controller->action.d_buffer ) ) / ( sizeof( controller->action.d_buffer[ 0 ] ) ) ); i++) {

        controller->action.d_buffer[ i ] = 0;
    }

    for(int i = 0; i < ( ( sizeof( controller->error_buffer ) ) / ( sizeof( controller->error_buffer[ 0 ] ) ) ); i++) {

        controller->error_buffer[ i ] = 0;
    }

    /* Pointer assignment to Pid Class functions ( methods ) */
    controller->pid  = pid;
    controller->init = pid_init;

    ESP_LOGI( CONTROLLER_TAG, "Instance successfully made" );

    /* Return instance of Pid Class */
    return controller;
}
