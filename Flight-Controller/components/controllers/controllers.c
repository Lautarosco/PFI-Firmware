#include <stdio.h>
#include <controllers.h>
#include <math.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

const char * CONTROLLER_TAG = "CONTROLLER";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Calculate Controller PID action
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
    
    float w = 2 * M_PI * obj->cfg.fc;   /* Calculate Low Pass Filter coefficient in rad/s */

    obj->error = sp - pv; /* Update error */

    /* Note: Roll, Pitch and Yaw SetPoints and measures are in angles but the model of the drone is in radians, so we need to convert them to radians */
    if( obj->cfg.tag != z ) {

        obj->error *= ( M_PI / 180.0f );
    }

    if( obj->gain.kp != 0 ) {

        obj->action.p = obj->error; /* Update Proportional value */
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
     *        _____
     *       |     |
     *   e   |  1  |   y
     * ----> | --- | ---->
     *       |  s  |
     *       |_____|
     * 
     * y = y + ( x * dt )
     * 
     */

    if( obj->gain.ki != 0 ) {

        obj->action.i += obj->error * obj->cfg.ts;
    }

    if( obj->gain.kd != 0 ) {

        /* Compute derivative with filter */
        if( obj->cfg.der_filter ) {
            
            /**
             * Derivative Discretization:
             * 
             *                 _____
             *   e  +         |     |       y'
             * ----->◯ ----> |  w  | ------------>
             *     - ^        |_____|        |
             *       |                       |
             *       |         _____         |
             *       |        |     |        |
             *       |        |  1  |        |
             *       |<-------| --- |<-------|
             *           y    |  s  |
             *                |_____|
             * 
             * ( 1 ) y' = w * ( e - y ) => Update out of the algorithm
             * ( 2 ) y = sum + ( y' * dt ) => Update [ y is the integral of y' ]
             * 
             */

            obj->action.d.out = w * ( obj->error - obj->action.d.sum ); /* ( 1 ) */
            obj->action.d.sum += obj->action.d.out * obj->cfg.ts; /* ( 2 ) */
        }

        /* Compute derivative without filter */
        else {


        }
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
    
    if( ( obj->gain.ki != 0 ) && ( fabs( obj->error ) > obj->cfg.intMinErr ) ) {

        if( obj->cfg.sat == anti_windup ) {

            if( ( obj->error < 0 ) && ( obj->action.i > 0 ) ) {

                c_i = 0;
            }

            else {

                c_i = obj->action.i * obj->gain.ki;
            }
        }

        else if( obj->cfg.sat == back_propagation ) {

            /* Back-Propagation Algorithm */
        }

        else {

            c_i = obj->action.i * obj->gain.ki;
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

        c_d = obj->action.d.out * obj->gain.kd;
    }

    return c_p + c_i + c_d;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Calculate Controller PI-D action
 * @param obj: Address of Pid object
 * @param pv: Process value
 * @param sp: Set point
 * @retval PID calculation
 */
static float manual_pi_d( pid_controller_t * obj, float pv, float sp, float d_state ) {

    if( !obj->init_ok ) {

        ESP_LOGE( CONTROLLER_TAG, "Object is not initialized!" );
        esp_restart();
    }
    
    float w = 2 * M_PI * obj->cfg.fc;   /* Calculate Low Pass Filter coefficient in rad/s */

    obj->error = sp - pv; /* Update error */

    /* Note: Roll, Pitch and Yaw SetPoints and measures are in angles but the model of the drone is in radians, so we need to convert them to radians */
    if( obj->cfg.tag != z ) {

        obj->error *= ( M_PI / 180.0f );
    }

    if( obj->gain.kp != 0 ) {

        obj->action.p = obj->error; /* Update Proportional value */
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
     *        _____
     *       |     |
     *   e   |  1  |   y
     * ----> | --- | ---->
     *       |  s  |
     *       |_____|
     * 
     * y = y + ( x * dt )
     * 
     */

    if( obj->gain.ki != 0 ) {

        obj->action.i += obj->error * obj->cfg.ts;
    }

    /* Derivative Action */
    if( obj->gain.kd != 0 ) {

        obj->action.d.out = d_state;
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
    
    if( ( obj->gain.ki != 0 ) && ( fabs( obj->error ) > obj->cfg.intMinErr ) ) {

        if( obj->cfg.sat == anti_windup ) {

            if( ( obj->error < 0 ) && ( obj->action.i > 0 ) ) {

                c_i = 0;
            }

            else {

                c_i = obj->action.i * obj->gain.ki;
            }
        }

        else if( obj->cfg.sat == back_propagation ) {

            /* Back-Propagation Algorithm */
        }

        else {

            c_i = obj->action.i * obj->gain.ki;
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

        c_d = obj->action.d.out * obj->gain.kd;
    }

    return c_p + c_i - c_d;
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

    /* Check if cut off frequency ensures numerical stability */
    if( obj->cfg.fc > ( 0.90f / ( M_PI * ( obj->cfg.ts / 1000.0f ) ) ) ) {

        obj->cfg.fc = 0.90f / ( M_PI * ( obj->cfg.ts / 1000.0f ) );
    }

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

    /* Set default attributes values in 0 */
    memset( controller, 0, sizeof( * controller ) );

    /* Pointer assignment to Pid Class functions ( methods ) */
    controller->pid         = pid;
    controller->init        = pid_init;
    controller->manual_pi_d = manual_pi_d;

    ESP_LOGI( CONTROLLER_TAG, "Instance successfully made" );

    /* Return instance of Pid Class */
    return controller;
}
