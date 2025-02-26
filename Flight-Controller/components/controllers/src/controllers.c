#include <stdio.h>
#include <controllers.h>
#include <math.h>
#include <esp_system.h>
#include <esp_log.h>
#include <string.h>

const char * CONTROLLER_TAG = "CONTROLLER";

#define GET_FUNC_NAME( func ) #func


/* ------------------------------------------------------------------------------------------------------------------------------------------ */



    typedef struct FunctionsMap {

        ControllerFunction * ActionFunc;
        const char * FuncName;

    } FunctionsMap_t;

    FunctionsMap_t functionTable[] = {

        { .ActionFunc = P_Basic,    .FuncName = "P_Basic"},
        { .ActionFunc = I_Basic,    .FuncName = "I_Basic"},
        { .ActionFunc = I_BackCalc, .FuncName = "I_BackCalc"},
        { .ActionFunc = I_Clamping, .FuncName = "I_Clamping"},
        { .ActionFunc = D_Basic,    .FuncName = "D_Basic"},
        { .ActionFunc = D_LPF,      .FuncName = "D_LPF"},
        { .ActionFunc = NULL,       .FuncName = NULL },

    };


float pidUpdate( pid_controller_t * obj, float pv, float sp ) {

    /* Check if PID object is initialized */
    if( !obj->init_ok ) {

        ESP_LOGE( CONTROLLER_TAG, "Object is not initialized!" );
        esp_restart();
    }

    obj->error = sp - pv;   /* Update actual error */

    /* Convert angles to radians */
    if( obj->tag != Z ) {

        obj->error *= ( M_PI / 180.0f );
    }

    float pAction = obj->pFunc( obj, obj->error );  /* Compute Proportional Action */
    float iAction = obj->iFunc( obj, obj->error );  /* Compute Integral Action */
    float dAction = obj->dFunc( obj, obj->error );  /* Compute Derivative Action */


        for( int i = 0; functionTable[ i ].ActionFunc != NULL; i++ ) {

            if( functionTable[ i ].ActionFunc == obj->pFunc ) {

                printf( "P func: %s\r\n", functionTable[ i ].FuncName );

            } else if( functionTable[ i ].ActionFunc == obj->iFunc ) {

                printf( "I func: %s\r\n", functionTable[ i ].FuncName );

            } else if( functionTable[ i ].ActionFunc == obj->dFunc ) {

                printf( "D func: %s\r\n", functionTable[ i ].FuncName );
            }
        }


    float pid_out = pAction + iAction + dAction;    /* Compute PID output */

    /* Saturate PID output */
    if( pid_out > obj->pid_out_limits.max ) {

        pid_out = obj->pid_out_limits.max;
    } else if( pid_out < obj->pid_out_limits.min ) {

        pid_out = obj->pid_out_limits.min;
    }

    obj->prev_error = obj->error;

    return pid_out;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void PidSetActionP( pid_controller_t * obj, ControllerFunction * pFunc ) {

    obj->pFunc = pFunc;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void PidSetActionI( pid_controller_t * obj, ControllerFunction * iFunc ) {

    obj->iFunc = iFunc;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


void PidSetActionD( pid_controller_t * obj, ControllerFunction * dFunc ) {

    obj->dFunc = dFunc;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float P_Basic( pid_controller_t * obj, float error ) {

    return obj->gain.kp * error;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float I_Basic( pid_controller_t * obj, float error ) {

    obj->integrator += error * obj->ts_ms;

    return obj->gain.ki * obj->integrator;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float I_Clamping( pid_controller_t * obj, float error ) {

    bool int_update = true; /* Flag to decide if integral should be updated or not */

    /* Compute Proportional Action */
    float pAction = obj->pFunc( obj, error );

    /* Compute Derivative Action */
    float dAction = obj->dFunc( obj, error );

    /* Compute Integral Action without updating the integrator yet */
    float iAction = obj->gain.ki * ( obj->integrator + ( error * obj->ts_ms ) );

    /* Compute controller output */
    float u = pAction + iAction + dAction;

    /* Check for positive saturation */
    if( u > obj->pid_out_limits.max ) {

        u = obj->pid_out_limits.max;    /* Saturate output */

        /* Check error sign */
        if( error > 0 ) {

            int_update = false; /* Don't update integrator */
        }

    } else if( u < obj->pid_out_limits.min ) {  /* Check for negative saturation */
        
        u = obj->pid_out_limits.min;    /* Saturate output */

        /* Check error sign */
        if( error < 0 ) {

            int_update = false; /* Don't update integrator */
        }
    }

    /* Check if integrator must be updated */
    if( int_update ) {

        obj->integrator += error * obj->ts_ms;  /* Update integrator */
    }

    return obj->gain.ki * obj->integrator;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float I_BackCalc( pid_controller_t * obj, float error ) {

    float pAction = obj->pFunc( obj, error );
    float dAction = obj->dFunc( obj, error );

    /* Compute the unsaturated output */
    float iAction = obj->gain.ki * obj->integrator;
    float uUnsat = pAction + iAction + dAction;

    /* Saturate PID output */
    float uSat = uUnsat;
    if( uSat > obj->pid_out_limits.max ) {

        uSat = obj->pid_out_limits.max;
    } else if( uSat < obj->pid_out_limits.min ) {

        uSat = obj->pid_out_limits.min;
    }

    /* Saturate the error */
    float eSat = uSat - uUnsat;

    /* Update integrator */
    obj->integrator += ( error * obj->ts_ms ) + ( obj->gain.kb * eSat * obj->ts_ms );

    /* Additionally clamp integrator */
    if( obj->integrator > obj->integral_limits.max ) {

        obj->integrator = obj->integral_limits.max;
    } else if( obj->integrator < obj->integral_limits.min ) {

        obj->integrator = obj->integral_limits.min;
    }

    return obj->gain.ki * obj->integrator;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float D_Basic( pid_controller_t * obj, float error ) {

    float derivative = ( error - obj->prev_error ) / obj->ts_ms;

    return obj->gain.kd * derivative;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


float D_LPF( pid_controller_t * obj, float error ) {

    float derivative = ( error - obj->prev_error ) / obj->ts_ms;

    /* Low Pass Filter coefficient */
    float a = obj->ts_ms / ( obj->ts_ms + obj->derivative_lpf.tau_s );

    /* Filtered value = ( α * New input ) + [ ( 1 - α ) * Previous output ] */
    obj->derivative_lpf.out = ( a * derivative ) + ( ( 1 - a ) * obj->derivative_lpf.out );

    return obj->gain.kd * derivative;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize Pid object
 * @param obj: Address of Pid object
 * @param cfg: Controller configs
 * @retval none
 */
static void pid_init( pid_controller_t * obj, states_t tag, float ts_ms, pid_gain_t pid_gains, pid_limits_t integral_limits, pid_limits_t pid_limits ) {

    ESP_LOGI( CONTROLLER_TAG, "Initializing Pid object..." );

    /* Check if cut off frequency ensures numerical stability */
    /*
    if( obj->cfg.fc > ( 0.90f / ( M_PI * ( obj->cfg.ts / 1000.0f ) ) ) ) {

        obj->cfg.fc = 0.90f / ( M_PI * ( obj->cfg.ts / 1000.0f ) );
    }
    */

    obj->tag             = tag;
    obj->ts_ms           = ts_ms;
    obj->gain            = pid_gains;
    obj->integral_limits = integral_limits;
    obj->pid_out_limits  = pid_limits;

    obj->integrator = 0.0f;
    obj->error      = 0.0f;
    obj->prev_error = 0.0f;

    /* Pid object initialized */
    obj->init_ok = true;

    ESP_LOGI( CONTROLLER_TAG, "Pid object initialized" );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */

pid_controller_t * Pid( ControllerFunction * pFunc, ControllerFunction * iFunc, ControllerFunction * dFunc ) {

    ESP_LOGI( CONTROLLER_TAG, "Making an instance of Pid Class..." );

    /* Assign memmory to Pid object */
    pid_controller_t * controller = malloc( sizeof( pid_controller_t ) );

    /* Set default attributes values in 0 */
    memset( controller, 0, sizeof( * controller ) );

    /* Pointer assignment to Pid Class functions ( methods ) */
    controller->init          = pid_init;
    controller->pidUpdate     = pidUpdate;
    controller->PidSetActionP = PidSetActionP;
    controller->PidSetActionI = PidSetActionI;
    controller->PidSetActionD = PidSetActionD;
    
    controller->pFunc = pFunc;
    controller->iFunc = iFunc;
    controller->dFunc = dFunc;

    ESP_LOGI( CONTROLLER_TAG, "Instance successfully made" );

    /* Return instance of Pid Class */
    return controller;
}
