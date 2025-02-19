#include <stdio.h>
#include <mma.h>
#include <esp_log.h>

#define U_MAX         1         /* Maximum controller value */
#define U_MIN         0         /* Minimum controller value */
#define W_MAX         850.43    /* Maximum angular velocity in rad/s */

const char * MMA_TAG = "MMA";


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Private fuction definition */

/**
 * @brief Saturate calculated duty cycle
 * @param dc: mma block output
 * @param dc_min: Minimum duty cycle
 * @param dc_max: Maximum duty cycle
 */
static float saturate( mma_t * obj, float dc, float dc_min, float dc_max ) {

#define UPPER_LIMIT dc_max * obj->limit.upper
#define LOWER_LIMIT dc_min * obj->limit.lower

    if( dc > UPPER_LIMIT )      /* Upper saturation */
        return UPPER_LIMIT;

    else if( dc < LOWER_LIMIT ) /* Lower saturation */
        return LOWER_LIMIT;
    
    else                        /* No saturation */
        return dc;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Map mma output to duty cycle
 * @param u: mma block output
 * @param dc_min: Minimum duty cycle
 * @param dc_max: Maximum duty cycle
 */
static float u2pwm( mma_t * obj, float u, float dc_min, float dc_max ) {

    float Gu = ( U_MAX - U_MIN ) / ( W_MAX );
    float u_n = u * Gu; /* Normalized controller action */

    float m = 0;
    float b = 0;

    m = ( dc_max - dc_min ) / ( U_MAX - U_MIN );
    b = dc_min - ( m * U_MIN );

    return saturate( obj, ( m * u_n ) + b, dc_min, dc_max );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
>>>>>>> parent of d673ece (Se resolvio el conflicto de merge entre local de juani y dev):Flight-Controller/components/mma/mma.c
 * @brief Compute MMA algorithm and update object outputs
 * @param obj: Address of Mma object
 * @param dc_min: Minimum duty cycle
 * @param dc_max: Maximum duty cycle
 * @retval none
 */
static void compute_obj( mma_t * obj, float dc_min, float dc_max ) {
    
    obj->output[ U1 ] = u2pwm( obj, obj->input[ C_Z ] + ( ( 0.5f ) * (   obj->input[ C_ROLL ] + obj->input[ C_PITCH ] + obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U2 ] = u2pwm( obj, obj->input[ C_Z ] + ( ( 0.5f ) * ( - obj->input[ C_ROLL ] + obj->input[ C_PITCH ] - obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U3 ] = u2pwm( obj, obj->input[ C_Z ] + ( ( 0.5f ) * ( - obj->input[ C_ROLL ] - obj->input[ C_PITCH ] + obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U4 ] = u2pwm( obj, obj->input[ C_Z ] + ( ( 0.5f ) * (   obj->input[ C_ROLL ] - obj->input[ C_PITCH ] - obj->input[ C_YAW ] ) ), dc_min, dc_max );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize object of Mma Class
 * @param obj: Mma object
 * @param upper_limit: Upper limit of mma output
 * @param lower_limit: Lower limit of mma output
 * @retval none
 */
static void mma_init( mma_t * obj, float upper_limit, float lower_limit ) {

    ESP_LOGI( MMA_TAG, "Initializing object of Mma Class..." );

    /* Check upper limit */

    /* If upper limit is positive value*/
    if( ( upper_limit >= 0.0f ) )  {

        obj->limit.upper = upper_limit;
    }

    /* If upper limit is off range  */
    else {

        /* Set default value */
        obj->limit.upper = 1047;
    }


    /* ---------------------------------------------------------------------- */


    /* Check lower limit */

    /* If lower limit is bigger than 0 and smaller than upper limit */
    if( ( lower_limit >= 0.0f ) && ( lower_limit < upper_limit ) ) {

        obj->limit.lower = lower_limit;
    }

    /* If lower limit is off range  */
    else {

        /* Set default value */
        obj->limit.lower = 0.0f;
    }

    ESP_LOGI( MMA_TAG, "Mma object successfully initialized" );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions definitions */

mma_t * Mma( void ) {

    ESP_LOGI( MMA_TAG, "Making an instance of Mma Class..." );

    /* Assign memmory for Mma object */
    mma_t * mma = ( mma_t * ) malloc( sizeof( mma_t ) );

    /* Default values for Mma Class attributes */
    for (int i = 0; i < ( ( sizeof( mma->input ) )  / ( sizeof( mma->input[ 0 ] ) ) ); i++) { mma->input[ i ]   = 0; }
    for (int i = 0; i < ( ( sizeof( mma->output ) ) / ( sizeof( mma->output[ 0 ] ) ) ); i++) { mma->output[ i ] = 0; }
    
    /* Upper and lower limits default values */
    mma->limit.upper = 0.0f;
    mma->limit.lower = 0.0f;

    /* Pointer assignment to Mma Class functions ( methods ) */
    mma->init    = mma_init;
    mma->compute = compute_obj;

    ESP_LOGI( MMA_TAG, "Instance successfully made" );

    /* Return instance of Mma Class */
    return mma;
}
