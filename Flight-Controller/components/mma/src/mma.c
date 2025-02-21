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
 * @param input: Value to saturate
 * @param min: Minimum value
 * @param max: Maximum value
 */
static float saturate(float input, float min, float max ) {

    if( input > max )      /* Upper saturation */
        return max;

    else if( input < min ) /* Lower saturation */
        return min;
    
    else                        /* No saturation */
        return input;
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Compute MMA algorithm and update object outputs
 * @param obj: Address of Mma object
 * @param min: Minimum rpm
 * @param max: Maximum rpm
 * @retval none
 */
static void compute_obj( mma_t * obj) {
    
    obj->output[ u1 ] = saturate(obj->input[ C_z ] + ( ( 0.5f ) * (   obj->input[ C_Roll ] + obj->input[ C_Pitch ] + obj->input[ C_Yaw ] ) ), obj->limit.lower, obj->limit.upper );
    obj->output[ u2 ] = saturate(obj->input[ C_z ] + ( ( 0.5f ) * ( - obj->input[ C_Roll ] + obj->input[ C_Pitch ] - obj->input[ C_Yaw ] ) ), obj->limit.lower, obj->limit.upper );
    obj->output[ u3 ] = saturate(obj->input[ C_z ] + ( ( 0.5f ) * ( - obj->input[ C_Roll ] - obj->input[ C_Pitch ] + obj->input[ C_Yaw ] ) ), obj->limit.lower, obj->limit.upper );
    obj->output[ u4 ] = saturate(obj->input[ C_z ] + ( ( 0.5f ) * (   obj->input[ C_Roll ] - obj->input[ C_Pitch ] - obj->input[ C_Yaw ] ) ), obj->limit.lower, obj->limit.upper );
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
