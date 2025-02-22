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
 * @brief Map mma output to duty cycle
 * @param u: mma block output
 * @param dc_min: Minimum duty cycle
 * @param dc_max: Maximum duty cycle
 */
static float u2pwm(float u, float dc_min, float dc_max ) {

    float Gu = ( U_MAX - U_MIN ) / ( W_MAX );
    float u_n = u * Gu; /* Normalized controller action */

    float m = 0;
    float b = 0;

    m = ( dc_max - dc_min ) / ( U_MAX - U_MIN );
    b = dc_min - ( m * U_MIN );

    return saturate(( m * u_n ) + b, dc_min, dc_max);
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
>>>>>>> parent of d673ece (Se resolvio el conflicto de merge entre local de juani y dev):Flight-Controller/components/mma/mma.c
 * @brief Compute MMA algorithm and update object outputs
 * @param obj: Address of Mma object
 * @param dc_min: Minimum duty cycle accepted by the ESC
 * @param dc_max: Maximum duty cycle accepted by the ESC
 * @retval none
 */
static void compute_obj( mma_t * obj, float dc_min, float dc_max ) {
    
    obj->output[ U1 ] = u2pwm( obj->input[ C_Z ] + ( ( 0.5f ) * (   obj->input[ C_ROLL ] + obj->input[ C_PITCH ] + obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U2 ] = u2pwm( obj->input[ C_Z ] + ( ( 0.5f ) * ( - obj->input[ C_ROLL ] + obj->input[ C_PITCH ] - obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U3 ] = u2pwm( obj->input[ C_Z ] + ( ( 0.5f ) * ( - obj->input[ C_ROLL ] - obj->input[ C_PITCH ] + obj->input[ C_YAW ] ) ), dc_min, dc_max );
    obj->output[ U4 ] = u2pwm( obj->input[ C_Z ] + ( ( 0.5f ) * (   obj->input[ C_ROLL ] - obj->input[ C_PITCH ] - obj->input[ C_YAW ] ) ), dc_min, dc_max );
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


    /* ---------------------------------------------------------------------- */


    /* Check lower limit */


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
