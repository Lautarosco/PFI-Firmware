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
static float saturate( float dc, float dc_min, float dc_max ) {
#define UPPER_LIMIT dc_max * ( 1 - 0.09f )
#define LOWER_LIMIT dc_min * ( 1 + 0.1f )

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
static float u2pwm( float u, float dc_min, float dc_max ) {
    float Gu = ( U_MAX - U_MIN ) / ( W_MAX );
    float u_n = u * Gu; /* Normalized controller action */

    float m = 0;
    float b = 0;

    m = ( dc_max - dc_min ) / ( U_MAX - U_MIN );
    b = dc_min - ( m * U_MIN );

    return saturate( ( m * u_n ) + b, dc_min, dc_max );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Compute MMA algorithm and update object outputs
 * @param obj: Address of Mma object
 * @param dc_min: Minimum duty cycle
 * @param dc_max: Maximum duty cycle
 * @retval none
 */
static void compute_obj( mma_t * obj, float dc_min, float dc_max ) {
    obj->output[ u1 ] = u2pwm( obj->input[ C_z ] + ( ( 0.5f ) * (   obj->input[ C_Roll ] + obj->input[ C_Pitch ] + obj->input[ C_Yaw ] ) ), dc_min, dc_max );
    obj->output[ u2 ] = u2pwm( obj->input[ C_z ] + ( ( 0.5f ) * ( - obj->input[ C_Roll ] + obj->input[ C_Pitch ] - obj->input[ C_Yaw ] ) ), dc_min, dc_max );
    obj->output[ u3 ] = u2pwm( obj->input[ C_z ] + ( ( 0.5f ) * ( - obj->input[ C_Roll ] - obj->input[ C_Pitch ] + obj->input[ C_Yaw ] ) ), dc_min, dc_max );
    obj->output[ u4 ] = u2pwm( obj->input[ C_z ] + ( ( 0.5f ) * (   obj->input[ C_Roll ] - obj->input[ C_Pitch ] - obj->input[ C_Yaw ] ) ), dc_min, dc_max );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions definitions */

mma_t * Mma( void ) {
    ESP_LOGI( MMA_TAG, "Making an instance of Mma Class..." );

    /* Assign memmory for Mma object */
    mma_t * mma = malloc( sizeof( mma_t ) );

    /* Default values for Mma Class attributes */
    for (int i = 0; i < ( ( sizeof( mma->input ) )  / ( sizeof( mma->input[ 0 ] ) ) ); i++) { mma->input[ i ]   = 0; }
    for (int i = 0; i < ( ( sizeof( mma->output ) ) / ( sizeof( mma->output[ 0 ] ) ) ); i++) { mma->output[ i ] = 0; }
    
    /* Pointer assignment to Mma Class functions ( methods ) */
    mma->compute = compute_obj;

    ESP_LOGI( MMA_TAG, "Instance successfully made" );

    /* Return instance of Mma Class */
    return mma;
}
