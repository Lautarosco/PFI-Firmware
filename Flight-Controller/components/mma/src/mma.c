#include <stdio.h>
#include <string.h>
#include <mma.h>
#include <esp_log.h>

#define U_MAX         100   /* Maximum controller value */
#define U_MIN         -100  /* Minimum controller value */
#define W_MAX         1047  /* Step 10 of prop. cal. in rad/s */
#define W_MIN         240   /* Step 2 of prop. cal. in rad/s */
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
 * @return float duty cycle
 */
static float u2pwm(float u, float dc_min, float dc_max ) {

    //float Gu = ( U_MAX - U_MIN ) / ( W_MAX - W_MIN );
    //float u_n = u * Gu; /* Normalized controller action */

    float m = ( dc_max - dc_min ) / ( U_MAX - U_MIN );
    float b = dc_min - ( m * U_MIN );

    return saturate(( m * u ) + b, dc_min, dc_max);
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Compute MMA algorithm and update object outputs
 * @param mma: Address of Mma object
 * @param dc_min: Minimum duty cycle accepted by the ESC
 * @param dc_max: Maximum duty cycle accepted by the ESC
 * @retval none
 */
static void compute_obj( mma_t * mma, float dc_min, float dc_max ) {
    
    mma->output[ U1 ] = u2pwm( mma->input[ C_Z ] + ( ( 0.5f ) * (   mma->input[ C_ROLL ] + mma->input[ C_PITCH ] + mma->input[ C_YAW ] ) ), dc_min, dc_max );
    mma->output[ U2 ] = u2pwm( mma->input[ C_Z ] + ( ( 0.5f ) * ( - mma->input[ C_ROLL ] + mma->input[ C_PITCH ] - mma->input[ C_YAW ] ) ), dc_min, dc_max );
    mma->output[ U3 ] = u2pwm( mma->input[ C_Z ] + ( ( 0.5f ) * ( - mma->input[ C_ROLL ] - mma->input[ C_PITCH ] + mma->input[ C_YAW ] ) ), dc_min, dc_max );
    mma->output[ U4 ] = u2pwm( mma->input[ C_Z ] + ( ( 0.5f ) * (   mma->input[ C_ROLL ] - mma->input[ C_PITCH ] - mma->input[ C_YAW ] ) ), dc_min, dc_max );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief Initialize object of Mma Class
 * @param mma: Mma object
 * @param upper_limit: Upper limit of mma output
 * @param lower_limit: Lower limit of mma output
 * @retval none
 */
static void mma_init( mma_t * mma ) {

    ESP_LOGI( MMA_TAG, "Initializing object of Mma Class..." );
    ESP_LOGI( MMA_TAG, "Mma object successfully initialized" );
}


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Public functions definitions */

void Mma( mma_t* mma ) {

    ESP_LOGI( MMA_TAG, "Making an instance of Mma Class..." );
    memset( mma, 0, sizeof( mma_t ) );

    /* Pointer assignment to Mma Class functions ( methods ) */
    mma->init    = mma_init;
    mma->compute = compute_obj;

    ESP_LOGI( MMA_TAG, "Instance successfully made" );

}
