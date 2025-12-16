#include <stdio.h>
#include <string.h>
#include <mma.h>
#include <esp_log.h>
#include <drone.h>

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
static float u2pwm( mma_t * mma, float u_z, float u, float dc_min, float dc_max ) {
    #define MAX_DC_UZ 0.0740
    #define RANGE_DC_U 0.0168

    // float base_dc = dc_min + (MAX_DC_UZ - dc_min) * (u_z / 100.0f);
    float base_dc = 0.071;
    // float base_dc = 0.071 + (0.01)*(u_z/100.0f);
    float correction = (u / 100.0f) * (RANGE_DC_U / 2.0f);

    float final_dc = base_dc + correction;

    #ifndef container_of
    #define container_of(ptr, type, member) \
        ((type *)((char *)(ptr) - offsetof(type, member)))
    #endif
    
    return saturate(final_dc, dc_min, dc_max);
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
    float base_thrust = mma->input[C_Z];
    
    float roll_correction = mma->input[C_ROLL];
    float pitch_correction = mma->input[C_PITCH]; 
    float yaw_correction = mma->input[C_YAW];
    
    mma->output[U1] = u2pwm(mma, base_thrust, (roll_correction + pitch_correction + yaw_correction), dc_min, dc_max);
    mma->output[U2] = u2pwm(mma, base_thrust, (-roll_correction + pitch_correction - yaw_correction), dc_min, dc_max);
    mma->output[U3] = u2pwm(mma, base_thrust, (-roll_correction - pitch_correction + yaw_correction), dc_min, dc_max);
    mma->output[U4] = u2pwm(mma, base_thrust, (roll_correction - pitch_correction - yaw_correction), dc_min, dc_max);
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
