#ifndef MMA_STRUCTS_H
#define MMA_STRUCTS_H


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief MMA inputs enum
 */
typedef enum mma_inputs {
    
    /* z input */
    C_z,
    
    /* roll input */
    C_Roll,
    
    /* pitch input */
    C_Pitch,
    
    /* yaw input */
    C_Yaw,

} mma_inputs_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief MMA outputs enum
 */
typedef enum mma_outputs {

    /* Output for pwm 1 */
    u1,

    /* Output for pwm 2 */
    u2,

    /* Output for pwm 3 */
    u3,

    /* Output for pwm 4 */
    u4,
    
} mma_outputs_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Forward declaration to avoid warning in function pointers */
typedef struct mma mma_t;

/**
 * @brief Complete description of Mma Class
 */
typedef struct mma {
    /* [ A ] Mma object inputs */
    float input[ 4 ];

    /* [ A ] Mma object outputs */
    float output[ 4 ];

    /** @brief [ M ] Compute MMA algorithm and update object outputs @param obj: Address of Mma Object @param dc_min: Minimum duty cycle @param dc_max: Maximum duty cycle @retval none */
    void ( *compute )( mma_t * obj, float dc_min, float dc_max );

} mma_t;


#endif
