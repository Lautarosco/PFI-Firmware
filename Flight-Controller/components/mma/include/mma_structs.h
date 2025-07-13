#ifndef MMA_STRUCTS_H
#define MMA_STRUCTS_H


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief MMA inputs enum
 */
typedef enum mma_inputs {
    
    /* z input of mma block */
    C_Z,
    
    /* roll input of mma block */
    C_ROLL,
    
    /* pitch input of mma block */
    C_PITCH,
    
    /* yaw input of mma block */
    C_YAW,

} mma_inputs_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/**
 * @brief MMA outputs enum
 */
typedef enum mma_outputs {

    /* Output of mma block for pwm 1 */
    U1,

    /* Output of mma block for pwm 2 */
    U2,

    /* Output of mma block for pwm 3 */
    U3,

    /* Output of mma block for pwm 4 */
    U4,
    
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

    /** @brief Initialize object of Mma Class @param mma: Mma object */
    void ( * init )( mma_t * mma);

    /** @brief [ M ] Compute MMA algorithm and update object outputs @param mma: Address of Mma Object @param dc_min: Minimum duty cycle @param dc_max: Maximum duty cycle @retval none */
    void ( * compute )( mma_t * mma, float dc_min, float dc_max );

} mma_t;


#endif
