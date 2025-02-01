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


/* @brief Upper and lower limit for mma output */
typedef struct limits {

    /* Lower limit, from 0 to 1 */
    float lower;

    /* Upper limit, from 0 to 1 */
    float upper;
    
} limits_t;


/* ------------------------------------------------------------------------------------------------------------------------------------------ */


/** @details Forward declaration to avoid warning in function pointers */
typedef struct mma mma_t;

/**
 * @brief Complete description of Mma Class
 */
typedef struct mma {
    /* [ A ] Mma object inputs ( controller action ) */
    float input[ 4 ];

    /* [ A ] Mma object outputs ( rpm ) */
    float output[ 4 ];

    /* Upper and lower limits of mma output */
    limits_t limit;

    /** @brief Initialize object of Mma Class @param obj: Mma object @param upper_limit: Upper limit of mma output @param lower_limit: Lower limit of mma output @retval none */
    void ( * init )( mma_t * obj, float upper_limit, float lower_limit );

    /** @brief Compute MMA algorithm and update object outputs @param obj: Address of Mma object @param min: Minimum rpm @param max: Maximum rpm @retval none */
    void ( * compute )( mma_t * obj, float min, float max );

} mma_t;


#endif
