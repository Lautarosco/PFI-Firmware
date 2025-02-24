#ifndef DRONE_H
#define DRONE_H

#include <esp_err.h>
// #include <drone_structs.h>
#include <drone_configs.h>  /* This header file already includes drone_structs.h */


/**
 * @brief Make an instance of Drone Class
 * @param none
 * @retval Pointer to Drone object
 */
drone_t * Drone( void );

/**
 * @brief Sine wave for tunning PID controllers in radians
 * @param A: Sine wave amplitude
 * @param t: Time initialized in 0 ( the function handles time values )
 * @param w: Sine wave angular frequency in rad / s
 * @retval float
 */
float __sin( float A, float w, float dt_ms );

#endif
