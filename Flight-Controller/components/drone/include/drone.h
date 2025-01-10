#ifndef DRONE_H
#define DRONE_H

#include <esp_err.h>
#include <drone_structs.h>


/**
 * @brief Get Drone Class configs
 * @param none
 * @retval drone_cfg_t
 */
drone_cfg_t GetDroneConfigs( void );

/**
 * @brief Make an instance of Drone Class
 * @param none
 * @retval Pointer to Drone object
 */
drone_t * Drone( void );

/**
 * @brief Sine wave for tunning PID controllers in radians
 * @param A: Sine wave amplitude
 * @param t: Time
 * @param w: Sine wave angular frequency in rad / s
 * @retval float
 */
float __sin( float A, float w, float t );

#endif
