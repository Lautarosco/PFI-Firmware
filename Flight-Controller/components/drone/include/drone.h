#pragma once

#include <drone_configs.h>

#define IGNORE_GPS


/**
 * @brief Make an instance of Drone Class
 * @param none
 * @retval Pointer to Drone object
 */
void Drone( drone_t * drone );

/**
 * @brief Compute first order IIR filter algorithm
 * @param in: New input to filter
 * @param out: Previous output of filter 
 * @param ts_s: Sampling time in seconds
 * @param alpha: Filter coefficient (0 - 1)
 * @return float
 */
float FirstOrderIIR( float in, float out, float ts_s, float alpha );
