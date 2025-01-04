#ifndef PWM_H
#define PWM_H

#include <pwm_structs.h>

/**
 * @brief Make an instance of Pwm Class
 * @param tag: Pwm ID
 * @retval Pointer to Pwm object
 */
pwm_t * Pwm( int tag );

#endif
