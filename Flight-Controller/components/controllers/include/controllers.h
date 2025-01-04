#ifndef QUADCOPTER_CONTROLLER
#define QUADCOPTER_CONTROLLER

#include <controllers_structs.h>

#define SP_DELTA_Z      0.01     /* Delta z in meters */
#define SP_DELTA_ROLL   0.01     /* Delta Roll in angles */
#define SP_DELTA_PITCH  0.01     /* Delta Pitch in angles */
#define SP_DELTA_YAW    0.01     /* Delta Yaw in angles */
#define MAX_Z_CM        100      /* Maximum SetPoint for z state */
#define MAX_ANGLE       5        /* Maximum angle rotation for Roll, Pitch and Yaw states */

/**
 * @brief Make an instance of Pid class
 * @param none
 * @retval Pointer Pid object
 */
pid_controller_t * Pid( void );

#endif
