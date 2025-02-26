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
pid_controller_t * Pid( ControllerFunction* pFunc, ControllerFunction* iFunc, ControllerFunction* dFunc );

/**
 * @brief Proportional action basic algorithm
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval Controller action output
 */
float P_Basic( pid_controller_t * obj, float error );

/**
 * @brief Integral action basic algorithm
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval Controller action output
 */
float I_Basic( pid_controller_t * obj, float error );

/**
 * @brief Integral action with Back Calculation ( feedback ) algorithm
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval Controller action output
 */
float I_BackCalc( pid_controller_t * obj, float error );

/**
 * @brief Derivative action basic algorithm
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval Controller action output
 */
float D_Basic( pid_controller_t * obj, float error );

#endif
