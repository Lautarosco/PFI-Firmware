#ifndef QUADCOPTER_CONTROLLER
#define QUADCOPTER_CONTROLLER

#include <controllers_structs.h>

/**
 * @brief Make an instance of Pid class
 * @param none
 * @retval Pointer Pid object
 */
pid_controller_t * Pid( ControllerFunction* pFunc, ControllerFunction* iFunc, ControllerFunction* dFunc );


/**
 * @brief Basic Proportional Action Function
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Basic Proportional Action Function
 */
float P_Basic( pid_controller_t * obj, float error );

/**
 * @brief Basic Integral Action Function
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Basic Integral Action Function
 */
float I_Basic( pid_controller_t * obj, float error );

/**
 * @brief Integral Action Function with Back calculation
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Integral Action Function with Back calculation
 */
float I_BackCalc( pid_controller_t * obj, float error );

/**
 * @brief Integral Action Function with Conditional Clamping
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Integral Action Function with Conditional Clamping
 */
float I_Clamping( pid_controller_t * obj, float error );

/**
 * @brief Basic Derivative Action Function
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Basic Derivative Action Function
 */
float D_Basic( pid_controller_t * obj, float error );

/**
 * @brief Derivative Action Function with Low Pass Filter
 * @param obj: Address of Pid object
 * @param error: Error
 * @retval result of Derivative Action Function with Low pass Filter
 */
float D_LPF( pid_controller_t * obj, float error );


/**
 * @brief Set controller Proportional action
 * @param obj: Address of Pid object
 * @param pFunc: Controller action related function
 * @retval none
 */
void PidSetActionP( pid_controller_t * obj, ControllerFunction * pFunc );

/**
 * @brief Set controller Integral action
 * @param obj: Address of Pid object
 * @param iFunc: Controller action related function
 * @retval none
 */
void PidSetActionI( pid_controller_t * obj, ControllerFunction * iFunc );

/**
 * @brief Set controller Derivative action
 * @param obj: Address of Pid object
 * @param dFunc: Controller action related function
 * @retval none
 */
void PidSetActionD( pid_controller_t * obj, ControllerFunction * dFunc );

#endif
