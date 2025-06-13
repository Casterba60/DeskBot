/*
 * controller.h
 *
 *  Created on: May 31, 2025
 *      Author: cole
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

//PID Controller Struct
/**
 * @brief PID Controller structure.
 *
 * Stores the proportional, integral, and derivative gains as well as stateful
 * variables like the integral accumulator and previous error.
 * Includes clamping parameters for integral windup protection and output limits.
 */

typedef struct {
	float kp;                /**< Proportional gain */
	float ki;                /**< Integral gain */
	float kd;                /**< Derivative gain */
	int integral;            /**< Accumulated integral term */
	int integral_clamp;      /**< Maximum absolute value of integral term */
	int tolerable_error;     /**< Error threshold below which the integral resets */
	int prev_error;          /**< Error value from the previous update */
	int output;              /**< Current output value */
	int out_min;             /**< Minimum allowable output */
	int out_max;             /**< Maximum allowable output */

} PIDController;

/**
 * @brief Initializes a PID controller.
 *
 * Sets gains and limits for a PID controller and clears its internal state.
 *
 * @param pid Pointer to the PIDController struct.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param integral_clamp Maximum absolute value of the integral term.
 * @param tolerable_error Error threshold for resetting the integral term.
 * @param out_min Minimum output value.
 * @param out_max Maximum output value.
 */
void PID_Init(PIDController* pid, float kp, float ki, float kd, int integral_clamp,int tolerable_error, int out_min, int out_max);

/**
 * @brief Updates the PID controller and computes a new output.
 *
 * Applies PID control based on the current error between setpoint and measured value.
 *
 * @param pid Pointer to the PIDController struct.
 * @param setpoint Desired target value.
 * @param measured Current measured value.
 * @param dt Time delta since last update (in ms or ticks).
 * @return The computed control output (clamped to limits).
 */
int PID_Update(PIDController* pid, int setpoint, int measured, int dt);

/**
 * @brief Resets the PID controller's internal state.
 *
 * Clears the integral accumulator, previous error, and output.
 *
 * @param pid Pointer to the PIDController struct.
 */
void PID_Reset(PIDController* pid);

#endif /* SRC_CONTROLLER_H_ */
