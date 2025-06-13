/*
 * controller.c
 *
 *  Created on: May 31, 2025
 *      Author: cole
 */
#include "controller.h"
#include <stdlib.h>

/**
 * @brief Initializes a PID controller structure with provided parameters.
 *
 * Sets the PID gains and clamps, and resets all dynamic state variables.
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
void PID_Init(PIDController* pid, float kp, float ki, float kd, int integral_clamp, int tolerable_error, int out_min, int out_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0;
	pid->prev_error = 0;
	pid->output = 0;
	pid->integral_clamp = integral_clamp;
	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->tolerable_error = tolerable_error;
}

/**
 * @brief Executes one update step of the PID controller.
 *
 * Computes the control output based on the current setpoint and measured value.
 * Includes anti-windup for the integral term and output clamping.
 *
 * @param pid Pointer to the PIDController struct.
 * @param setpoint The desired target value.
 * @param measured The current measured value.
 * @param dt Time elapsed since the last update (e.g., in ms).
 * @return The clamped PID output value.
 */
int PID_Update(PIDController* pid, int setpoint, int measured, int dt)
{
	int error = setpoint - measured;

	pid->integral += error*dt/40; //scale integral error down
	//clamp integral
	if (pid->integral > pid->integral_clamp)
	{
		pid->integral = pid->integral_clamp;
	}
	else if  (pid->integral < -1*pid->integral_clamp)
	{
		pid->integral = -1*pid->integral_clamp;
	}

	if(abs(error) < pid->tolerable_error)
	{
		pid->integral = 0;
	}

	int derivative = (error - pid->prev_error)/dt;

	pid->output = pid->kp*error + pid->ki*pid->integral + pid->kd*derivative;

	// Clamp output
	if (pid->output > pid->out_max)
	{
		pid->output = pid->out_max;
	}
	else if (pid->output < pid->out_min)
	{
		pid->output = pid->out_min;
	}

	pid->prev_error = error;
	return pid->output;
}

/**
 * @brief Resets the PID controller's dynamic state.
 *
 * Clears the integral accumulator, previous error, and output value.
 *
 * @param pid Pointer to the PIDController struct.
 */
void PID_Reset(PIDController* pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}


