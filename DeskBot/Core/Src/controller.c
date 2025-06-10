/*
 * controller.c
 *
 *  Created on: May 31, 2025
 *      Author: cole
 */
#include "controller.h"

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

void PID_Reset(PIDController* pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output = 0;
}


