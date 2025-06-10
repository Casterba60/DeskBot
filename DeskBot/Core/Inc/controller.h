/*
 * controller.h
 *
 *  Created on: May 31, 2025
 *      Author: cole
 */

#ifndef SRC_CONTROLLER_H_
#define SRC_CONTROLLER_H_

//PID Controller Struct
typedef struct {
	float kp;
	float ki;
	float kd;
	int integral;
	int integral_clamp;
	int tolerable_error;
	int prev_error;
	int output;
	int out_min;
	int out_max;

} PIDController;

//Functions
void PID_Init(PIDController* pid, float kp, float ki, float kd, int integral_clamp,int tolerable_error, int out_min, int out_max);
int PID_Update(PIDController* pid, int setpoint, int measured, int dt);
void PID_Reset(PIDController* pid);

#endif /* SRC_CONTROLLER_H_ */
