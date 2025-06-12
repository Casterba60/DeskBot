/*
 * joint.h
 *
 *  Created on: Jun 8, 2025
 *      Author: cole
 */

#ifndef INC_JOINT_H_
#define INC_JOINT_H_

#include "controller.h" //pid controller
#include "motor.h"
#include "limit_switch.h"
#include <stdbool.h>

typedef struct {
	PIDController position_pid;
	PIDController velocity_pid;

	int desired_position; //input command (encoder ticks)
	int desired_velocity; //Output of position

	int actual_position; //Encoder reading
	int actual_velocity; //Measured velocity (change in enc / dt)

	int control_output; // Final output to motor (0 - 100)
	//motor and encoder
	motor_t* p_mot; //pointer to motor struct
	TIM_HandleTypeDef* encoderHandle; //pointer to encoder handle

	int previousEncoderCount;
	int encoder_init;

	int enable;
} joint;

void Joint_Init(joint* joint, motor_t* p_mot, TIM_HandleTypeDef* encoderHandle,
		PIDController* pos_pid,PIDController* vel_pid);
void Joint_Update(joint* joint, int dt_ms);

bool Joint_Home(joint* joint,LimitSwitch* limswitch,int direction, int speed);

#endif /* INC_JOINT_H_ */
