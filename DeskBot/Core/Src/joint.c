/*
 * joint.c
 *
 *  Created on: Jun 8, 2025
 *      Author: cole
 */

#include "joint.h"

//HELPER FUNCTIONS:
void Read_Encoder(joint* joint, int dt_ms)
{

	int curr_count = __HAL_TIM_GET_COUNTER(joint->encoderHandle);
	int delta = curr_count - joint->previousEncoderCount;

	 // Handle 16-bit counter overflow/underflow
	if (delta > 32767) {
		delta -= 65536;
	}
	else if (delta < -32768) {
		delta += 65536;
	}
	if (!joint->encoder_init) {
		delta = 0;  // Avoid spike on first call
	    joint->encoder_init = 1;
	}

	joint->previousEncoderCount = curr_count;

	joint->actual_velocity = delta / dt_ms;
	joint->actual_position = joint->actual_position + delta;
}

//DEF FUNCTIONS
void Joint_Init(joint* joint, motor_t* p_mot, TIM_HandleTypeDef* encoderHandle,
		PIDController* pos_pid,PIDController* vel_pid)
{
	joint->position_pid = *pos_pid;
	joint->velocity_pid = *vel_pid;

	joint->desired_position = 0;
	joint->desired_velocity = 0;
	joint->actual_position = 0;
	joint->actual_velocity = 0;
	joint->control_output = 0;

	joint->p_mot = p_mot;
	joint->encoderHandle = encoderHandle;
	joint->enable = 1;

	joint->previousEncoderCount = 0;
	joint->encoder_init = 0;
}

void Joint_Update(joint* joint, int dt_ms)
{
	static int outer_loop_counter = 0;

	Read_Encoder(joint,dt_ms); //gets position and velocity

	if(!joint->enable){
		return;
	}
	//run outer loop every 10 calls
	if(outer_loop_counter == 0)
	{
		joint->desired_velocity = PID_Update(&joint->position_pid,
											joint->desired_position,
											joint->actual_position,10*dt_ms);
	}
	//inner loop
	joint->control_output = PID_Update(&joint->velocity_pid,
										joint->desired_velocity,
										joint->actual_velocity,dt_ms);
	Set_Duty(joint->p_mot,joint->control_output);

	outer_loop_counter++;
	if(outer_loop_counter >= 2){ //ratio of outer loop to inner loop control
		outer_loop_counter = 0;
	}
}

bool Joint_Home(joint* joint,LimitSwitch* limswitch, int direction, int speed)
{
	joint->enable = 0;
	// 1. Set the motor to move in a fixed direction (e.g., -1 or 1)
	Set_Duty(joint->p_mot, direction * speed);

	// 2. Continuously check the limit switch
	if (LimitSwitch_IsTriggered(limswitch)) {
		Set_Duty(joint->p_mot, 0);             // Stop motor
		joint->actual_position = 0;                 // Zero the software encoder position
		__HAL_TIM_SET_COUNTER(joint->encoderHandle, 0); // Reset hardware encoder
		joint->encoder_init = 0;             // Reset filtering state
		joint->desired_position = 0;
		joint->enable = 1;
		return true;  // Homed successfully
	}

	return false;  // Still homing
}





