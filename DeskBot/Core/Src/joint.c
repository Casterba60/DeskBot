/*
 * joint.c
 *
 *  Created on: Jun 8, 2025
 *      Author: cole
 */

#include "joint.h"

//HELPER FUNCTIONS:
int Read_Velocity(TIM_HandleTypeDef* htim, int dt_ms)
{
	static int prev_count = 0;
	static int initialized = 0;

	int curr_count = __HAL_TIM_GET_COUNTER(htim);
	int delta = curr_count - prev_count;

	 // Handle 16-bit counter overflow/underflow
	if (delta > 32767) {
		delta -= 65536;
	}
	else if (delta < -32768) {
		delta += 65536;
	}
	if (!initialized) {
		delta = 0;  // Avoid spike on first call
	    initialized = 1;
	}

	prev_count = curr_count;

	return delta / dt_ms;
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
}

void Joint_Update(joint* joint, int dt_ms)
{
	static int outer_loop_counter = 0;

	joint->actual_position = __HAL_TIM_GET_COUNTER(joint->encoderHandle);
	joint->actual_velocity = Read_Velocity(joint->encoderHandle,dt_ms);

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
	if(outer_loop_counter >= 10){
		outer_loop_counter = 0;
	}
}





