/*
 * motor.c
 *
 *  Created on: Apr 17, 2025
 *      Author: cole
 */
#include "motor.h"

/**
 * @brief Initializes a motor control object.
 *
 * Associates the motor object with a given timer and two output channels.
 * The initial duty cycle is set to 0 (coasting).
 *
 * @param p_mot Pointer to the motor structure to initialize.
 * @param htim Pointer to the timer used for PWM.
 * @param channelA Output channel A (forward).
 * @param channelB Output channel B (reverse).
 */
void Motor_Init(motor_t* p_mot,TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB){
	p_mot->htim = htim;
	p_mot->channelA = channelA;
	p_mot->channelB = channelB;
	p_mot->dutyCycle = 0;
}

/**
 * @brief Disengages both motor outputs to coast.
 *
 * Sets both PWM outputs to 0, letting the motor coast freely.
 *
 * @param p_mot Pointer to the motor object.
 */
void Coast(motor_t* p_mot){
	__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,0);
	__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,0);
}

/**
 * @brief Sets motor output based on signed speed.
 *
 * Converts the speed percentage (−100 to +100) to a PWM duty cycle. The PWM resolution
 * is defined by PWM_MAX_DUTY_CYCLE + 1. The scaling maps 100% to maximum PWM value.
 *
 *
 * Special case:
 * - speed == 0: Sets both outputs to max duty, simulates full brake.
 * - speed > 0: Drives forward using channel A.
 * - speed < 0: Drives in reverse using channel B.
 *
 * @param p_mot Pointer to the motor object.
 * @param speed Signed percentage (-100 to 100). 0 = brake, >0 = forward, <0 = reverse.
 */
void Set_Duty(motor_t* p_mot,int32_t speed) { //100% is 399
	if(!speed){
		p_mot->dutyCycle = 3599;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,PWM_MAX_DUTY_CYCLE);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,PWM_MAX_DUTY_CYCLE);
	}
	else if(speed > 0 && speed <= 100) {
		p_mot->dutyCycle = ((PWM_MAX_DUTY_CYCLE + 1) * speed) / 100 - 1;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,p_mot->dutyCycle);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,0);
	}
	else if (speed < 0 && speed >= -100) {
		p_mot->dutyCycle = ((PWM_MAX_DUTY_CYCLE + 1) * -speed) / 100 - 1;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,0);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,p_mot->dutyCycle);
	}
}







