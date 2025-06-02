/*
 * motor.c
 *
 *  Created on: Apr 17, 2025
 *      Author: cole
 */
#include "motor.h"

void Motor_Init(motor_t* p_mot,TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB){
	p_mot->htim = htim;
	p_mot->channelA = channelA;
	p_mot->channelB = channelB;
	p_mot->dutyCycle = 0;
}

void Coast(motor_t* p_mot){
	__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,0);
	__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,0);
}

void Set_Duty(motor_t* p_mot,int32_t speed) { //100% is 4799
	if(!speed){
		p_mot->dutyCycle = 4799;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,4799);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,4799);
	}
	else if(speed > 0 && speed <= 100) {
		p_mot->dutyCycle = 48*speed-1;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,48*speed-1);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,0);
	}
	else if (speed < 0 && speed >= -100) {
		p_mot->dutyCycle = -48*speed-1;
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelA,0);
		__HAL_TIM_SET_COMPARE(p_mot->htim,p_mot->channelB,-48*speed-1);
	}
}







