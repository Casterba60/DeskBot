/*
 * motor.h
 *
 *  Created on: Apr 17, 2025
 *      Author: cole
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "stm32f4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channelA;
	uint32_t channelB;
	int32_t dutyCycle;
} motor_t;

void Motor_Init(motor_t* p_mot,TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB);

void Coast(motor_t* p_mot);

void Set_Duty(motor_t* p_mot, int32_t speed);

#endif /* SRC_MOTOR_H_ */
