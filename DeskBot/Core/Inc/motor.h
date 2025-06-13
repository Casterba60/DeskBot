/*
 * motor.h
 *
 *  Created on: Apr 17, 2025
 *      Author: cole
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "stm32f4xx_hal.h"

/**
 * @brief Maximum PWM value based on timer auto-reload register (ARR).
 */
#define PWM_MAX_DUTY_CYCLE 3599  /**< Maximum PWM value based on timer auto-reload */

/**
 * @brief Motor control structure.
 *
 * Encapsulates the TIM handle and PWM output channels used to drive an H-bridge or similar
 * motor driver using two PWM outputs.
 */

typedef struct {
	TIM_HandleTypeDef *htim;   /**< Timer handle used for PWM generation */
	uint32_t channelA;         /**< PWM output channel A */
	uint32_t channelB;         /**< PWM output channel B */
	int32_t dutyCycle;         /**< Signed duty cycle (-100 to 100), where 0 is stop/coast */
} motor_t;

/**
 * @brief Initializes a motor object with timer and channel configuration.
 *
 * @param p_mot Pointer to the motor object to initialize.
 * @param htim Pointer to the timer used for PWM control.
 * @param channelA TIM channel used for forward direction (e.g., TIM_CHANNEL_1).
 * @param channelB TIM channel used for reverse direction (e.g., TIM_CHANNEL_2).
 */
void Motor_Init(motor_t* p_mot,TIM_HandleTypeDef *htim, uint32_t channelA, uint32_t channelB);

/**
 * @brief Sets the motor to coast (no active drive).
 *
 * Sets both PWM outputs to 0, effectively putting the motor in a high-impedance state.
 *
 * @param p_mot Pointer to the motor object.
 */
void Coast(motor_t* p_mot);

/**
 * @brief Sets the motor speed and direction using a signed percentage.
 *
 * @param p_mot Pointer to the motor object.
 * @param speed Signed speed percentage from -100 to 100.
 *              Positive values drive forward; negative drive reverse; 0 coasts.
 */
void Set_Duty(motor_t* p_mot, int32_t speed);

#endif /* SRC_MOTOR_H_ */
