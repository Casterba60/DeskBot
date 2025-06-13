/*
 * servo.h
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */

/* servo.h */
#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

/**
 * @brief Maximum number of servos supported.
 */
#define MAX_SERVOS 8

/**
 * @brief Initializes the servo PWM timer.
 *
 * Starts the base timer in interrupt mode for periodic PWM pulse generation.
 *
 * @param htim Pointer to the timer handle used for PWM timing.
 */
void servo_init_timer(TIM_HandleTypeDef* htim);

/**
 * @brief Adds a servo to the management system.
 *
 * Registers a GPIO pin for software PWM-based servo control.
 *
 * @param port GPIO port of the servo signal pin.
 * @param pin GPIO pin number (0–15).
 * @return Index of the added servo (0–MAX_SERVOS-1), or -1 if full.
 */
int8_t servo_add(GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief Sets the angle of a registered servo.
 *
 * Converts degrees to PWM pulse width and updates servo control state.
 *
 * @param index Index of the servo (as returned by `servo_add`).
 * @param angle_deg Angle in degrees (clipped to [0°, 180°]).
 */
void servo_set_angle(uint8_t index, float angle_deg);

/**
 * @brief Updates the PWM signal for all servos (called from timer interrupt).
 *
 * Should be called at a high frequency (e.g., every 5 µs) to manage software PWM.
 */
void servo_update_tick(void);

#endif /* INC_SERVO_H_ */
