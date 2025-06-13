/*
 * limit_switch.h
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */

#ifndef INC_LIMIT_SWITCH_H
#define INC_LIMIT_SWITCH_H

#include "stm32f4xx_hal.h"

/**
 * @brief Struct representing a digital limit switch input.
 */
typedef struct {
	GPIO_TypeDef *port;  /**< GPIO port of the limit switch */
	uint16_t pin;        /**< GPIO pin number */
} LimitSwitch;

/**
 * @brief Initializes a limit switch object with a specified GPIO port and pin.
 *
 * Note: This function does not configure the GPIO as input. You must do that in
 * MX_GPIO_Init() or equivalent HAL setup.
 *
 * @param switch_obj Pointer to a LimitSwitch struct.
 * @param port GPIO port (e.g., GPIOA, GPIOB).
 * @param pin GPIO pin number (e.g., GPIO_PIN_0).
 */
void LimitSwitch_Init(LimitSwitch *switch_obj, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief Checks whether the limit switch is currently triggered (pressed).
 *
 * Assumes an active-low switch (triggered when pin is pulled LOW). Modify if your
 * hardware uses active-high logic.
 *
 * @param switch_obj Pointer to a LimitSwitch struct.
 * @retval 1 if triggered (pressed), 0 otherwise.
 */
uint8_t LimitSwitch_IsTriggered(LimitSwitch *switch_obj);

#endif // LIMIT_SWITCH_H
