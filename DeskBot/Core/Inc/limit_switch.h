/*
 * limit_switch.h
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */

#ifndef INC_LIMIT_SWITCH_H
#define INC_LIMIT_SWITCH_H

#include "stm32f4xx_hal.h"  // Adjust this include as needed for your STM32 family

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} LimitSwitch;

/**
 * @brief Initialize a limit switch.
 * @param switch_obj Pointer to a LimitSwitch struct.
 * @param port GPIO port (e.g., GPIOA, GPIOB).
 * @param pin GPIO pin number (e.g., GPIO_PIN_0).
 */
void LimitSwitch_Init(LimitSwitch *switch_obj, GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief Check if the limit switch is triggered (pressed/activated).
 * @param switch_obj Pointer to a LimitSwitch struct.
 * @retval 1 if triggered, 0 otherwise.
 */
uint8_t LimitSwitch_IsTriggered(LimitSwitch *switch_obj);

#endif // LIMIT_SWITCH_H
