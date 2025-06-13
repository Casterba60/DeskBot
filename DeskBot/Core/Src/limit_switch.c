/*
 * limit_switch.c
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */

#ifndef SRC_LIMIT_SWITCH_C_
#define SRC_LIMIT_SWITCH_C_

#include "limit_switch.h"

/**
 * @brief Initializes a LimitSwitch object with GPIO port and pin.
 *
 * This function assigns the hardware port and pin to the LimitSwitch structure.
 * It does not configure GPIO mode or pull resistors — those should be handled
 * in your HAL MX_GPIO_Init() function or equivalent.
 *
 * @param switch_obj Pointer to a LimitSwitch structure.
 * @param port GPIO port for the switch.
 * @param pin GPIO pin used by the switch.
 */
void LimitSwitch_Init(LimitSwitch *switch_obj, GPIO_TypeDef *port, uint16_t pin) {
    switch_obj->port = port;
    switch_obj->pin = pin;
}

/**
 * @brief Reads the GPIO pin and checks if the limit switch is triggered.
 *
 * This implementation assumes the switch is active-low (i.e., it pulls the line
 * LOW when pressed). Modify this function if your switch is active-high.
 *
 * @param switch_obj Pointer to a LimitSwitch structure.
 * @return 1 if triggered (pressed), 0 otherwise.
 */
uint8_t LimitSwitch_IsTriggered(LimitSwitch *switch_obj) {
    return HAL_GPIO_ReadPin(switch_obj->port, switch_obj->pin) == GPIO_PIN_RESET;
    // Use GPIO_PIN_SET if your switch is active-high instead
}

#endif
