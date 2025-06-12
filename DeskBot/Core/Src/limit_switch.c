/*
 * limit_switch.c
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */

#ifndef SRC_LIMIT_SWITCH_C_
#define SRC_LIMIT_SWITCH_C_

#include "limit_switch.h"

void LimitSwitch_Init(LimitSwitch *switch_obj, GPIO_TypeDef *port, uint16_t pin) {
    switch_obj->port = port;
    switch_obj->pin = pin;

    // Assume GPIO is already configured elsewhere in HAL_Init or MX_GPIO_Init
    // Otherwise, you would configure GPIO as input here.
}

uint8_t LimitSwitch_IsTriggered(LimitSwitch *switch_obj) {
    return HAL_GPIO_ReadPin(switch_obj->port, switch_obj->pin) == GPIO_PIN_RESET;
    // Use GPIO_PIN_SET if your switch is active-high instead
}

#endif
