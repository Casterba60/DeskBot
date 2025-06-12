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

#define MAX_SERVOS 8

void servo_init_timer(TIM_HandleTypeDef* htim);
int8_t servo_add(GPIO_TypeDef* port, uint16_t pin);
void servo_set_angle(uint8_t index, float angle_deg);
void servo_update_tick(void);

#endif /* INC_SERVO_H_ */
