/*
 * servo.c
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */
/* servo.c */
#include "servo.h"

#define PWM_FRAME_TICKS 4500    // 20ms / 5us = 4000
#define MIN_PULSE_TICKS 140      // 1000us / 5us
#define MAX_PULSE_TICKS 460       // 2000us / 5us

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint16_t pulse_ticks;
} Servo;

static Servo servos[MAX_SERVOS];
static uint8_t servo_count = 0;
static volatile uint16_t pwm_counter = 0;
static TIM_HandleTypeDef* servo_timer;

void servo_init_timer(TIM_HandleTypeDef* htim)
{
    servo_timer = htim;
    //htim->Instance->PSC = 15;  // 16 MHz / 16 = 1 MHz
    //htim->Instance->ARR = 4;
    HAL_TIM_Base_Start_IT(htim);
}

int8_t servo_add(GPIO_TypeDef* port, uint16_t pin)
{
    if (servo_count >= MAX_SERVOS) return -1;
    servos[servo_count].port = port;
    servos[servo_count].pin = pin;
    servos[servo_count].pulse_ticks = 300; // default to 90°
    return servo_count++;
}

void servo_set_angle(uint8_t index, float angle_deg)
{
    if (index >= servo_count) return;
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    uint16_t ticks = MIN_PULSE_TICKS + (uint16_t)((angle_deg / 180.0f) * (MAX_PULSE_TICKS - MIN_PULSE_TICKS));
    servos[index].pulse_ticks = ticks;
}

void servo_update_tick(void)
{
    pwm_counter++;
    if (pwm_counter >= PWM_FRAME_TICKS)
        pwm_counter = 0;

    for (uint8_t i = 0; i < servo_count; ++i)
    {
        if (pwm_counter == 0)
            servos[i].port->BSRR = (1 << servos[i].pin);
        else if (pwm_counter == servos[i].pulse_ticks)
            servos[i].port->BSRR = (1 << (servos[i].pin + 16));
    }
}
