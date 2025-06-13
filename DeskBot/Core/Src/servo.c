/*
 * servo.c
 *
 *  Created on: Jun 10, 2025
 *      Author: cole
 */
/* servo.c */
#include "servo.h"

/// Number of timer ticks per PWM frame (e.g., 20 ms = 4000 ticks @ 5 µs resolution)
#define PWM_FRAME_TICKS 4500

/// Minimum pulse width in ticks (corresponds to 0°)
#define MIN_PULSE_TICKS 140

/// Maximum pulse width in ticks (corresponds to 180°)
#define MAX_PULSE_TICKS 460

/**
 * @brief Internal servo tracking structure.
 */
typedef struct {
	GPIO_TypeDef* port;      /**< GPIO port for the servo signal */
	uint16_t pin;            /**< GPIO pin number (0–15) */
	uint16_t pulse_ticks;    /**< Pulse width in ticks (1 tick = ~5 µs) */
} Servo;

static Servo servos[MAX_SERVOS];           ///< Array of managed servos
static uint8_t servo_count = 0;            ///< Number of active servos
static volatile uint16_t pwm_counter = 0;  ///< Shared timer counter for PWM timing
static TIM_HandleTypeDef* servo_timer;     ///< Timer used for scheduling

/**
 * @brief Initializes the base timer used for PWM timing.
 *
 * Enables timer interrupts to allow software PWM generation.
 *
 * @param htim Pointer to the timer handle.
 */
void servo_init_timer(TIM_HandleTypeDef* htim)
{
    servo_timer = htim;
    //htim->Instance->PSC = 15;  // 16 MHz / 16 = 1 MHz
    //htim->Instance->ARR = 4;
    HAL_TIM_Base_Start_IT(htim);
}

/**
 * @brief Registers a new servo on a specified GPIO pin.
 *
 * Adds the servo to the internal tracking list. Default angle is 90°.
 *
 * @param port GPIO port for the servo signal.
 * @param pin GPIO pin number (0–15).
 * @return Index of the added servo, or -1 if the list is full.
 */
int8_t servo_add(GPIO_TypeDef* port, uint16_t pin)
{
    if (servo_count >= MAX_SERVOS) return -1;
    servos[servo_count].port = port;
    servos[servo_count].pin = pin;
    servos[servo_count].pulse_ticks = 300; // default to 90°
    return servo_count++;
}

/**
 * @brief Sets the desired angle for a registered servo.
 *
 * Converts the angle to a PWM pulse width and stores it in the servo struct.
 *
 * @param index Index of the servo.
 * @param angle_deg Desired angle in degrees (0 to 180).
 */
void servo_set_angle(uint8_t index, float angle_deg)
{
    if (index >= servo_count) return;
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    uint16_t ticks = MIN_PULSE_TICKS + (uint16_t)((angle_deg / 180.0f) * (MAX_PULSE_TICKS - MIN_PULSE_TICKS));
    servos[index].pulse_ticks = ticks;
}

/**
 * @brief Updates servo PWM outputs based on the shared tick counter.
 *
 * Call this function from the timer ISR. It will:
 * - Set the pin high when a new PWM frame starts.
 * - Set the pin low when the pulse duration ends.
 */
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
