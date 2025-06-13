/*
 * joint.h
 *
 *  Created on: Jun 8, 2025
 *      Author: cole
 */

#ifndef INC_JOINT_H_
#define INC_JOINT_H_

#include "controller.h" //pid controller
#include "motor.h"
#include "limit_switch.h"
#include <stdbool.h>

/**
 * @brief Structure representing a joint with dual-loop PID control.
 *
 * Combines motor control, encoder feedback, and hierarchical PID loops for position and velocity.
 */
typedef struct {
	PIDController position_pid;           /**< Outer loop: Position PID controller */
	PIDController velocity_pid;           /**< Inner loop: Velocity PID controller */

	int desired_position;                 /**< Target position in encoder ticks */
	int desired_velocity;                 /**< Target velocity (output of position PID) */

	int actual_position;                  /**< Measured encoder position */
	int actual_velocity;                  /**< Measured velocity (ticks/dt) */

	int control_output;                   /**< Final control effort passed to motor */

	motor_t* p_mot;                       /**< Pointer to motor structure */
	TIM_HandleTypeDef* encoderHandle;     /**< Pointer to encoder timer */

	int previousEncoderCount;             /**< Previous encoder count for velocity calc */
	int encoder_init;                     /**< Flag for first encoder read */

	int enable;                           /**< Joint enable flag (1 = active, 0 = ignore) */
} joint;

/**
 * @brief Initializes a joint with motor, encoder, and PID parameters.
 *
 * @param joint Pointer to the joint object.
 * @param p_mot Pointer to the motor structure.
 * @param encoderHandle Pointer to the timer used as quadrature encoder.
 * @param pos_pid Pointer to position PID controller config.
 * @param vel_pid Pointer to velocity PID controller config.
 */
void Joint_Init(joint* joint, motor_t* p_mot, TIM_HandleTypeDef* encoderHandle,
		PIDController* pos_pid,PIDController* vel_pid);

/**
 * @brief Updates the joint control loop (called periodically).
 *
 * Reads encoder values, computes desired velocity (outer loop),
 * and applies velocity PID to generate motor control output (inner loop).
 *
 * @param joint Pointer to the joint object.
 * @param dt_ms Time step since last update (in milliseconds).
 */
void Joint_Update(joint* joint, int dt_ms);

/**
 * @brief Homes the joint using a limit switch.
 *
 * Drives motor in specified direction until the switch is triggered, then resets encoder position.
 *
 * @param joint Pointer to the joint object.
 * @param limswitch Pointer to limit switch object.
 * @param direction Direction to drive motor during homing (-1 or +1).
 * @param speed Speed to apply while homing.
 * @return true if homing is complete; false if still in progress.
 */
bool Joint_Home(joint* joint,LimitSwitch* limswitch,int direction, int speed);

#endif /* INC_JOINT_H_ */
