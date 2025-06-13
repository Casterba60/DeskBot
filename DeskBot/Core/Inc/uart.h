/*
 * uart.h
 *
 *  Created on: Jun 11, 2025
 *      Author: cole
 */

#ifndef INC_UART_H_
#define INC_UART_H_


#include "stm32f4xx_hal.h"
#include <stdbool.h>

/**
 * @brief Maximum number of characters the UART input buffer can hold.
 */
#define UART_BUFFER_SIZE 100

// Angle limits for each joint (in degrees)
#define THETA1_MIN -180
#define THETA1_MAX 180
#define THETA2_MIN 0
#define THETA2_MAX 200
#define THETA3_MIN 0
#define THETA3_MAX 220
#define THETA4_MIN 0
#define THETA4_MAX 180
#define THETA5_MIN 0
#define THETA5_MAX 150

/**
 * @brief Struct representing a 4-DOF robotic arm's joint angles as well as end effector command
 */
typedef struct {
    int theta1;
    int theta2;
    int theta3;
    int theta4;
    int theta5;
} JointAngles;

/**
 * @brief Initializes the UART module and starts reception.
 *
 * Sets up the UART interrupt to receive characters asynchronously.
 *
 * @param huart Pointer to the UART handle.
 */
void UART_Init(UART_HandleTypeDef* huart);
/**
 * @brief Parses and validates a complete UART message.
 *
 * Should be called periodically or from the main loop after UART input is received.
 */
void UART_ProcessReceivedData(void);

/**
 * @brief Retrieves the most recently received and validated joint angles.
 *
 * @param angles Pointer to a JointAngles struct to store the result.
 * @return true if valid angles are available; false otherwise.
 */
bool UART_GetLatestAngles(JointAngles* angles);



#endif /* INC_UART_H_ */
