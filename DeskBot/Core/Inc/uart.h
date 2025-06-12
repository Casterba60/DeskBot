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

#define UART_BUFFER_SIZE 100

// Angle limits
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

typedef struct {
    int theta1;
    int theta2;
    int theta3;
    int theta4;
    int theta5;
} JointAngles;

void UART_Init(UART_HandleTypeDef* huart);
void UART_ProcessReceivedData(void);
bool UART_GetLatestAngles(JointAngles* angles);



#endif /* INC_UART_H_ */
