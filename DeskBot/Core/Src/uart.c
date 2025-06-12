/*
 * uart.c
 *
 *  Created on: Jun 11, 2025
 *      Author: cole
 */
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef* huart_global;
static char uart_rx_buffer[UART_BUFFER_SIZE];
static char uart_byte;
static volatile bool message_complete = false;
static JointAngles latest_angles;
static bool angles_ready = false;

void UART_Init(UART_HandleTypeDef* huart) {
    huart_global = huart;
    HAL_UART_Receive_IT(huart_global, (uint8_t*)&uart_byte, 1);
}

static void send_invalid_message(int index) {
    char msg[30];
    snprintf(msg, sizeof(msg), "invalid angle %d\r\n", index);
    HAL_UART_Transmit(huart_global, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

static bool validate_and_store(int* vals) {
    if (vals[0] < THETA1_MIN || vals[0] > THETA1_MAX) { send_invalid_message(1); return false; }
    if (vals[1] < THETA2_MIN || vals[1] > THETA2_MAX) { send_invalid_message(2); return false; }
    if (vals[2] < THETA3_MIN || vals[2] > THETA3_MAX) { send_invalid_message(3); return false; }
    if (vals[3] < THETA4_MIN || vals[3] > THETA4_MAX) { send_invalid_message(4); return false; }
    if (vals[4] < THETA5_MIN || vals[4] > THETA5_MAX) { send_invalid_message(5); return false; }

    latest_angles.theta1 = vals[0];
    latest_angles.theta2 = vals[1];
    latest_angles.theta3 = vals[2];
    latest_angles.theta4 = vals[3];
    latest_angles.theta5 = vals[4];
    return true;
}

void UART_ProcessReceivedData(void) {
    if (!message_complete) return;

    message_complete = false;
    angles_ready = false;

    if (uart_rx_buffer[0] != '(' || uart_rx_buffer[strlen(uart_rx_buffer) - 1] != ')') return;

    uart_rx_buffer[strlen(uart_rx_buffer) - 1] = '\0';
    char* inner = uart_rx_buffer + 1;

    int vals[5];
    if (sscanf(inner, "%d,%d,%d,%d,%d", &vals[0], &vals[1], &vals[2], &vals[3], &vals[4]) != 5) return;

    if (validate_and_store(vals)) {
        angles_ready = true;
    }
}

bool UART_GetLatestAngles(JointAngles* angles) {
    if (!angles_ready) return false;
    *angles = latest_angles;
    angles_ready = false;
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    static uint16_t index = 0;

    if (huart->Instance == huart_global->Instance) {
        if (uart_byte == '\r') {
            // Treat CR as end of message
            uart_rx_buffer[index] = '\0';
            index = 0;
            message_complete = true;
            HAL_UART_Transmit(huart_global, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
        }
        else if (uart_byte == 0x08 || uart_byte == 0x7F) {
            // Handle backspace/delete
            if (index > 0) {
                index--;
                HAL_UART_Transmit(huart_global, (uint8_t*)"\b \b", 3, HAL_MAX_DELAY); // erase last char
            }
        }
        else if (index < UART_BUFFER_SIZE - 1) {
            // Echo and store character
            uart_rx_buffer[index++] = uart_byte;
            HAL_UART_Transmit(huart_global, (uint8_t*)&uart_byte, 1, HAL_MAX_DELAY);
        }

        // Restart reception
        HAL_UART_Receive_IT(huart_global, (uint8_t*)&uart_byte, 1);
    }
}

