/*
 * uart_driver.c
 *
 *  Created on: 26 Feb 2023
 *      Author: jaron
 */
#include "uart_driver.h"
#include "globals.h"
#include "motor_control.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

extern UART_HandleTypeDef huart2;

extern char receive_buffer[32];
extern char send_buffer[32];
extern uint8_t message_waiting_flag;



void uart_startup_transmit()
{
	HAL_Delay(5);
	char * startup_msg = "#:24733725:$\n";
	strcpy(send_buffer, startup_msg);
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)send_buffer, strlen(send_buffer));
	HAL_UART_Receive_IT(&huart2, (uint8_t *)receive_buffer, 1);
}

void uart_task()
{
	if (message_waiting_flag)
	{
		uart_transmit(receive_buffer, strlen(receive_buffer));
		forward(atoi(receive_buffer));
		message_waiting_flag = 0;
		strcpy(receive_buffer, "\0");
	}
}

void uart_transmit(char * message, int length)
{
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, length);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t index = 0;
	if(receive_buffer[index] == '\n')
	{
		receive_buffer[index+1] = '\0';
		message_waiting_flag = 1;
		index = 0;
	}
	else if (index < 30)
	{
		index = index + 1;
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t *)(receive_buffer + index), 1);

}

