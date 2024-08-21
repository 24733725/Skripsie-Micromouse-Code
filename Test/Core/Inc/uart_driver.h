/*
 * uart_driver.h
 *
 *  Created on: 26 Feb 2023
 *      Author: jaron
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "globals.h"
#include "stm32f4xx_hal.h"

void uart_startup_transmit();
void uart_task();
void uart_transmit(char * message, int length);
#endif /* UART_DRIVER_H_ */
