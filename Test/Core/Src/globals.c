/*
 * globals.c
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#include "globals.h"
#include "inttypes.h"

char send_buffer[32];
char receive_buffer[32];
volatile uint8_t control_loop_flag = 0;
uint8_t message_waiting_flag = 0;
