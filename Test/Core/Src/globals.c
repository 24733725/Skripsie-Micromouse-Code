/*
 * globals.c
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#include "globals.h"
#include "inttypes.h"

char usb_send_buffer[32];
char usb_receive_buffer[32];
volatile uint8_t control_loop_flag = 0;
