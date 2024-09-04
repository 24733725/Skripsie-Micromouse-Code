/*
 * globals.c
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#include "globals.h"
#include "inttypes.h"

char send_buffer[64];
char receive_buffer[32];
volatile uint8_t control_loop_flag = 0;
uint8_t message_waiting_flag = 0;

volatile int16_t L_speed_setpoint = 0; //mm/s
volatile int16_t R_speed_setpoint = 0;//mm/s
volatile int16_t L_prev_enc_count = 0;
volatile int16_t R_prev_enc_count = 0;
volatile int32_t L_ctrl_signal = 0;
volatile int32_t R_ctrl_signal = 0;
volatile int32_t L_error = 0;
volatile int32_t R_error = 0;
volatile int32_t L_acc_error = 0;
volatile int32_t R_acc_error = 0;
volatile int32_t L_acc = 0;
volatile int32_t R_acc = 0;
volatile int32_t Dist_error_acc = 0;

volatile uint8_t measurements[3]; //L:M:R



