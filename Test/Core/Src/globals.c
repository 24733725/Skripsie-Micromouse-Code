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
uint8_t message_waiting_flag = 0;

int16_t L_speed_setpoint = 0; //mm/s
int16_t R_speed_setpoint = 0;//mm/s
int16_t L_prev_enc_count = 0;
int16_t R_prev_enc_count = 0;
int32_t L_ctrl_signal = 0;
int32_t R_ctrl_signal = 0;
int32_t L_error = 0;
int32_t R_error = 0;
int32_t L_acc_error = 0;
int32_t R_acc_error = 0;
int32_t L_acc = 0;
int32_t R_acc = 0;
int32_t Dist_error_acc = 0;

uint8_t measurements[3]; //L:M:R

Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];

MouseStruct Mouse = {0, 0, 0};





