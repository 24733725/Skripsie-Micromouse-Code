/*
 * mazemanager.c
 *
 *  Created on: 17 Sep 2024
 *      Author: jaron
 */
#include "mazemanager.h"
#include "globals.h"
#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "uart_driver.h"
#include "string.h"
#include "stdio.h"

extern uint8_t measurements[3]; //L:M:R
extern char send_buffer[64];
extern Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];

extern int32_t L_acc;
extern int32_t R_acc;
extern uint8_t heading;
extern uint8_t current_cell_x;
extern uint8_t current_cell_y;

void maze_init(){
	// set all walls to zero, populate with manhattan dist
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
			maze[i][j].dist = abs(END_CELL_X-i) + abs(END_CELL_Y-j);
			maze[i][j].walls = 0;
		}
	}
	//fill top and bottom edge walls
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		maze[i][0].walls = maze[i][0].walls | 0b1;
		maze[i][MAZE_CELL_HEIGHT-1].walls = maze[i][MAZE_CELL_HEIGHT-1].walls | 0b001;
	}
	//fill left and right edge walls
	for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
		maze[0][j].walls = maze[0][j].walls | 0b1000;
		maze[MAZE_CELL_WIDTH-1][j].walls = maze[MAZE_CELL_WIDTH-1][j].walls| 0b10;
	}
	// explored starting square
	maze[0][0].walls =  0b11111101;
}

void explore(){
	move(600,0);
	while(1){
		HAL_Delay(10);
		if (measurements[0]>100) turn(-90);
		else if (measurements[2]>100) turn(90);
		else turn(180);

		HAL_Delay(10);
		move(600,0);
	}
	HAL_Delay(10);
	turn(90);
}
void update(){
	if ((L_acc%208) > 98 && (L_acc%208) < 110){
		//check walls
	}
	if (L_acc >= 208){
		switch (heading) {
			case 0:
				current_cell_x += L_acc/208; //(120*180)/33pi

				L_acc -= 208;
				R_acc -= 208;
				break;
			case 1:
				current_cell_x += L_acc/295; //sqrt(2) * 208
				current_cell_y += L_acc/295;

				L_acc -= 295;
				R_acc -= 295;
				break;
			case 2:
				current_cell_y += L_acc/208;

				L_acc -= 208;
				R_acc -= 208;
				break;
			case 3:
				current_cell_x -= L_acc/295;
				current_cell_y += L_acc/295;

				L_acc -= 295;
				R_acc -= 295;
				break;
			case 4:
				current_cell_x -= L_acc/208;

				L_acc -= 208;
				R_acc -= 208;
				break;
			case 5:
				current_cell_x -= L_acc/295;
				current_cell_y -= L_acc/295;

				L_acc -= 295;
				R_acc -= 295;
				break;
			case 6:
				current_cell_y -= L_acc/208;

				L_acc -= 208;
				R_acc -= 208;
				break;
			case 7:
				current_cell_x += L_acc/295;
				current_cell_y -= L_acc/295;

				L_acc -= 295;
				R_acc -= 295;
				break;
			default:
				break;
		}
	}
	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
	uart_transmit(send_buffer, strlen(send_buffer));
}
