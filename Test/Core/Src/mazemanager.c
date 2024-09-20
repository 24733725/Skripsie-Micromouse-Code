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
extern uint8_t E_Stop;

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
		add_wall(i, 0, NORTH);
		add_wall(i, MAZE_CELL_HEIGHT-1, SOUTH);
	}
	//fill left and right edge walls
	for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
		add_wall(0, j, WEST);
		add_wall(MAZE_CELL_WIDTH-1, j, EAST);
	}
	// explored starting square
	add_wall(0, 0, NORTH);
	add_wall(0, 0, SOUTH);
	add_wall(0, 0, WEST);
	set_explored(0, 0);

	for (int i = 0; i < MAZE_CELL_HEIGHT; i++) {
		sprintf(send_buffer, "|%.2d|%.2d|%.2d|%.2d|%.2d|%.2d|\n",(int)maze[0][i].walls, (int)maze[1][i].walls,(int)maze[2][i].walls,(int)maze[3][i].walls,(int)maze[4][i].walls,(int)maze[5][i].walls);
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(100);
	}
}

void explore(){
	move(400,0);
	while(1){
		HAL_Delay(300);
		if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(LEFT))==0) turn(-90);
		else if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(RIGHT))==0) turn(90);
		else turn(180);
		HAL_Delay(300);
		move(400,0);
		if(current_cell_x >= MAZE_CELL_WIDTH || current_cell_y >= MAZE_CELL_HEIGHT) break;
	}
}
void update(){
	if ((L_acc%208) > 98 && (L_acc%208) < 110){
		//check walls
		switch (heading) {
			case 0:
				//update if cell has not been explored, update forward if not at edge
				if (measurements[0]<80){//L
					//update block in front
					add_wall(current_cell_x+1, current_cell_y, NORTH);
					//update block above block in front if not on edge and not explored
					if (current_cell_y > 0){
						add_wall(current_cell_x+1, current_cell_y-1, SOUTH);
					}
				}
				if (measurements[1]<195){//F
					//update block in front
					add_wall(current_cell_x+1, current_cell_y, EAST);
					//update block above block in front if not on edge and not explored
					if (current_cell_x + 2 < MAZE_CELL_WIDTH){
						add_wall(current_cell_x+2, current_cell_y, WEST);
					}
				}
				if (measurements[2]<80){ //R
					//update block in front
					add_wall(current_cell_x+1, current_cell_y, SOUTH);
					//update block above block in front if not on edge and not explored
					if (current_cell_y+1 < MAZE_CELL_HEIGHT){
						add_wall(current_cell_x+1, current_cell_y+1, NORTH);
					}
				}
				set_explored(current_cell_x+1, current_cell_y);
				break;
			case 2:
				if (measurements[0]<80){//L
					add_wall(current_cell_x+1, current_cell_y+1, WEST);
					if (current_cell_x+2 < MAZE_CELL_WIDTH ){
						add_wall(current_cell_x+1, current_cell_y+1, EAST);
					}
				}
				if (measurements[1]<195){//F
					add_wall(current_cell_x, current_cell_y+1, SOUTH);
					if (current_cell_y +2 < MAZE_CELL_HEIGHT){
						add_wall(current_cell_x, current_cell_y+2, NORTH);
					}
				}
				if (measurements[2]<80){ //R
					add_wall(current_cell_x, current_cell_y+1, EAST);
					if (current_cell_x -1 >= 0){
						add_wall(current_cell_x-1, current_cell_y+1, WEST);
					}
				}
				set_explored(current_cell_x, current_cell_y+1);
				break;
			case 4:
				if (measurements[0]<80){//L
					add_wall(current_cell_x-1, current_cell_y, SOUTH);
					if (current_cell_y+1 < MAZE_CELL_HEIGHT ){
						add_wall(current_cell_x-1, current_cell_y-1, NORTH);
					}
				}
				if (measurements[1]<195){//F
					add_wall(current_cell_x-1, current_cell_y, WEST);
					if (current_cell_x - 2 >= 0){
						add_wall(current_cell_x-2, current_cell_y, EAST);
					}
				}
				if (measurements[2]<80){ //R
					add_wall(current_cell_x-1, current_cell_y, NORTH);
					if (current_cell_y -1 <= 0){
						add_wall(current_cell_x-1, current_cell_y-1, SOUTH);
					}
				}
				set_explored(current_cell_x-1, current_cell_y);
				break;
			case 6:
				if (measurements[0]<80){//L
					add_wall(current_cell_x, current_cell_y-1, WEST);
					if (current_cell_x-1 <=0){
						add_wall(current_cell_x-1, current_cell_y-1, EAST);
					}
				}
				if (measurements[1]<195){//F
					add_wall(current_cell_x, current_cell_y-1, NORTH);
					if (current_cell_y - 2 >= 0){
						add_wall(current_cell_x, current_cell_y-2, SOUTH);
					}
				}
				if (measurements[2]<80){ //R
					add_wall(current_cell_x, current_cell_y-1, EAST);
					if (current_cell_x +2 < MAZE_CELL_WIDTH){
						add_wall(current_cell_x+1, current_cell_y-1, WEST);
					}
				}
				set_explored(current_cell_x, current_cell_y-1);
				break;
			default:
				break;
		}
	}
	if (L_acc >= 208){
		//update current cell coords
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
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	uart_transmit(send_buffer, strlen(send_buffer));
}
Direction rel_to_fixed_dir(Relative_Direction mouse_dir){
	return ((heading/2)+ mouse_dir)%4;
}
void add_wall(uint8_t x, uint8_t y, Direction dir){
	if ((maze[x][y].walls & 0xF0) == 0){
		maze[x][y].walls  |=  (0b01<<dir);
	}
}
uint8_t read_wall(uint8_t x, uint8_t y, Direction dir){
	if ((maze[x][y].walls & (0x01<<dir))==0){
		return 0;
	}
	else return 1;
}
void set_explored(uint8_t x, uint8_t y){
	maze[x][y].walls  |=  0xF0;
}
uint8_t get_explored(uint8_t x, uint8_t y){
	if ((maze[x][y].walls & 0xF0) == 0){
		return 0;
	}
	else return 1;
}
