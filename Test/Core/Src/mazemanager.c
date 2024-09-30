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

extern MouseStruct Mouse;

//extern TIM_HandleTypeDef htim5;

void maze_init(){
	// set all walls to zero, populate with manhattan dist
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
			maze[i][j].dist = abs(END_CELL_X-i) + abs(END_CELL_Y-j);
			maze[i][j].walls = 0x0F;
		}
	}
	//should actually assume all walls filled in
	//fill top and bottom edge walls
	//	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
	//		add_wall(i, 0, NORTH);
	//		add_wall(i, MAZE_CELL_HEIGHT-1, SOUTH);
	//	}
	//	//fill left and right edge walls
	//	for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
	//		add_wall(0, j, WEST);
	//		add_wall(MAZE_CELL_WIDTH-1, j, EAST);
	//	}
	// explored starting square
	rem_wall(0, 0, EAST);
	set_explored(0, 0);

	for (int i = 0; i < MAZE_CELL_HEIGHT; i++) {
		sprintf(send_buffer, "|%.2d|%.2d|%.2d|%.2d|%.2d|%.2d|\n",(int)maze[0][i].walls, (int)maze[1][i].walls,(int)maze[2][i].walls,(int)maze[3][i].walls,(int)maze[4][i].walls,(int)maze[5][i].walls);
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(50);
	}
}

void explore(){
	move(400,0);
	while(1){
		HAL_Delay(200);
		turn(90);
//		if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(LEFT))==0) turn(-90);
//		else if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(RIGHT))==0) turn(90);
//		else turn(180);
		//		save_maze();
		HAL_Delay(300);
		move(400,0);
//		if(Mouse.current_cell_x >= MAZE_CELL_WIDTH || Mouse.current_cell_y >= MAZE_CELL_HEIGHT) break;
		//		sprintf(send_buffer, "L:%.3d M:%.3d R:%.3d E:%d\n",(int)measurements[0],(int)measurements[1] ,(int)measurements[2], (int)htim5.Instance->CNT);
		//		uart_transmit(send_buffer, strlen(send_buffer));
	}
}
void update(){
	static uint8_t prev_measurements[3] = {255, 255, 255};
	if(measurements[0] - prev_measurements[0] > 55){ //big jump in left, therefore opening
		switch (Mouse.heading) {//update block in front
		case 0:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, NORTH);
			break;
		case 2:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y+1, EAST);
			break;
		case 4:
			rem_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, SOUTH);
			break;
		case 6:
			rem_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, WEST);
			break;
		default:
			break;
		}
	}
	if(measurements[0] > 180){ //check middle
		switch (Mouse.heading) {//update block in front
		case 0:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, EAST);
			break;
		case 2:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y+1, SOUTH);
			break;
		case 4:
			rem_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
			break;
		case 6:
			rem_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, NORTH);
			break;
		default:
			break;
		}
	}
	if(measurements[2] - prev_measurements[2] > 55){ //big jump in right, therefore opening
		switch (Mouse.heading) {//update block in front
		case 0:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, SOUTH);
			break;
		case 2:
			rem_wall(Mouse.current_cell_x+1, Mouse.current_cell_y+1, WEST);
			break;
		case 4:
			rem_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, NORTH);
			break;
		case 6:
			rem_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, EAST);
			break;
		default:
			break;
		}
	}
	//update current cell coords
	switch (Mouse.heading) {
	case 0:
		if (L_acc >= 208 && L_acc >= 208){
			Mouse.current_cell_y ++; //208 = (120*180)/33pi

			L_acc -= 208;
			R_acc -= 208;
		}
		break;
	case 1:
		if (L_acc >= 295 && L_acc >= 295){
			Mouse.current_cell_x ++; //sqrt(2) * 208
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 2:
		if (L_acc >= 208 && L_acc >= 208){
			Mouse.current_cell_x ++;

			L_acc -= 208;
			R_acc -= 208;
		}
		break;
	case 3:
		if (L_acc >= 295 && L_acc >= 295){
			Mouse.current_cell_x ++;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 4:
		if (L_acc >= 208 && L_acc >= 208){
			Mouse.current_cell_y --;

			L_acc -= 208;
			R_acc -= 208;
		}
		break;
	case 5:
		if (L_acc >= 295 && L_acc >= 295){
			Mouse.current_cell_x --;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 6:
		if (L_acc >= 208 && L_acc >= 208){
			Mouse.current_cell_x --;

			L_acc -= 208;
			R_acc -= 208;
		}
		break;
	case 7:
		if (L_acc >= 295 && L_acc >= 295){
			Mouse.current_cell_x --;
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	default:
		break;
	}
	set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
	prev_measurements[0] = measurements[0];
	prev_measurements[1] = measurements[1];
	prev_measurements[2] = measurements[2];
}
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d \n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	uart_transmit(send_buffer, strlen(send_buffer));

void save_maze(){

	//	FLASH_Erase_Sector(FLASH_SECTOR_5, VOLTAGE_RANGE_3);
	uint32_t Laddress = 0x08020000;
	//	uint32_t Raddress = 0x08030000;
	for (int i = 0; i<MAZE_CELL_HEIGHT; i++){
		for (int j = 0; j<MAZE_CELL_WIDTH; j++){
			HAL_FLASH_Program(TYPEPROGRAM_BYTE, Laddress+6*i + j, maze[j][i].walls);
		}
		//		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, Raddress+2*i, R_vals[i]);
	}
	//	HAL_FLASH_Lock();
}
Direction rel_to_fixed_dir(Relative_Direction mouse_dir){
	return ((Mouse.heading/2)+ mouse_dir)%4;
}
void rem_wall(uint8_t x, uint8_t y, uint8_t dir) {
    if ((maze[x][y].walls & 0xF0) == 0) {
        maze[x][y].walls &= ~(1 << dir);
        if (dir == NORTH && y + 1 < MAZE_CELL_HEIGHT) {
            maze[x][y + 1].walls &= ~(1 << SOUTH);
        } else if (dir == EAST && x + 1 < MAZE_CELL_WIDTH) {
            maze[x + 1][y].walls &= ~(1 << WEST);
        } else if (dir == SOUTH && y > 0) {
            maze[x][y - 1].walls &= ~(1 << NORTH);
        } else if (dir == WEST && x > 0) {
            maze[x - 1][y].walls &= ~(1 << EAST);
        }
    }
}
uint8_t read_wall(uint8_t x, uint8_t y, Direction dir){
	if ((maze[x][y].walls & (0x01<<dir))==0){
		return 0;
	}
	else return 1;
}
void set_explored(uint8_t x, uint8_t y) {
    if (x >= 0 && x < MAZE_CELL_WIDTH && y >= 0 && y < MAZE_CELL_HEIGHT) {
        maze[x][y].walls |= 0xF0;
    }
}
uint8_t get_explored(uint8_t x, uint8_t y) {
    if (x >= 0 && x < MAZE_CELL_WIDTH && y >= 0 && y < MAZE_CELL_HEIGHT) {
        return (maze[x][y].walls & 0xF0) != 0;
    }
    return 1;
}
