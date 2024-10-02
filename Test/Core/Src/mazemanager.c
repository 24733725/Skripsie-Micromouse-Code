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
			maze[i][j].walls = 0;
		}
	}
	//should actually assume all walls filled in
	//fill top and bottom edge walls
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		add_wall(i, 0, SOUTH);
		add_wall(i, MAZE_CELL_HEIGHT-1, NORTH);
	}
	//fill left and right edge walls
	for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
		add_wall(0, j, WEST);
		add_wall(MAZE_CELL_WIDTH-1, j, EAST);
	}
	// explored starting square
    add_wall(0, 0, EAST);
    add_wall(0, 0, SOUTH);
    add_wall(0, 0, WEST);
	set_explored(0, 0);

//	print_maze();

//    add_wall(0, 2, EAST);
//    add_wall(0, 2, NORTH);
//    set_explored(0, 1);
//    flood(END_CELL_X, END_CELL_Y);
//
//    print_maze();
}
void print_maze(){
	HAL_Delay(15);
	for (int i = MAZE_CELL_HEIGHT-1; i>=0; i--) {
		for (int j = 0; j < MAZE_CELL_WIDTH; j++) {
			sprintf(send_buffer, "|%.2d",(int)maze[j][i].walls);
			uart_transmit(send_buffer, strlen(send_buffer));
			HAL_Delay(15);
		}
		sprintf(send_buffer, "|\n");
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(15);
	}
	sprintf(send_buffer, "\n");
	uart_transmit(send_buffer, strlen(send_buffer));
	HAL_Delay(15);
	for (int i = MAZE_CELL_HEIGHT-1; i>=0; i--) {
		for (int j = 0; j < MAZE_CELL_WIDTH; j++) {
			sprintf(send_buffer, "|%.2d",(int)maze[j][i].dist);
			uart_transmit(send_buffer, strlen(send_buffer));
			HAL_Delay(15);
		}
		sprintf(send_buffer, "|\n");
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(15);
	}
	sprintf(send_buffer, "\n");
	uart_transmit(send_buffer, strlen(send_buffer));
	HAL_Delay(15);
}
void turn_to_direction(uint8_t target_dir){
	uint8_t diff = (4 + target_dir - (Mouse.heading / 2)) % 4;
    if (diff == 1){
    	turn(90);
    }
    else if (diff == 2){
    	turn(180);
    }
    else if (diff == 3){
    	turn(-90);
    }
}
void explore(uint8_t x, uint8_t y){
	while(!((Mouse.current_cell_x == x) && (Mouse.current_cell_y == y))){
		flood(x, y);

		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(300);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y));

		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		move(400,0);
//		print_maze();
//		if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(LEFT))==0) turn(-90);
//		else if (read_wall(current_cell_x, current_cell_y, rel_to_fixed_dir(RIGHT))==0) turn(90);
//		else turn(180);
		//		save_maze();
//		if(Mouse.current_cell_x >= MAZE_CELL_WIDTH || Mouse.current_cell_y >= MAZE_CELL_HEIGHT) break;
		//		sprintf(send_buffer, "L:%.3d M:%.3d R:%.3d E:%d\n",(int)measurements[0],(int)measurements[1] ,(int)measurements[2], (int)htim5.Instance->CNT);
		//		uart_transmit(send_buffer, strlen(send_buffer));
	}
	sprintf(send_buffer, "why\n");
	uart_transmit(send_buffer, strlen(send_buffer));
}
void update(){
//	static uint8_t prev_measurements[3] = {255, 255, 255};
	//update current cell coords
	// at 400mm/s, 9-10 counts per loop
	switch (Mouse.heading) {
	case 0:
		// update coords
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, NORTH);
			}
		}
		//set L and R walls
		if (L_acc >= 120 && L_acc <= 135 && R_acc >= 120 && R_acc <= 135){
			if(measurements[0] < 100){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, WEST);
			}
			if(measurements[2] < 100){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, EAST);
			}
		}
		break;
	case 1:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++; //sqrt(2) * 208
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 2:
		// update coords
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, EAST);
			}
		}
		//set L and R walls
		if (L_acc >= 120 && L_acc <= 135 && R_acc >= 120 && R_acc <= 135){
			if(measurements[0] < 100){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, NORTH);
			}
			if(measurements[2] < 100){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, SOUTH);
			}
		}
		break;
	case 3:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 4:
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, SOUTH);
			}
		}
		//set L and R walls
		if (L_acc >= 120 && L_acc <= 135 && R_acc >= 120 && R_acc <= 135){
			if(measurements[0] < 100){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, EAST);
			}
			if(measurements[2] < 100){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, WEST);
			}
		}
		break;
	case 5:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	case 6:
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
			}
		}
		//set L and R walls
		if (L_acc >= 120 && L_acc <= 135 && R_acc >= 120 && R_acc <= 135){
			if(measurements[0] < 100){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, SOUTH);
			}
			if(measurements[2] < 100){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, NORTH);
			}
		}
		break;
	case 7:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
		}
		break;
	default:
		break;
	}
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d \n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	uart_transmit(send_buffer, strlen(send_buffer));

//	prev_measurements[0] = measurements[0];
//	prev_measurements[1] = measurements[1];
//	prev_measurements[2] = measurements[2];
}
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);

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
void add_wall(uint8_t x, uint8_t y, uint8_t dir) {
    if ((maze[x][y].walls & 0xF0) == 0) {
        maze[x][y].walls |= (0b01 << dir);
        if (dir == NORTH) {
            if (y + 1 < MAZE_CELL_HEIGHT) {
                maze[x][y + 1].walls |= (0b01 << SOUTH);
            }
        } else if (dir == EAST) {
            if (x + 1 < MAZE_CELL_WIDTH) {
                maze[x + 1][y].walls |= (0b01 << WEST);
            }
        } else if (dir == SOUTH) {
            if (y > 0) {
                maze[x][y - 1].walls |= (0b01 << NORTH);
            }
        } else if (dir == WEST) {
            if (x > 0) {
                maze[x - 1][y].walls |= (0b01 << EAST);
            }
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
uint8_t dir_of_lowest(uint8_t x, uint8_t y) {
	uint8_t min = 255;
	uint8_t dir = NORTH;
    if (read_wall(x, y, NORTH) == 0) {
        if (maze[x][y + 1].dist < min) {
            min = maze[x][y + 1].dist;
            dir = NORTH;
        }
    }
    if (read_wall(x, y, EAST) == 0) {
        if (maze[x + 1][y].dist < min) {
            min = maze[x + 1][y].dist;
            dir = EAST;
        }
    }
    if (read_wall(x, y, SOUTH) == 0) {
        if (maze[x][y - 1].dist < min) {
            min = maze[x][y - 1].dist;
            dir = SOUTH;
        }
    }
    if (read_wall(x, y, WEST) == 0) {
        if (maze[x - 1][y].dist < min) {
            min = maze[x - 1][y].dist;
            dir = WEST;
        }
    }
    return dir;
}
void flood(uint8_t ex, uint8_t ey) {
	uint8_t nochange_flag = 0;
    maze[ex][ey].dist = 0;

    uint iter = 0;
    while (nochange_flag == 0) {
    	uint8_t change_flag = 0;

        for (uint8_t x = 0; x < MAZE_CELL_WIDTH; x++) {
            for (uint8_t y = 0; y < MAZE_CELL_HEIGHT; y++) {
                if (!(x == ex && y == ey)) {
                	uint8_t min = 255;
                    if (read_wall(x, y, NORTH) == 0) {
                        if (maze[x][y + 1].dist < min) {
                            min = maze[x][y + 1].dist;
                        }
                    }
                    if (read_wall(x, y, EAST) == 0) {
                        if (maze[x + 1][y].dist < min) {
                            min = maze[x + 1][y].dist;
                        }
                    }
                    if (read_wall(x, y, SOUTH) == 0) {
                        if (maze[x][y - 1].dist < min) {
                            min = maze[x][y - 1].dist;
                        }
                    }
                    if (read_wall(x, y, WEST) == 0) {
                        if (maze[x - 1][y].dist < min) {
                            min = maze[x - 1][y].dist;
                        }
                    }
                    if (maze[x][y].dist != min + 1) {
                        change_flag = 1;
                        maze[x][y].dist = min + 1;
                    }
                }
            }
        }
        // Check if there were no changes in this iteration
        if (change_flag == 0) {
            nochange_flag = 1;
        }
//        print_maze();
//    	sprintf(send_buffer, "fl:%d\n", iter);
//    	uart_transmit(send_buffer, strlen(send_buffer));
//    	HAL_Delay(15);
    	iter++;
    }
//	sprintf(send_buffer, "flood cmplt\n");
//	uart_transmit(send_buffer, strlen(send_buffer));
}
