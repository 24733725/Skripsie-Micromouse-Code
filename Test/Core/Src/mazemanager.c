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
extern uint8_t prev_measurements[3]; //L:M:R
extern char send_buffer[64];
extern Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];

extern int32_t L_acc;
extern int32_t R_acc;

extern uint8_t target_x;
extern uint8_t target_y;
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
//    add_wall(1, 3, EAST);
//    add_wall(1, 3, SOUTH);
//    add_wall(2, 3, WEST);
//    add_wall(3, 3, WEST);
//    add_wall(1, 0, NORTH);
//    add_wall(3, 1, WEST);
//    add_wall(2, 1, WEST);
//    set_explored(0, 1);
//    set_explored(1, 1);
//    set_explored(2, 1);
//    set_explored(3, 2);
//
//    print_maze();
//    int t1 = HAL_GetTick();
    flood(END_CELL_X, END_CELL_Y);
//    int t2 = HAL_GetTick();
//
//	sprintf(send_buffer, "%d\n",t1);
//	uart_transmit(send_buffer, strlen(send_buffer));
//	HAL_Delay(15);
//	sprintf(send_buffer, "%d\n",t2);
//	uart_transmit(send_buffer, strlen(send_buffer));
//	HAL_Delay(15);
//
	print_maze();
//	save_maze();
//
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
//		R_acc += 50;
//		L_acc += 50;
    }
    else if (diff == 2){
    	turn(180);
//		R_acc += 75;
//		L_acc += 75;
    }
    else if (diff == 3){
    	turn(-90);
//		R_acc += 50;
//		L_acc += 50;
    }
//    else{
//		R_acc += 60;
//		L_acc += 60;
//    }
}
void explore(){

	while(!((Mouse.current_cell_x == target_x) && (Mouse.current_cell_y == target_y))){
		flood(target_x, target_y);

		HAL_Delay(500);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(300);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

//		sprintf(send_buffer, "turn\n");
//		uart_transmit(send_buffer, strlen(send_buffer));
		turn_to_direction(dir_of_lowest(Mouse.current_cell_x,Mouse.current_cell_y));
		HAL_Delay(500);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(100);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		move(400,0);
		save_maze();
//		print_maze();


	}
	sprintf(send_buffer, "why\n");
	uart_transmit(send_buffer, strlen(send_buffer));
}
void update(){
	static uint8_t L_open_count = 0;
	static uint8_t R_open_count = 0;
	//update current cell coords
	// at 400mm/s, 9-10 counts per loop
	switch (Mouse.heading) {
	case 0:
		// update coords
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			if (L_open_count == 0){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, WEST);
			}
			if (R_open_count == 0){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, EAST);
			}
			L_open_count = 0;
			R_open_count = 0;

			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;

			flood(target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 170 && L_acc <= 200 && R_acc >= 170 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y+1, NORTH);
			}
		}
		//set L and R walls
		if ((measurements[0]- prev_measurements[0]>50)&& measurements[0] > 200){
			L_open_count = 1;
		}
		if ((measurements[2] - prev_measurements[2]>50) && measurements[2] > 200){
			R_open_count = 1;
		}
		break;
	case 1:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++; //sqrt(2) * 208
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
//			flood(Mouse.target_x, Mouse.target_y);
		}
		break;
	case 2:
		// update coords
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			if (L_open_count == 0){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, NORTH);
			}
			if (R_open_count == 0){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, SOUTH);
			}
			L_open_count = 0;
			R_open_count = 0;
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 170 && L_acc <= 200 && R_acc >= 170 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x+1, Mouse.current_cell_y, EAST);
			}
		}
		//set L and R walls
		if ((measurements[0]- prev_measurements[0]>50)&& measurements[0] > 200){
			L_open_count = 1;
		}
		if ((measurements[2] - prev_measurements[2]>50) && measurements[2] > 200){
			R_open_count = 1;
		}
		break;
	case 3:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
//			flood(Mouse.target_x, Mouse.target_y);
		}
		break;
	case 4:
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			if (L_open_count == 0){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, EAST);
			}
			if (R_open_count == 0){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, WEST);
			}
			L_open_count = 0;
			R_open_count = 0;

			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 170 && L_acc <= 200 && R_acc >= 170 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x, Mouse.current_cell_y-1, SOUTH);
			}
		}
		//set L and R walls
		if ((measurements[0]- prev_measurements[0]>50)&& measurements[0] > 200){
			L_open_count = 1;
		}
		if ((measurements[2] - prev_measurements[2]>50) && measurements[2] > 200){
			R_open_count = 1;
		}
		break;
	case 5:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;
			Mouse.current_cell_y --;

			L_acc -= 295;
			R_acc -= 295;
//			flood(Mouse.target_x, Mouse.target_y);
		}
		break;
	case 6:
		if (L_acc >= COUNTS_PER_CELL && R_acc >= COUNTS_PER_CELL){
			if (L_open_count == 0){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, SOUTH);
			}
			if (R_open_count == 0){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, NORTH);
			}
			L_open_count = 0;
			R_open_count = 0;
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
			}
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 170 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
			}
		}
		//set L and R walls
		if ((measurements[0] - prev_measurements[0]>50)&& measurements[0] > 200){
			L_open_count = 1;
		}
		if ((measurements[2] - prev_measurements[2]>50) && measurements[2] > 200){
			R_open_count = 1;
		}
		break;
	case 7:
		if (L_acc >= 295 && R_acc >= 295){
			set_explored(Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;
			Mouse.current_cell_y ++;

			L_acc -= 295;
			R_acc -= 295;
//			flood(Mouse.target_x, Mouse.target_y);
		}
		break;
	default:
		break;
	}
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d \n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);
//	uart_transmit(send_buffer, strlen(send_buffer));
}
//	sprintf(send_buffer, "x:%d Y:%d LC:%d H:%d\n",(int)current_cell_x,(int)current_cell_y ,(int)L_acc, (int)heading);

void save_maze(){

//	FLASH_Erase_Sector(FLASH_SECTOR_5, VOLTAGE_RANGE_3);
	if (HAL_FLASH_Unlock() != HAL_OK) while(1){  HAL_Delay(10);}
	static uint32_t Laddress = 0x08020000;
	//	uint32_t Raddress = 0x08030000;
	for (int i = 0; i<MAZE_CELL_HEIGHT; i++){
		for (int j = 0; j<MAZE_CELL_WIDTH; j++){
			HAL_FLASH_Program(TYPEPROGRAM_BYTE, Laddress+j + i*0x10, maze[j][MAZE_CELL_HEIGHT-i-1].walls);//
		}
		//		HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, Raddress+2*i, R_vals[i]);
	}
	Laddress += MAZE_CELL_HEIGHT*0x10;
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
	if (y < MAZE_CELL_HEIGHT && y >= 0 && x < MAZE_CELL_WIDTH && x >= 0){
		if ((maze[x][y].walls & 0xF0) == 0) {
			maze[x][y].walls |= (0b01 << dir);
			if (dir == NORTH) {
				if (y + 1 < MAZE_CELL_HEIGHT) {
					if ((maze[x][y+1].walls & 0xF0) == 0) maze[x][y + 1].walls |= (0b01 << SOUTH);
				}
			} else if (dir == EAST) {
				if (x + 1 < MAZE_CELL_WIDTH) {
					if ((maze[x+1][y].walls & 0xF0) == 0)maze[x + 1][y].walls |= (0b01 << WEST);
				}
			} else if (dir == SOUTH) {
				if (y > 0) {
					if ((maze[x][y-1].walls & 0xF0) == 0)maze[x][y - 1].walls |= (0b01 << NORTH);
				}
			} else if (dir == WEST) {
				if (x > 0) {
					if ((maze[x-1][y].walls & 0xF0) == 0)maze[x - 1][y].walls |= (0b01 << EAST);
				}
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
                	uint8_t min = MAZE_CELL_HEIGHT * MAZE_CELL_WIDTH -1;
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

                    if ((min != MAZE_CELL_HEIGHT * MAZE_CELL_WIDTH -1) && (maze[x][y].dist != min + 1)) {
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
