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

extern Cell race_maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];
extern Cell exp_maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];
extern Path paths[MAX_PATHS];

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
			exp_maze[i][j].dist = abs(END_CELL_X-i) + abs(END_CELL_Y-j);
			exp_maze[i][j].walls = 0;
			race_maze[i][j].dist = abs(END_CELL_X-i) + abs(END_CELL_Y-j);
			race_maze[i][j].walls = 0;
		}
	}
	//should actually assume all walls filled in
	//fill top and bottom edge walls
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		add_wall(exp_maze, i, 0, SOUTH);
		add_wall(exp_maze, i, MAZE_CELL_HEIGHT-1, NORTH);
	}
	//fill left and right edge walls
	for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
		add_wall(exp_maze, 0, j, WEST);
		add_wall(exp_maze, MAZE_CELL_WIDTH-1, j, EAST);
	}
	// explored starting square
    add_wall(exp_maze, 0, 0, EAST);
    add_wall(exp_maze, 0, 0, SOUTH);
    add_wall(exp_maze, 0, 0, WEST);
	set_explored(exp_maze, 0, 0);
//	print_maze(exp_maze);
// TEsting maze purposes:
//    add_wall(exp_maze, 0, 2, EAST);
//    add_wall(exp_maze, 0, 2, NORTH);
//    add_wall(exp_maze, 1, 3, EAST);
//    add_wall(exp_maze, 1, 3, SOUTH);
//    add_wall(exp_maze, 2, 3, WEST);
//    add_wall(exp_maze, 3, 3, WEST);
//    add_wall(exp_maze, 1, 0, NORTH);
//    add_wall(exp_maze, 3, 1, WEST);
//    add_wall(exp_maze, 2, 1, WEST);
//    set_explored(exp_maze, 0, 1);
//    set_explored(exp_maze, 1, 1);
//    set_explored(exp_maze, 2, 1);
//    set_explored(exp_maze, 3, 2);
//    set_explored(exp_maze, 2, 2);
//    set_explored(exp_maze, 1, 2);
//    set_explored(exp_maze, 4, 2);
//    set_explored(exp_maze, 5, 2);
//    set_explored(exp_maze, 5, 1);
//    set_explored(exp_maze, 5, 0);
//
//    print_maze(exp_maze);
//    int t1 = HAL_GetTick();
    flood(exp_maze, END_CELL_X, END_CELL_Y);
//    int t2 = HAL_GetTick();
//
//	sprintf(send_buffer, "%d\n",t1);
//	uart_transmit(send_buffer, strlen(send_buffer));
//	HAL_Delay(15);
//	sprintf(send_buffer, "%d\n",t2);
//	uart_transmit(send_buffer, strlen(send_buffer));
//	HAL_Delay(15);
//
	print_maze(exp_maze);
//	save_maze();
//	race();
}
void printpath(Path p){
	for (int i = 0; i < p.len; i++) {
		sprintf(send_buffer, "%d ",(int)p.direction[i]);
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(1);
	}
	sprintf(send_buffer, "\n");
	uart_transmit(send_buffer, strlen(send_buffer));
	HAL_Delay(1);
	for (int i = 0; i < p.len; i++) {
		sprintf(send_buffer, "%d ",(int)p.distance[i]);
		uart_transmit(send_buffer, strlen(send_buffer));
		HAL_Delay(1);
	}
	sprintf(send_buffer, "\n\n");
	uart_transmit(send_buffer, strlen(send_buffer));
	HAL_Delay(1);
}
void print_maze(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT]){
	HAL_Delay(15);
	for (int i = MAZE_CELL_HEIGHT-1; i>=0; i--) {
		for (int j = 0; j < MAZE_CELL_WIDTH; j++) {
			sprintf(send_buffer, "|%.3d",(int)maze[j][i].walls);
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
	uint8_t diff = (8 + target_dir - Mouse.heading ) % 8;
    if (diff == 1){
    	turn(45);
    }
    else if (diff == 2){
    	turn(90);
    }
    else if (diff == 3){
    	turn(135);
    }
    else if (diff == 4){
    	turn(180);
    	reverse(-150);
    }
    else if (diff == 5){
    	turn(-135);
    }
    else if (diff == 6){
    	turn(-90);
    }
    else if (diff == 7){
    	turn(-45);
    }
}
void explore(){

	while(!((Mouse.current_cell_x == target_x) && (Mouse.current_cell_y == target_y))){
		if (measurements[1]<20) break;
		flood(exp_maze,target_x, target_y);

		HAL_Delay(100);

		turn_to_direction(dir_of_lowest(exp_maze, Mouse.current_cell_x,Mouse.current_cell_y));
		HAL_Delay(100);

		move(300,0);
//		save_maze(exp_maze);
//		print_maze();


	}
	set_explored(exp_maze, END_CELL_X, END_CELL_Y);
	sprintf(send_buffer, "why\n");
	uart_transmit(send_buffer, strlen(send_buffer));
}
void go_home(){
	target_x = 0;
	target_y = 0;
	flood(exp_maze, target_x, target_y);
	HAL_Delay(500);
	while(!((Mouse.current_cell_x == target_x) && (Mouse.current_cell_y == target_y))){
		if (measurements[1]<20) break;
		flood(exp_maze, target_x, target_y);

		HAL_Delay(100);

		turn_to_direction(dir_of_lowest(exp_maze, Mouse.current_cell_x,Mouse.current_cell_y));
		HAL_Delay(100);

		move(300,0);
		save_maze(exp_maze);
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
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y+1, WEST);
			}
			if (R_open_count == 0){
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y+1, EAST);
			}
			L_open_count = 0;
			R_open_count = 0;

			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;

			flood(exp_maze, target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y+1, NORTH);
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
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
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
				add_wall(exp_maze, Mouse.current_cell_x+1, Mouse.current_cell_y, NORTH);
			}
			if (R_open_count == 0){
				add_wall(exp_maze, Mouse.current_cell_x+1, Mouse.current_cell_y, SOUTH);
			}
			L_open_count = 0;
			R_open_count = 0;
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x ++; //208 = (120*180)/33pi

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(exp_maze, target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(exp_maze, Mouse.current_cell_x+1, Mouse.current_cell_y, EAST);
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
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
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
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y-1, EAST);
			}
			if (R_open_count == 0){
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y-1, WEST);
			}
			L_open_count = 0;
			R_open_count = 0;

			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_y --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(exp_maze, target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y-1, SOUTH);
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
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
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
				add_wall(exp_maze, Mouse.current_cell_x-1, Mouse.current_cell_y, SOUTH);
			}
			if (R_open_count == 0){
				add_wall(exp_maze, Mouse.current_cell_x-1, Mouse.current_cell_y, NORTH);
			}
			L_open_count = 0;
			R_open_count = 0;
			if(measurements[1] < 200){
				add_wall(exp_maze, Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
			}
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
			Mouse.current_cell_x --;

			L_acc -= COUNTS_PER_CELL;
			R_acc -= COUNTS_PER_CELL;
			flood(exp_maze, target_x, target_y);
		}
		// set middle wall
		if (L_acc >= 150 && L_acc <= 200 && R_acc >= 150 && R_acc <= 200){
			if(measurements[1] < 200){
				add_wall(exp_maze, Mouse.current_cell_x-1, Mouse.current_cell_y, WEST);
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
			set_explored(exp_maze, Mouse.current_cell_x, Mouse.current_cell_y);
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

void save_maze(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT]){

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
void dlog(){
	static uint32_t address = 0x08030000;
	// Lacc, R_acc, heading, time
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, L_acc);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+4, R_acc);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+8, Mouse.heading);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+12, HAL_GetTick());
	address += 0x10;
}
Direction rel_to_fixed_dir(Relative_Direction mouse_dir){
	return ((Mouse.heading)+ mouse_dir + 8)%8;
}
void rem_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, uint8_t dir) {
    if ((maze[x][y].walls & 0xF0) == 0) {
        maze[x][y].walls &= ~(1 << (dir/2));
        if (dir == NORTH && y + 1 < MAZE_CELL_HEIGHT) {
            maze[x][y + 1].walls &= ~(1 << (SOUTH/2));
        } else if (dir == EAST && x + 1 < MAZE_CELL_WIDTH) {
            maze[x + 1][y].walls &= ~(1 << (WEST/2));
        } else if (dir == SOUTH && y > 0) {
            maze[x][y - 1].walls &= ~(1 << (NORTH/2));
        } else if (dir == WEST && x > 0) {
            maze[x - 1][y].walls &= ~(1 << (EAST/2));
        }
    }
}
void add_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, uint8_t dir) {
	if (y < MAZE_CELL_HEIGHT && y >= 0 && x < MAZE_CELL_WIDTH && x >= 0){
		if ((maze[x][y].walls & 0xF0) == 0) {
			maze[x][y].walls |= (0b01 << (dir)/2);
			if (dir == NORTH) {
				if (y + 1 < MAZE_CELL_HEIGHT) {
					if ((maze[x][y+1].walls & 0xF0) == 0) maze[x][y + 1].walls |= (0b01 << (SOUTH/2));
				}
			} else if (dir == EAST) {
				if (x + 1 < MAZE_CELL_WIDTH) {
					if ((maze[x+1][y].walls & 0xF0) == 0)maze[x + 1][y].walls |= (0b01 << (WEST/2));
				}
			} else if (dir == SOUTH) {
				if (y > 0) {
					if ((maze[x][y-1].walls & 0xF0) == 0)maze[x][y - 1].walls |= (0b01 << (NORTH/2));
				}
			} else if (dir == WEST) {
				if (x > 0) {
					if ((maze[x-1][y].walls & 0xF0) == 0)maze[x - 1][y].walls |= (0b01 << (EAST/2));
				}
			}
		}
	}
}
uint8_t read_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, Direction dir){
	if ((maze[x][y].walls & (0x01<<(dir/2)))==0){
		return 0;
	}
	else return 1;
}

void set_explored(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y) {
    if (x >= 0 && x < MAZE_CELL_WIDTH && y >= 0 && y < MAZE_CELL_HEIGHT) {
        maze[x][y].walls |= 0xF0;
    }
}
uint8_t get_explored(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y) {
    if (x >= 0 && x < MAZE_CELL_WIDTH && y >= 0 && y < MAZE_CELL_HEIGHT) {
        return (maze[x][y].walls & 0xF0) != 0;
    }
    return 1;
}
uint8_t dir_of_lowest(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y) {
	uint8_t min = 255;
	uint8_t dir = NORTH;
    if (read_wall(maze, x, y, NORTH) == 0) {
        if (maze[x][y + 1].dist < min) {
            min = maze[x][y + 1].dist;
            dir = NORTH;
        }
    }
    if (read_wall(maze, x, y, EAST) == 0) {
        if (maze[x + 1][y].dist < min) {
            min = maze[x + 1][y].dist;
            dir = EAST;
        }
    }
    if (read_wall(maze, x, y, SOUTH) == 0) {
        if (maze[x][y - 1].dist < min) {
            min = maze[x][y - 1].dist;
            dir = SOUTH;
        }
    }
    if (read_wall(maze, x, y, WEST) == 0) {
        if (maze[x - 1][y].dist < min) {
            min = maze[x - 1][y].dist;
            dir = WEST;
        }
    }
    return dir;
}
void flood(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t ex, uint8_t ey) {
	uint8_t nochange_flag = 0;
    maze[ex][ey].dist = 0;

    while (nochange_flag == 0) {
    	uint8_t change_flag = 0;

        for (uint8_t x = 0; x < MAZE_CELL_WIDTH; x++) {
            for (uint8_t y = 0; y < MAZE_CELL_HEIGHT; y++) {
                if (!(x == ex && y == ey)) {
                	uint8_t min = MAZE_CELL_HEIGHT * MAZE_CELL_WIDTH -1;
                    if (read_wall(maze, x, y, NORTH) == 0) {
                        if (maze[x][y + 1].dist < min) {
                            min = maze[x][y + 1].dist;
                        }
                    }
                    if (read_wall(maze, x, y, EAST) == 0) {
                        if (maze[x + 1][y].dist < min) {
                            min = maze[x + 1][y].dist;
                        }
                    }
                    if (read_wall(maze, x, y, SOUTH) == 0) {
                        if (maze[x][y - 1].dist < min) {
                            min = maze[x][y - 1].dist;
                        }
                    }
                    if (read_wall(maze, x, y, WEST) == 0) {
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
    }
}
uint8_t get_mid_direction(uint8_t a , uint8_t b){
    if ((a==0 && b ==2) || (a==2 && b ==0)) return 1;
    else if ((a==2 && b ==4) || (a==4 && b ==2)) return 3;
    else if ((a==4 && b ==6) || (a==6 && b ==4)) return 5;
    else if ((a==6 && b ==0) || (a==0 && b ==6)) return 7;
    return 0;
}
void init_race_maze(){ //copy across to race maze, blocking off unexplored cells
	for (int i = 0; i < MAZE_CELL_WIDTH; i++){
		for (int j = 0; j < MAZE_CELL_HEIGHT; j++){
			if (get_explored(exp_maze, i, j)==0){
				race_maze[i][j].walls = 0xF;
			}
			else{
				race_maze[i][j].walls = exp_maze[i][j].walls;
			}
			race_maze[i][j].dist = exp_maze[i][j].dist;
		}
	}
	flood(race_maze, END_CELL_X, END_CELL_Y);
}
Path get_shortest_path(){
	uint8_t x = 0;
	uint8_t y = 0;
	Path temp;
	temp.len = 0;

	while (!(x==END_CELL_X && y == END_CELL_Y)){
		uint8_t p = dir_of_lowest(race_maze, x, y);
		temp.direction[temp.len] = p;
		temp.distance[temp.len] = 1;
		temp.len++;
		if (p == NORTH) y++;
		else if (p == EAST) x++;
		else if (p == SOUTH) y--;
		else if (p == WEST) x--;
	}
	return temp;
}
Path detect_diagonals(Path p){
	Path temp;
	temp.len = 0;
	uint8_t j = 0;
	while(j < p.len){
		uint8_t diag[MAX_PATH_LENGTH] = {0};
		uint8_t diag_len = 0;
		uint8_t diag_flag = 0;
		uint8_t hcounts = 2;
		diag[diag_len] = p.direction[j];
		diag_len++;

		if (j < p.len-2){
			if ((p.direction[j] == p.direction[j+2])&& !(p.direction[j] == p.direction[j+1])){
				diag_flag = 1;
				diag[diag_len] = p.direction[j+1];
				diag_len++;
				diag[diag_len] = p.direction[j+2];
				diag_len++;

				uint8_t i = j+3;
				while(i < p.len){
					if ((p.direction[i] == p.direction[i-2])){
						hcounts++;
						diag[diag_len] = p.direction[i];
						diag_len++;
						i++;
					}
					else break;
				}
			}
		}
		if (diag_flag == 0){
			temp.direction[temp.len] = p.direction[j];
			temp.distance[temp.len] = 2;
			temp.len++;
			j++;
		}
		else{
			temp.direction[temp.len] = diag[0];
			temp.distance[temp.len] = 1;
			temp.len++;

			temp.direction[temp.len] = get_mid_direction(diag[0], diag[1]);
			temp.distance[temp.len] = hcounts;
			temp.len++;

			temp.direction[temp.len] = diag[diag_len-1];
			temp.distance[temp.len] = 1;
			temp.len++;

			j+= hcounts + 1;
		}
	}

	return temp;
}
Path compress_path(Path p){
	Path temp;
	temp.len = 0;
	uint8_t i = 0;
	uint8_t sum = 0;
	while (i < p.len){
		sum = p.distance[i];
		uint8_t j = i+1;
		while (j< p.len){
			if (p.direction[j] == p.direction[j - 1]){
				sum += p.distance[j];
				j++;
			}
			else break;
		}
		temp.direction[temp.len] = p.direction[i];
		temp.distance[temp.len] = sum;
		temp.len++;
		i=j;
	}
	return temp;
}
uint16_t score_path(Path p){
	uint16_t score = 5*p.len;
	for (int i = 0; i < p.len; i++) {
		score += p.distance[i];
	}
	return score;
}
void race(){
	turn(180);
	reverse(-150);

	init_race_maze();
	print_maze(race_maze);
	paths[0] = get_shortest_path();
	printpath(paths[0]);
	paths[0] = detect_diagonals(paths[0]);
	printpath(paths[0]);
	paths[0] = compress_path(paths[0]);
	printpath(paths[0]);
	sprintf(send_buffer, "%d\n\n", score_path(paths[0]));
	uart_transmit(send_buffer, strlen(send_buffer));
	HAL_Delay(1);
	for (int i = 0; i<paths[0].len; i++){
		uint8_t dir = paths[0].direction[i];
		turn_to_direction(dir);
		HAL_Delay(200);
		uint16_t mm = 0;
		if ((dir == 1)||(dir == 3)||(dir == 5)||(dir == 7)){
			mm = (int)((paths[0].distance[i]*COUNTS_PER_CELL*1.4142)/2);
		}
		else {
			mm = (int)((paths[0].distance[i]*COUNTS_PER_CELL)/2);
		}
		if (i==1) mm+=30;
		race_forward(mm);
		HAL_Delay(200);
	}
}








