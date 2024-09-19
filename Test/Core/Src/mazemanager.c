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

extern uint8_t measurements[3]; //L:M:R
extern Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT];

void explore(){
	move(600,0);
	while(1){
		HAL_Delay(10);
		if (measurements[0]>190) turn(-90);
		else if (measurements[2]>190) turn(90);
		else turn(180);

		HAL_Delay(10);
		move(600,0);
	}
	HAL_Delay(10);
	turn(90);
}
