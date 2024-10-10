/*
 * globals.h
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "inttypes.h"

#define STR_CONTROL_LOOP_PERIOD_MS 20
#define TURN_CONTROL_LOOP_PERIOD_MS 10
#define COUNTS_PER_ROTATION 120
#define WHEEL_DIAMETER_MM 33
#define WHEEL_SPACING_MM 81
#define PI 3.14159

//straight line
#define L_Kp 50
//#define L_Kd 0.01
#define L_Ki 1
#define L_Kff 0.475
#define L_ff_offset 125

#define R_Kp 50
//#define R_Kd 0.01
#define R_Ki 1
#define R_Kff 0.5
#define R_ff_offset 125

//turning
#define L_Kpt 15
#define R_Kpt 15
#define L_Kdt 0.5
#define R_Kdt 0.5
#define R_ff_offset_t 125
#define L_ff_offset_t 125
#define Enc_Turn_Error 2

//smooth stop
#define L_Kpss 2.9
#define R_Kpss 3.9
#define L_Kdss 0.32
#define R_Kdss 0.3
#define Enc_SS_Error 1

#define K_kick 25

#define TOF_ADDRESS 0x29<<1

#define MAZE_CELL_WIDTH 6
#define MAZE_CELL_HEIGHT 6
#define END_CELL_X 5
#define END_CELL_Y 0

#define COUNTS_PER_CELL 208

typedef struct {
	uint8_t walls;  // binary - 0000 1111 last 1st bit: explored y/n and 4 bits:walls from top clockwise
	uint8_t dist;
}Cell;
typedef enum {
	NORTH = 0,
	EAST = 1,
	SOUTH = 2,
	WEST = 3,
} Direction;
typedef enum {
	STRAIGHT = 0,
	RIGHT = 1,
	BACK = 2,
	LEFT = 3,
} Relative_Direction;
typedef struct {
    uint8_t heading;
    uint8_t current_cell_x;
    uint8_t current_cell_y;
} MouseStruct;
#endif /* INC_GLOBALS_H_ */
