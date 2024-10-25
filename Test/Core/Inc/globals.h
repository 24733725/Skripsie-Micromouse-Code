/*
 * globals.h
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "inttypes.h"

#define STR_CONTROL_LOOP_PERIOD_MS 10
#define TURN_CONTROL_LOOP_PERIOD_MS 10
#define RACE_CONTROL_LOOP_PERIOD_MS 5
#define COUNTS_PER_ROTATION 120
#define WHEEL_DIAMETER_MM 33
#define WHEEL_SPACING_MM 81
#define PI 3.14159

//straight line
#define L_Kp 50
//#define L_Kd 0.01
#define L_Ki 0.5
#define L_Kff 0.475
#define L_ff_offset 125

#define R_Kp 50
//#define R_Kd 0.01
#define R_Ki 0.5
#define R_Kff 0.5
#define R_ff_offset 125
#define K_pdisterror 10
//turning
#define L_Kpt 17
#define R_Kpt 16
#define L_Kdt 0.5
#define R_Kdt 0.5
#define R_ff_offset_t 125
#define L_ff_offset_t 125
#define Enc_Turn_Error 1

//smooth stop
#define L_Kpss 3
#define R_Kpss 3
#define L_Kdss 0.5
#define R_Kdss 0.5
#define Enc_SS_Error 1

#define K_kick 24

//RACE
#define Enc_Str_Error 2
#define L_KpR 20
#define R_KpR 20
#define L_KdR 1.5
#define R_KdR 1.5
#define MAX_POWER 350
#define R_ff_offset_R 125
#define L_ff_offset_R 125
#define KeR 8
#define K_kickR 40

#define TOF_ADDRESS 0x29<<1

#define MAZE_CELL_WIDTH 6
#define MAZE_CELL_HEIGHT 6
#define END_CELL_X 5
#define END_CELL_Y 0

#define COUNTS_PER_CELL 208

#define MAX_PATH_LENGTH 256
#define MAX_PATHS 16

typedef struct {
	uint8_t walls;  // binary - 0000 1111 last 1st bit: explored y/n and 4 bits:walls from top clockwise
	uint8_t dist;
}Cell;
typedef enum {
    NORTH = 0,
    NE = 1,
    EAST = 2,
    SE = 3,
    SOUTH = 4,
    SW = 5,
    WEST = 6,
    NW = 7,
} Direction;
typedef enum {
    STRAIGHT = 0,
    SR = 1,
    RIGHT = 2,
    RB = 3,
    BACK = 4,
    LB = 5,
    LEFT = 6,
    SL = 7,
} Relative_Direction;
typedef struct {
    uint8_t heading;
    uint8_t current_cell_x;
    uint8_t current_cell_y;
} MouseStruct;

typedef struct {
	uint8_t direction[MAX_PATH_LENGTH];
	uint8_t distance[MAX_PATH_LENGTH];
	uint8_t len;
}Path;
#endif /* INC_GLOBALS_H_ */
