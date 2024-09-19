/*
 * globals.h
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "inttypes.h"

#define CONTROL_LOOP_PERIOD_MS 20
#define COUNTS_PER_ROTATION 120
#define WHEEL_DIAMETER_MM 33
#define WHEEL_SPACING_MM 81
#define PI 3.14159

#define L_Kpt 5
#define L_Kp 25
#define L_Kd 0.01
#define L_Ki 1
#define L_Kff 0.475
#define L_ff_offset 120

#define R_Kpt 5
#define R_Kp 25
#define R_Kd 0.01
#define R_Ki 1
#define R_Kff 0.5
#define R_ff_offset 120

#define K_pspeederror 10
#define K_pdisterror 10
#define K_idisterror 0

#define TOF_ADDRESS 0x29<<1

#define MAZE_CELL_WIDTH 6
#define MAZE_CELL_HEIGHT 13
typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t walls;
	uint8_t dist;
}Cell;

#endif /* INC_GLOBALS_H_ */
