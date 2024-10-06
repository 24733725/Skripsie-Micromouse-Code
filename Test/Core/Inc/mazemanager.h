/*
 * mazemanager.h
 *
 *  Created on: 17 Sep 2024
 *      Author: jaron
 */

#ifndef INC_MAZEMANAGER_H_
#define INC_MAZEMANAGER_H_

#include "globals.h"
#include "inttypes.h"

void maze_init();
void explore();
void update();
void save_maze();
void print_maze();
Direction rel_to_fixed_dir(Relative_Direction mouse_dir);
void rem_wall(uint8_t x, uint8_t y, Direction dir);
void add_wall(uint8_t x, uint8_t y, uint8_t dir);
uint8_t read_wall(uint8_t x, uint8_t y, Direction dir);
void set_explored(uint8_t x, uint8_t y);
uint8_t get_explored(uint8_t x, uint8_t y);
uint8_t dir_of_lowest(uint8_t x, uint8_t y);
void flood(uint8_t ex, uint8_t ey);
#endif /* INC_MAZEMANAGER_H_ */
