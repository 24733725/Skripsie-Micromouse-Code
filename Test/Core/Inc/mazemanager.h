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
void go_home();
void race();
void update();
void save_maze(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT]);
void dlog();
void print_maze(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT]);
Direction rel_to_fixed_dir(Relative_Direction mouse_dir);
void rem_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, Direction dir);
void add_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, uint8_t dir);
uint8_t read_wall(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y, Direction dir);
uint8_t read_left_wall(uint8_t x, uint8_t y);
uint8_t read_right_wall(uint8_t x, uint8_t y);
void set_explored(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y);
uint8_t get_explored(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y);
uint8_t dir_of_lowest(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t x, uint8_t y);
void flood(Cell maze[MAZE_CELL_WIDTH][MAZE_CELL_HEIGHT], uint8_t ex, uint8_t ey);
uint8_t get_mid_direction(uint8_t a , uint8_t b);
void init_race_maze();
Path get_shortest_path();
Path detect_diagonals(Path p);
Path compress_path(Path p);
uint16_t score_path(Path p);
#endif /* INC_MAZEMANAGER_H_ */
