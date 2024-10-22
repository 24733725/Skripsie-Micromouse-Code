/*
 * MotorControl.h
 *
 *  Created on: Aug 13, 2024
 *      Author: jaron
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "inttypes.h"

void motorsInit();
void move(int16_t velocity, int16_t omega);
void reverse(int16_t velocity);
void smooth_turn_L();
void smooth_turn_R();
void turn(int16_t deg);
void R_motor_feedback_control(int8_t kick);
void L_motor_feedback_control(int8_t kick);
void reset_counts();
//void smooth_stop(uint16_t ms);
//void smooth_stop1(uint16_t ms);
void smooth_stop(uint16_t dist);
void race_forward(uint16_t mm);

#endif /* INC_MOTOR_CONTROL_H_ */
