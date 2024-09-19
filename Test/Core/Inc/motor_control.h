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
void turn(int16_t deg);//dist in mm
void R_motor_feedback_control(uint8_t kick);
void L_motor_feedback_control(uint8_t kick);
void reset_counts();


#endif /* INC_MOTOR_CONTROL_H_ */
