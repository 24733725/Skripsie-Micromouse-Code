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
void forward(int16_t power);
void dist(int16_t maxpower, uint16_t dist);//dist in mm
void R_motor_feedback_control();
void L_motor_feedback_control();

#endif /* INC_MOTOR_CONTROL_H_ */
