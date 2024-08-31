/*
 * globals.h
 *
 *  Created on: Aug 8, 2024
 *      Author: jaron
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#define CONTROL_LOOP_PERIOD_MS 20
#define COUNTS_PER_ROTATION 120
#define WHEEL_DIAMETER_MM 33
#define PI 3.14159

#define L_Kp 1
#define L_Kd 0.01
#define L_Ki 0
#define L_Kff 0.494
#define L_ff_offset 155.3
#define R_Kp 1
#define R_Kd 0.01
#define R_Ki 0
#define R_Kff 0.43
#define R_ff_offset 155

#define K_pdisterror 3
#define K_idisterror 0.1

#define TOF_ADDRESS 0x29<<1


#endif /* INC_GLOBALS_H_ */
