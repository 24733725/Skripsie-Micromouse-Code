/*
 * TOF_manager.h
 *
 *  Created on: Aug 30, 2024
 *      Author: jaron
 */
#include "inttypes.h"
#include "stm32f4xx_hal.h"
#ifndef INC_TOF_MANAGER_H_
#define INC_TOF_MANAGER_H_

void TOF_init();
void TOF_task();
void writeMM(I2C_HandleTypeDef a);
void TOF_get_measurement();
void TOF_start_measurement();


#endif /* INC_TOF_MANAGER_H_ */
