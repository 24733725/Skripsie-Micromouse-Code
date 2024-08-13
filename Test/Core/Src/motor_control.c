/*
 * motor_control.c
 *
 *  Created on: Aug 13, 2024
 *      Author: jaron
 */
#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "inttypes.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;


void motorsInit(){

	  //Motor 1
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	  //Motor 2
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

	  //Encoder 1
	  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	  //Encoder 2
	  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

}
void forward(int16_t power){ // -1000 < power < 1000
	if (power>1000) power = 1000;
	if (power<-1000) power = -1000;

	if (power == 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		//motor 2
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else if (power > 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (int)(power*0.9));
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		//motor 2
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)(power*0.9));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else{
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (int)(-power*0.9));
		//motor 2
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (int)(-power*0.9));
	}
}

