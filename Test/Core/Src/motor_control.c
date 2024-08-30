/*
 * motor_control.c
 *
 *  Created on: Aug 13, 2024
 *      Author: jaron
 */
#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "inttypes.h"
#include "globals.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern int16_t L_speed_setpoint; //mm/s
extern int16_t R_speed_setpoint;//mm/s
extern int16_t L_prev_enc_count;
extern int16_t R_prev_enc_count;
extern int16_t L_ctrl_signal;
extern int16_t R_ctrl_signal;
extern int32_t L_error;
extern int32_t R_error;
extern int32_t L_acc_error;
extern int32_t R_acc_error;

void motorsInit(){

	  //Motor 1
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	  //Motor 2
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
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
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, power);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		//motor 2
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, power);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else{
//		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -power);
//		motor 2
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -power);
	}
}
void R_motor_feedback_control(){//speed in mm/s
	R_prev_enc_count = htim3.Instance->CNT;
	R_error = (int)(R_speed_setpoint - (WHEEL_DIAMETER_MM*PI*R_prev_enc_count*1000)/(COUNTS_PER_ROTATION*CONTROL_LOOP_PERIOD_MS));
	R_acc_error += R_error;

//					Proportional  		Integral		  FeedForward
	R_ctrl_signal = R_Kp*R_error + R_Ki*R_acc_error + R_Kff*R_speed_setpoint;

	if (R_ctrl_signal!=0) R_ctrl_signal+= R_ff_offset; //add offset if not standing still

	if (R_ctrl_signal >= 1000) R_ctrl_signal = 999;
	if (R_ctrl_signal <= -1000) R_ctrl_signal = -999;

	if (R_ctrl_signal == 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	}
	else if (R_ctrl_signal > 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, R_ctrl_signal);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	}
	else{
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -R_ctrl_signal);
	}
//	prev_control_signal = R_ctrl_signal;
	htim3.Instance->CNT = 0;
}
void L_motor_feedback_control(){//speed in mm/s
	L_prev_enc_count = htim5.Instance->CNT;
	L_error = (int)(L_speed_setpoint - (WHEEL_DIAMETER_MM*PI*L_prev_enc_count*1000)/(COUNTS_PER_ROTATION*CONTROL_LOOP_PERIOD_MS));
	L_acc_error += L_error;

	//					Proportional  		Integral		  FeedForward
	L_ctrl_signal = L_Kp*L_error + L_Ki*L_acc_error + L_Kff*L_speed_setpoint;
	if (L_ctrl_signal!=0) L_ctrl_signal += L_ff_offset; //add offset if not standing still

	if (L_ctrl_signal>1000) L_ctrl_signal = 999;
	if (L_ctrl_signal<-1000) L_ctrl_signal = -999;

	if (L_ctrl_signal == 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else if (L_ctrl_signal > 0){
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, L_ctrl_signal);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else{
		//motor 1
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -L_ctrl_signal);
	}
//	prev_control_signal = L_ctrl_signal;
	htim5.Instance->CNT = 0;
}

