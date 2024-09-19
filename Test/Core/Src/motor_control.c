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
#include "uart_driver.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

extern int16_t L_speed_setpoint; //mm/s
extern int16_t R_speed_setpoint;//mm/s
extern int16_t L_prev_enc_count;
extern int16_t R_prev_enc_count;
extern int32_t L_ctrl_signal;
extern int32_t R_ctrl_signal;
extern int32_t L_error;
extern int32_t R_error;
extern int32_t L_acc_error;
extern int32_t R_acc_error;
extern int32_t L_acc;
extern int32_t R_acc;
extern int32_t Dist_error_acc;

extern uint8_t measurements[3]; //L:M:R
extern char send_buffer[64];
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
	  htim3.Instance->CNT = 0;
	  htim5.Instance->CNT = 0;
}
void reset_counts(){
	  htim3.Instance->CNT = 0;
	  htim5.Instance->CNT = 0;
//	  L_speed_setpoint = 0; //mm/s
//	  R_speed_setpoint = 0;//mm/s
	  L_prev_enc_count = 0;
	  R_prev_enc_count = 0;
	  L_ctrl_signal = 0;
	  R_ctrl_signal = 0;
	  L_error = 0;
	  R_error = 0;
	  L_acc_error = 0;
	  R_acc_error = 0;
	  L_acc = 0;
	  R_acc = 0;
	  Dist_error_acc = 0;
	  //Motor 1
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  //Motor 2
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}
void move(int16_t velocity, int16_t omega){ // velocity in mm/s, omega in deg/s
	L_speed_setpoint = velocity + (WHEEL_SPACING_MM*omega*PI)/(2*180); //mm/s
	R_speed_setpoint = velocity - (WHEEL_SPACING_MM*omega*PI)/(2*180);//mm/s

	uint32_t prev_ctr_loop_time = HAL_GetTick();

	while(measurements[1]>190 && velocity != 0){
		if (HAL_GetTick() - prev_ctr_loop_time > CONTROL_LOOP_PERIOD_MS){
			prev_ctr_loop_time = HAL_GetTick();
			R_motor_feedback_control();
			L_motor_feedback_control();
		}
	}
	reset_counts();
}
void turn(int16_t deg){
	reset_counts();
	int16_t L_count_target = (WHEEL_SPACING_MM*deg*COUNTS_PER_ROTATION)/(WHEEL_DIAMETER_MM*360);
	int16_t R_count_target = -(WHEEL_SPACING_MM*deg*COUNTS_PER_ROTATION)/(WHEEL_DIAMETER_MM*360);

	uint32_t prev_ctr_loop_time = HAL_GetTick();
	uint8_t turn_cmplt = 0;
	while(turn_cmplt == 0){
		if (HAL_GetTick() - prev_ctr_loop_time > CONTROL_LOOP_PERIOD_MS){
			prev_ctr_loop_time = HAL_GetTick();
			R_prev_enc_count = htim3.Instance->CNT;
			L_prev_enc_count = htim5.Instance->CNT;
//			R_motor_feedback_control();
			R_error = R_count_target - R_prev_enc_count;
			R_ctrl_signal = R_Kpt*R_error;
			if (R_error > 0) R_ctrl_signal += R_ff_offset;
			if (R_error < 0) R_ctrl_signal -= R_ff_offset;

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


//			L_motor_feedback_control();
			L_error = L_count_target - L_prev_enc_count;
			L_ctrl_signal = L_Kpt*L_error;
			if (L_error > 0) L_ctrl_signal += L_ff_offset;
			if (L_error < 0) L_ctrl_signal -= L_ff_offset;

			if (L_ctrl_signal>=1000) L_ctrl_signal = 999;
			if (L_ctrl_signal<=-1000) L_ctrl_signal = -999;

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
			sprintf(send_buffer, "L:%d > %d R:%d > %d\n",(int)L_error,(int)L_ctrl_signal,(int)R_error, (int)R_ctrl_signal);
			uart_transmit(send_buffer, strlen(send_buffer));
			if (L_error < 3 && L_error > -3 && R_error < 3 && R_error > -3) turn_cmplt =1;
		}
	}
	reset_counts();
}
void R_motor_feedback_control(){//speed in mm/s
	Dist_error_acc += L_acc - R_acc;
	R_prev_enc_count = htim3.Instance->CNT;
	R_acc += R_prev_enc_count;

	//error in encoder count for that ctrl period
	R_error = (int)((R_speed_setpoint*COUNTS_PER_ROTATION*CONTROL_LOOP_PERIOD_MS)/(WHEEL_DIAMETER_MM*PI*1000)) - R_prev_enc_count;

	R_acc_error += R_error;
	if(R_acc_error > 1000) R_acc_error = 1000;
	if(R_acc_error < -1000) R_acc_error = -1000;  //limits integral term

//					Proportional  		Integral		  FeedForward 				proportional distance error   integral distance error
	R_ctrl_signal = R_Kp*R_error + R_Ki*R_acc_error + R_Kff*R_speed_setpoint;// + K_pdisterror*(L_acc-R_acc) + K_idisterror*Dist_error_acc;
	if (R_speed_setpoint > 0) R_ctrl_signal += R_ff_offset;
	if (R_speed_setpoint < 0) R_ctrl_signal -= R_ff_offset;

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
	htim3.Instance->CNT = 0;
}

void L_motor_feedback_control(){//speed in mm/s
	L_prev_enc_count = htim5.Instance->CNT;
	L_acc += L_prev_enc_count;
	//error in encoder count for that ctrl period
	L_error = (int)((L_speed_setpoint*COUNTS_PER_ROTATION*CONTROL_LOOP_PERIOD_MS)/(WHEEL_DIAMETER_MM*PI*1000)) - L_prev_enc_count;

	L_acc_error += L_error;
	if(L_acc_error > 1000) L_acc_error = 1000;
	if(L_acc_error < -1000) L_acc_error = -1000;  //limits integral term

//					Proportional  		Integral		  FeedForward
	L_ctrl_signal = L_Kp*L_error + L_Ki*L_acc_error + L_Kff*L_speed_setpoint;//K_pdisterror*(R_acc-L_acc) - K_idisterror*Dist_error_acc;
	if (L_speed_setpoint > 0) L_ctrl_signal += L_ff_offset;
	if (L_speed_setpoint < 0) L_ctrl_signal -= L_ff_offset;


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
	htim5.Instance->CNT = 0;
}
