/*
 * TOF_manager.c
 *
 *  Created on: Aug 30, 2024
 *      Author: jaron
 */
#include "TOF_manager.h"
#include "stm32f4xx_hal.h"

#define TOF_ADDRESS 0x29<<1

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
char i2c_buff[8];
extern uint8_t measurements[3]; //L:M:R
extern TIM_HandleTypeDef htim11;

void TOF_init(){
	//default settings
	writeMM(hi2c1);
	writeMM(hi2c2);
	writeMM(hi2c3);
	//begin continuous ranging 0x03, single 0x01
	i2c_buff[0] = 0x01;
	HAL_Delay(50);
	//Left
	HAL_I2C_Mem_Write(&hi2c2, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//middle
	HAL_I2C_Mem_Write(&hi2c1, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//right
	HAL_I2C_Mem_Write(&hi2c3, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);

	HAL_TIM_Base_Start_IT(&htim11);
}
void writeMM(I2C_HandleTypeDef a){ //default settings
	i2c_buff[0] = 0x01;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0207, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x01;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0208, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0096, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0xfd;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0097, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00e3, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x04;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00e4, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x02;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00e5, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x01;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00e6, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x03;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00e7, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x02;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00f5, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x05;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00d9, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0xce;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00db, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x03;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00dc, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0xf8;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00dd, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x009f, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x3c;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00a3, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00b7, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x3c;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00bb, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x09;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00b2, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x09;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00ca, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x01;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0198, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x17;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x01b0, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x01ad, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x05;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x00ff, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x05;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0100, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x05;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0199, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x1b;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x01a6, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x3e;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x01ac, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x1f;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x01a7, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	i2c_buff[0] = 0x00;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0030, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_Delay(1);
	//custom settings
	i2c_buff[0] = 0x10;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0011, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000); // Enables polling for ‘New Sample ready’
//	// when measurement completes
	HAL_Delay(1);
	i2c_buff[0] = 0x01;// Set default ranging inter-measurement
//	// period to 100ms
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x001b, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
//	HAL_Delay(1);
	i2c_buff[0] = 0xA;// Set max convergence time to 10ms
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x001c, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);

	i2c_buff[0] = 0x24;
	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0014, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
// Configures interrupt on ‘New Sample
	//// Ready threshold event’
}

void TOF_task(){

}
void TOF_start_measurement(){
	i2c_buff[0] = 0x01;
	//left
	HAL_I2C_Mem_Write_IT(&hi2c2, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);
	//middle
	HAL_I2C_Mem_Write_IT(&hi2c1, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);
	//right
	HAL_I2C_Mem_Write_IT(&hi2c3, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);
}
void TOF_get_measurement(){ //single shot

	//Left
	HAL_I2C_Mem_Read_IT(&hi2c2, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, measurements, 1);
	//middle
	HAL_I2C_Mem_Read_IT(&hi2c1, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &measurements[1], 1);
	//right
	HAL_I2C_Mem_Read_IT(&hi2c3, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &measurements[2], 1);
}

