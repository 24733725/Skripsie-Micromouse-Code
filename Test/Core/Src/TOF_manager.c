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
char i2c_buff[128];
extern uint8_t measurements[3]; //L:M:R


void TOF_init(){
	//default settings
	writeMM(hi2c1);
	writeMM(hi2c2);
	writeMM(hi2c3);
	//begin continuous ranging
	i2c_buff[0] = 0x01;
	HAL_Delay(50);
	//Left
	HAL_I2C_Mem_Write(&hi2c2, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//middle
	HAL_I2C_Mem_Write(&hi2c1, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//right
	HAL_I2C_Mem_Write(&hi2c3, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
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
//	i2c_buff[0] = 0x10;
//	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x0011, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000); // Enables polling for ‘New Sample ready’
//	// when measurement completes
//	HAL_Delay(1);
//	i2c_buff[0] = 0x09;// Set default ranging inter-measurement
//	// period to 100ms
//	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x001b, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
//	HAL_Delay(1);
//	i2c_buff[0] = 0x14;// Set max convergence time to
//	HAL_I2C_Mem_Write(&a, TOF_ADDRESS, 0x001c, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
}

void TOF_task(){
	i2c_buff[0] = 0x01;
	HAL_Delay(50);
	//Left
	HAL_I2C_Mem_Write(&hi2c2, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//middle
	HAL_I2C_Mem_Write(&hi2c1, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	//right
	HAL_I2C_Mem_Write(&hi2c3, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	uint8_t status;
	uint8_t range_status;

	// wait for new measurement ready status
	HAL_I2C_Mem_Read(&hi2c2, TOF_ADDRESS, 0x04f, I2C_MEMADD_SIZE_16BIT, &status, 1, 10);
	range_status = status & 0x07;
//	if (range_status == 0x04){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//Left
		HAL_I2C_Mem_Read_IT(&hi2c2, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, measurements, 1);
		//middle
		HAL_I2C_Mem_Read_IT(&hi2c1, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &measurements[1], 1);
		//right
		HAL_I2C_Mem_Read_IT(&hi2c3, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &measurements[2], 1);

		//clear interrupt
		i2c_buff[0] = 0x07;
		HAL_I2C_Mem_Write_IT(&hi2c2, TOF_ADDRESS, 0x015, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);
		//middle
		HAL_I2C_Mem_Write_IT(&hi2c1, TOF_ADDRESS, 0x015, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);
		//right
		HAL_I2C_Mem_Write_IT(&hi2c3, TOF_ADDRESS, 0x015, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1);

//	}
}

void TOF_get_measurment(){ //single shot
	i2c_buff[0] = 0x01;
	uint8_t range = 0;
	//Left
	HAL_I2C_Mem_Write(&hi2c2, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c2, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &range, 1, 1000);
	measurements[0] = range;
	//middle
	HAL_I2C_Mem_Write(&hi2c1, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &range, 1, 1000);
	measurements[1] = range;
	//right
	HAL_I2C_Mem_Write(&hi2c3, TOF_ADDRESS, 0x018, I2C_MEMADD_SIZE_16BIT, (uint8_t*)i2c_buff, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c3, TOF_ADDRESS, 0x062, I2C_MEMADD_SIZE_16BIT, &range, 1, 1000);
	measurements[2] = range;
}

// Recommended : Public registers - See data sheet for more detail
//WriteByte(0x0011, 0x10); // Enables polling for ‘New Sample ready’
//// when measurement completes
//WriteByte(0x010a, 0x30); // Set the averaging sample period
//// (compromise between lower noise and
//// increased execution time)
//WriteByte(0x003f, 0x46); // Sets the light and dark gain (upper
//// nibble). Dark gain should not be
//// changed.
//WriteByte(0x0031, 0xFF); // sets the # of range measurements after
//// which auto calibration of system is
//// performed
//WriteByte(0x0040, 0x63); // Set ALS integration time to 100ms
//
//WriteByte(0x002e, 0x01); // perform a single temperature calibration
//// of the ranging sensor
//Optional: Public registers - See data sheet for more detail
//WriteByte(0x001b, 0x09); // Set default ranging inter-measurement
//// period to 100ms
//WriteByte(0x003e, 0x31); // Set default ALS inter-measurement period
//// to 500ms
//WriteByte(0x0014, 0x24); // Configures interrupt on ‘New Sample
//// Ready threshold event’
