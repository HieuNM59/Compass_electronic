/*
 * MPU6050.c
 *
 *  Created on: Apr 27, 2023
 *      Author: LumiQA
 */


#include "MPU6050.h"
#include "flash.h"
#include "dwt_stm32_delay.h"

I2C_HandleTypeDef* mpu_handle;
TIM_HandleTypeDef* tim_handle;

extern driff_t  driffVal;

float xLastVal, yLastVal, zLastVal;
int16_t xAngle, yAngle, zAngle;
volatile int16_t xS, yS, zS, xA, yA, zA, xVal, yVal, zVal;

void Mpu6050_Init(I2C_HandleTypeDef* mpuInitstructure, TIM_HandleTypeDef* timInitstructure)
{
	mpu_handle = mpuInitstructure;
	tim_handle = timInitstructure;

	//Mpu6050_Write(PWR_MGMT_1, 0x80);
	DWT_Delay_us(10);
	Mpu6050_Write(PWR_MGMT_1, 0x00);  // internal 8MHz, disabled SLEEP mode, disable CYCLE mode
	DWT_Delay_us(10);
	Mpu6050_Write(SMPLRT_DIV, 0x07);	 //sample rate: 8khz
	DWT_Delay_us(10);
	Mpu6050_Write(CONFIG, 0x06);		 //DLPF disable
	DWT_Delay_us(10);
	Mpu6050_Write(GYRO_CONFIG, 0x18);  //full scale range mode 3 +-2000do/s
	DWT_Delay_us(10);
	Mpu6050_Write(ACCEL_CONFIG, 0x00); //full scale range mode 1 +-4g
	DWT_Delay_us(10);
	Mpu6050_Write(INT_ENABLE, 0x01);
	DWT_Delay_us(10);
	//Mpu6050_Write(PWR_MGMT_1, 0x01);
	HAL_TIM_Base_Start_IT(tim_handle);
}

void MPU6050_Reset(void){
	__NVIC_SystemReset();
}

void Mpu6050_Write(uint8_t adress,uint8_t data)
{
	unsigned char Buff[2];
	Buff[0] = adress;
	HAL_Delay(10);
	Buff[1] = data;
	HAL_Delay(10);
	while(HAL_I2C_Master_Transmit(mpu_handle, 0xD0, &Buff[0], 2, 2));
}
unsigned char Mpu6050_Read(uint8_t adress)
{
	uint8_t data;
	while(HAL_I2C_Master_Transmit(mpu_handle, 0xD0, &adress, 1, 100));
	while(HAL_I2C_Master_Receive(mpu_handle, 0xD1, &data, 1, 500));
	return data;
}

float getAntiDriffCoefficient(uint8_t numSample, axis_e axis){
	int byteL, byteH;

	int16_t sumAntiDriffVal = 0;
	float avgDriff = 0;
	HAL_TIM_Base_Stop_IT(tim_handle);

	switch(axis){
		case AXIS_X:
			for(uint8_t i_cnt = 0; i_cnt < numSample; i_cnt++){
				byteH = Mpu6050_Read(GYRO_XOUT_H);
				byteL = Mpu6050_Read(GYRO_XOUT_L);
				sumAntiDriffVal +=  ((byteH << 8) | byteL);
				HAL_Delay(5);
			}
			break;

		case AXIS_Y:
			for(uint8_t i_cnt = 0; i_cnt < numSample; i_cnt++){
				byteH = Mpu6050_Read(GYRO_YOUT_H);
				byteL = Mpu6050_Read(GYRO_YOUT_L);
				sumAntiDriffVal +=  ((byteH << 8) | byteL);
				HAL_Delay(5);
			}
			break;

		case AXIS_Z:
			for(uint8_t i_cnt = 0; i_cnt < numSample; i_cnt++){
				byteH = Mpu6050_Read(GYRO_ZOUT_H);
				byteL = Mpu6050_Read(GYRO_ZOUT_L);
				sumAntiDriffVal +=  ((byteH << 8) | byteL);
				HAL_Delay(5);
			}
			break;
		default:
			break;
	}

	avgDriff = sumAntiDriffVal / numSample;
	HAL_TIM_Base_Start_IT(tim_handle);
	return avgDriff;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	int zH,zL;
	int xH,xL;
	int yH,yL;
	if((htim->Instance)==tim_handle->Instance)
	{
		xH = Mpu6050_Read(GYRO_XOUT_H);
		xL = Mpu6050_Read(GYRO_XOUT_L);
		xVal = (xH << 8) | xL;
		xA = ( xVal + driffVal.x_Axis) / 1.64;			// he so chong troi
		xLastVal = xLastVal + (xA * 0.0199);				// 0.05 = timer 50ms Tinh theo fOSC = 36Mhz
		xAngle = xLastVal;								//

		yH = Mpu6050_Read(GYRO_YOUT_H);
		yL = Mpu6050_Read(GYRO_YOUT_L);
		yVal = (yH << 8) | yL;
		yA = ( yVal + driffVal.y_Axis) / 1.64;			//he so chong troi
		yLastVal = yLastVal + (yA * 0.0199);				// 0.05 = timer 50ms Tinh theo fOSC = 36Mhz
		yAngle = yLastVal;

		zH = Mpu6050_Read(GYRO_ZOUT_H);
		zL = Mpu6050_Read(GYRO_ZOUT_L);
		zVal = (zH << 8) | zL;
		zA = ( zVal + driffVal.z_Axis) / 1.64;			//he so chong troi
		zLastVal = zLastVal + (zA * 0.0199);				// 0.05 = timer 50ms Tinh theo fOSC = 36Mhz
		zAngle = zLastVal;
	}
}


