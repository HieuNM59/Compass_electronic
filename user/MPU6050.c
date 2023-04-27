/*
 * MPU6050.c
 *
 *  Created on: Apr 27, 2023
 *      Author: LumiQA
 */


#include "MPU6050.h"
#include "dwt_stm32_delay.h"

I2C_HandleTypeDef* mpu_handle;
TIM_HandleTypeDef* tim_handle;


int16_t TP_X,TP_Y,TP_Z;
int16_t G_X,G_Y,G_Z;
float mau[100];
int16_t angle;
uint8_t l=0,h=0;
volatile int16_t s,j,a,hi;
uint16_t k;

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

float getAntiDriffCoefficient(uint8_t numSample){
	int byteL, byteH;

	int16_t sumAntiDriffVal = 0;
	float avgDriff = 0;
	HAL_TIM_Base_Stop_IT(tim_handle);
	for(uint8_t i_cnt = 0; i_cnt < numSample; i_cnt++){
		byteH = Mpu6050_Read(GYRO_ZOUT_H);
		byteL = Mpu6050_Read(GYRO_ZOUT_L);
		sumAntiDriffVal +=  ((byteH << 8) | byteL);
		HAL_Delay(1);
	}
	avgDriff = (float)sumAntiDriffVal / numSample;
//	HAL_TIM_Base_Start_IT(tim_handle);
	return avgDriff;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if((htim->Instance)==tim_handle->Instance)
	{
		int H,L;
		H=Mpu6050_Read(GYRO_ZOUT_H);
		L=Mpu6050_Read(GYRO_ZOUT_L);
		hi=(H<<8)|L;
		a=(hi+43)/1.64;			//he so chong troi
		mau[0] = mau[0]+(a*0.2);
		s=mau[0];
		k=-s;
		h=k/256;
		l=k%256;
		angle=h<<8|l;
	}
}
