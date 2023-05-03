/*
 * MPU6050.h
 *
 *  Created on: Apr 27, 2023
 *      Author: LumiQA
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx.h"

#define   SMPLRT_DIV      	0x19   //0x07(125Hz)
#define   CONFIG         	0x1A   //0x06(5Hz)
#define   GYRO_CONFIG      	0x1B   //0x18(���Լ죬2000deg/s)
#define   ACCEL_CONFIG   	0x1C   //0x01(���Լ죬2G��5Hz)
#define   ACCEL_XOUT_H   	0x3B
#define   ACCEL_XOUT_L   	0x3C
#define   ACCEL_YOUT_H   	0x3D
#define   ACCEL_YOUT_L   	0x3E
#define   ACCEL_ZOUT_H   	0x3F
#define   ACCEL_ZOUT_L   	0x40
#define   TEMP_OUT_H      	0x41
#define   TEMP_OUT_L      	0x42
#define   GYRO_XOUT_H      	0x43
#define   GYRO_XOUT_L      	0x44
#define   GYRO_YOUT_H      	0x45
#define   GYRO_YOUT_L      	0x46
#define   GYRO_ZOUT_H      	0x47
#define   GYRO_ZOUT_L      	0x48
#define   PWR_MGMT_1      	0x6B   //
#define   WHO_AM_I         	0x75   //
#define   INT_ENABLE 		0x38
#define   mpu6050 			0x68

typedef enum _AXIS_{
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
}axis_e;

typedef struct {
	int x_Axis;
	int y_Axis;
	int z_Axis;
}driff_t;

extern int16_t xAngle, yAngle, zAngle;
void Mpu6050_Init(I2C_HandleTypeDef* mpuInitstructure, TIM_HandleTypeDef* timInitstructure);

void MPU6050_Reset(void);

int16_t GetData(unsigned char address);

void readdata ();

void Mpu6050_Write(uint8_t adress,uint8_t data);

unsigned char Mpu6050_Read(uint8_t adress);

float getAntiDriffCoefficient(uint8_t numSample, axis_e axis);

#endif /* MPU6050_H_ */
