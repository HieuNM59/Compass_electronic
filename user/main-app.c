/*
 * main-app.c
 *
 *  Created on: Apr 27, 2023
 *      Author: LumiQA
 */

#include "dwt_stm32_delay.h"
#include "main-app.h"
#include "MPU6050.h"
#include "button.h"
#include "flash.h"


#define SAVE_X_AXIS_ADDR	((uint32_t)0x0801FC00) // page 127
#define SAVE_Y_AXIS_ADDR	((uint32_t)(SAVE_X_AXIS_ADDR + 4)) // page 127
#define SAVE_Z_AXIS_ADDR	((uint32_t)(SAVE_Y_AXIS_ADDR + 4)) // page 127

#define TIME_SEND_ANGLE		50 //ms

#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

typedef enum {
	STATE_IDLE,
	GET_X_AXIS,
	GET_Y_AXIS,
	GET_Z_AXIS,
	RESET_MCU,
	CALIB,
}get_state_e;

uint8_t u8_Recv;
driff_t  driffVal;
get_state_e g_mcuPollState;
uint16_t g_timeBlinkLed = 1000;

void sendAngleToMain(void);
void ledBlink(void);

void ledBlink(void){
	static uint32_t lastBlinkTimes = 0;

	if(HAL_GetTick() - lastBlinkTimes >= g_timeBlinkLed){
		lastBlinkTimes = HAL_GetTick();
	}
	else{
		return;
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

}
void mainInit(void){
	DWT_Delay_Init();
	driffVal.x_Axis = Flash_Read_Int(SAVE_X_AXIS_ADDR);
	driffVal.y_Axis = Flash_Read_Int(SAVE_Y_AXIS_ADDR);
	driffVal.z_Axis = Flash_Read_Int(SAVE_Z_AXIS_ADDR);

	if((driffVal.x_Axis == 0xFFFF) || (driffVal.x_Axis == 0xFFFF) || (driffVal.z_Axis == 0xFFFF)){
		Mpu6050_Init(&hi2c1, &htim2);
		driffVal.x_Axis = getAntiDriffCoefficient(100, AXIS_X);
		driffVal.x_Axis = -driffVal.x_Axis;

		driffVal.y_Axis = getAntiDriffCoefficient(100, AXIS_Y);
		driffVal.y_Axis = -driffVal.y_Axis;

		driffVal.z_Axis = getAntiDriffCoefficient(100, AXIS_Z);
		driffVal.z_Axis = -driffVal.z_Axis;


		Flash_Erase(SAVE_X_AXIS_ADDR);
		Flash_Write_Int(SAVE_X_AXIS_ADDR, driffVal.x_Axis);
		Flash_Write_Int(SAVE_Y_AXIS_ADDR, driffVal.y_Axis);
		Flash_Write_Int(SAVE_Z_AXIS_ADDR, driffVal.z_Axis);
		NVIC_SystemReset();
	}
	else{
		Mpu6050_Init(&hi2c1, &htim2);
	}
	HAL_UART_Receive_IT(&huart1, &u8_Recv, 1);
}

void mainProcess(void){


	sendAngleToMain();

	buttonHoldScan();

	if(g_buttonEvent == HOLD_1S){
		g_timeBlinkLed = 100;
	}

	ledBlink();

	if(g_buttonEvent == HOLD_3S || g_mcuPollState == CALIB){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		if(g_mcuPollState == CALIB){
			HAL_Delay(1000);
		}
		else{
			HAL_Delay(5000);
		}

		driffVal.x_Axis = getAntiDriffCoefficient(100, AXIS_X);
		driffVal.x_Axis = -driffVal.x_Axis;

		driffVal.y_Axis = getAntiDriffCoefficient(100, AXIS_Y);
		driffVal.y_Axis = -driffVal.y_Axis;

		driffVal.z_Axis = getAntiDriffCoefficient(100, AXIS_Z);
		driffVal.z_Axis = -driffVal.z_Axis;


		Flash_Erase(SAVE_X_AXIS_ADDR);
		Flash_Write_Int(SAVE_X_AXIS_ADDR, driffVal.x_Axis);
		Flash_Write_Int(SAVE_Y_AXIS_ADDR, driffVal.y_Axis);
		Flash_Write_Int(SAVE_Z_AXIS_ADDR, driffVal.z_Axis);
		NVIC_SystemReset();
	}

}

void sendAngleToMain(void){
	static uint32_t lastTimes = 0;
	uint8_t payload[2];
	if(HAL_GetTick() - lastTimes > TIME_SEND_ANGLE){
		lastTimes = HAL_GetTick();
	}
	else{
		return;
	}
	switch (g_mcuPollState) {
		case GET_X_AXIS:
			g_timeBlinkLed = 150;
			payload[0] = xAngle & 0xFF;
			payload[1] = (xAngle >> 8) & 0xFF;
			HAL_UART_Transmit(&huart1, payload, 2, 100);
			break;

		case GET_Y_AXIS:
			g_timeBlinkLed = 150;
			payload[0] = yAngle & 0xFF;
			payload[1] = (yAngle >> 8) & 0xFF;
			HAL_UART_Transmit(&huart1, payload, 2, 100);
			break;

		case GET_Z_AXIS:
			g_timeBlinkLed = 150;
			payload[0] = zAngle & 0xFF;
			payload[1] = (zAngle >> 8) & 0xFF;
			HAL_UART_Transmit(&huart1, payload, 2, 100);
			break;
		default:
			g_timeBlinkLed = 1000;
			break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1)
	{
		switch(u8_Recv){
		case 'A':
			g_mcuPollState = RESET_MCU;
			NVIC_SystemReset();
			break;
		case 'X':
			g_mcuPollState = GET_X_AXIS;
			break;
		case 'Y':
			g_mcuPollState = GET_Y_AXIS;
			break;
		case 'Z':
			g_mcuPollState = GET_Z_AXIS;
			break;
		case 'F':
			g_mcuPollState = CALIB;
			break;
		default:
			g_mcuPollState = STATE_IDLE;
			break;
		}
		HAL_UART_Receive_IT(&huart1, &u8_Recv, 1);
	}
}
