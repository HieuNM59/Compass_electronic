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

#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

float Driffval = 0;


void mainInit(void){
	DWT_Delay_Init();
	Mpu6050_Init(&hi2c1, &htim2);
}

void mainProcess(void){

	buttonHoldScan();

	if(g_buttonEvent == HOLD_3S){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
		Driffval = getAntiDriffCoefficient(1000);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	}

}
