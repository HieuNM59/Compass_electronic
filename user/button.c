/*
 * button.c
 *
 *  Created on: Apr 28, 2023
 *      Author: LumiQA
 */

#include "button.h"
#include "stm32f1xx.h"

#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC

#define TIME_SCAN_BUTTON 	100
#define TIME_HOLD_2S		20
#define TIME_HOLD_3S		30


button_hold_event_e g_buttonEvent;

void buttonHoldScan(void){
	static uint32_t btnTicks = 0;
	static uint8_t btnCount = 0;
	if(HAL_GetTick() - btnTicks > TIME_SCAN_BUTTON){
		if(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
			btnCount++;
		}
		else{
			btnCount = 0;
		}

		if(btnCount > TIME_HOLD_3S){
			g_buttonEvent = HOLD_3S;
		}
		else{
			g_buttonEvent = IDLE;
		}
		btnTicks = HAL_GetTick();
	}
}
