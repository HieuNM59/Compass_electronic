/*
 * button.h
 *
 *  Created on: Apr 28, 2023
 *      Author: LumiQA
 */

#ifndef BUTTON_H_
#define BUTTON_H_

typedef enum {
	IDLE = 0,
	HOLD_SHORT,
	HOLD_1S,
	HOLD_2S,
	HOLD_3S,
	HOLD_5S,

}button_hold_event_e;

extern button_hold_event_e g_buttonEvent;

void buttonHoldScan(void);


#endif /* BUTTON_H_ */
