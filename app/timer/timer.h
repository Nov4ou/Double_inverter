/*
 * timer.h
 *
 *  Created on: Mar 24, 2024
 *      Author: nov4ou
 */

#ifndef APP_TIMER_TIMER_H_
#define APP_TIMER_TIMER_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile

void TIM0_Init(float Freq, float Period);
interrupt void TIM0_IRQn();


#endif /* APP_TIMER_TIMER_H_ */
