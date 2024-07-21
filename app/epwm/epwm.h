/*
 * epwm.h
 *
 *  Created on: Mar 25, 2024
 *      Author: nov4ou
 */

#ifndef APP_EPWM_EPWM_H_
#define APP_EPWM_EPWM_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile

#define PI 3.14159265358979

// #define PWM_TBPRD 4500 // 10kHz PWM frequency
// #define PWM_FREQ 10000 // 10kHz PWM frequency
// #define SINE_FREQ 49.51   // 50Hz sine wave frequency
// // #define SINE_FREQ 34
// #define MAX_CMPA 4500  // Maximum value for CMPA, should be same as TBPRD

#define PWM_TBPRD 1500 // 30kHz PWM frequency
#define PWM_FREQ 30000 // 30kHz PWM frequency
#define SINE_FREQ 49.51   // 50Hz sine wave frequency
// #define SINE_FREQ 34
#define MAX_CMPA 1500  // Maximum value for CMPA, should be same as TBPRD


void EPWM1_Init(Uint16 tbprd);
void EPWM2_Init(Uint16 tbprd);
void EPWM3_Init(Uint16 tbprd);
void EPWM4_Init(Uint16 tbprd);
void EPWM5_Init(Uint16 tbprd);
void EPWM6_Init(Uint16 tbprd);
void EPWM7_Init(Uint16 tbprd);
void EPWM8_Init(Uint16 tbprd);
__interrupt void epwm5_timer_isr(void);
__interrupt void epwm6_timer_isr(void);
__interrupt void epwm7_timer_isr(void);
__interrupt void epwm8_timer_isr(void);

#endif /* APP_EPWM_EPWM_H_ */
