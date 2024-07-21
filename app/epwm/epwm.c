/*
 * epwm.c
 *
 *  Created on: Mar 25, 2024
 *      Author: nov4ou
 */
#include "epwm.h"
#include "F2806x_Device.h"
#include "F2806x_EPwm_defines.h"

float rectifier_dutycycle = 0;
extern float ref_sinwave;
Uint16 sineValue1 = 0, sineValue2 = 0;
extern _Bool flag;
float i_ref_rt;

void EPWM1_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1; // ePWM1
  EDIS;

  InitEPwm1Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Send sync signal when counter is zero
  // Allow each timer to be sync'ed
  EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase synchronization (Master)
  EPwm1Regs.TBPHS.half.TBPHS = 0;
  EPwm1Regs.TBCTR = 0x0000; // Clear counter
  EPwm1Regs.TBPRD = tbprd;
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm1Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm1Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM1A on Zero
  EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
  EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm1Regs.DBRED = 10;
  EPwm1Regs.DBFED = 10;

  EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm1Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM2_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1; // ePWM2
  EDIS;

  InitEPwm2Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // Use sync input
  // Allow each timer to be sync'ed
  EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;   // Enable phase synchronization
  EPwm2Regs.TBPHS.half.TBPHS = 0;
  EPwm2Regs.TBCTR = 0x0000; // Clear counter
  EPwm2Regs.TBPRD = tbprd;
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm2Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm2Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM2A on Zero
  EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, up count
  EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM2B on Zero
  EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM2B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm2Regs.DBRED = 10;
  EPwm2Regs.DBFED = 10;

  EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm2Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM3_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1; // ePWM3
  EDIS;

  InitEPwm3Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // // Use sync input
  // Allow each timer to be sync'ed
  EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm3Regs.TBPHS.half.TBPHS = 0;
  EPwm3Regs.TBCTR = 0x0000; // Clear counter
  EPwm3Regs.TBPRD = tbprd;
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm3Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm3Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM3A on Zero
  EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM3A on event A, up count
  EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM3B on Zero
  EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM3B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm3Regs.DBRED = 10;
  EPwm3Regs.DBFED = 10;

  EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm3Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM4_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 1; // ePWM4
  EDIS;

  InitEPwm4Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // // Use sync input
  // Allow each timer to be sync'ed
  EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm4Regs.TBPHS.half.TBPHS = 0;
  EPwm4Regs.TBCTR = 0x0000; // Clear counter
  EPwm4Regs.TBPRD = tbprd;
  EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm4Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm4Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM4A on Zero
  EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM4A on event A, up count
  EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM4B on Zero
  EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM4B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm4Regs.DBRED = 10;
  EPwm4Regs.DBFED = 10;

  EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm4Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm4Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM5_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5
  EDIS;

  InitEPwm5Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // // Use sync input
  // Allow each timer to be sync'ed
  EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm5Regs.TBPHS.half.TBPHS = 0;
  EPwm5Regs.TBCTR = 0x0000; // Clear counter
  EPwm5Regs.TBPRD = tbprd;
  EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
  EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm5Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm5Regs.CMPB = tbprd / 2;           // Set Compare B value

  // // Set actions
  // EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM5A on Zero
  // EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM5A on event A, up count
  // EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM5B on Zero
  // EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM5B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm5Regs.DBRED = 10;
  EPwm5Regs.DBFED = 10;

  EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm5Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM6_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6
  EDIS;

  InitEPwm6Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // // Use sync input
  // Allow each timer to be sync'ed
  EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm6Regs.TBPHS.half.TBPHS = 0;
  EPwm6Regs.TBCTR = 0x0000; // Clear counter
  EPwm6Regs.TBPRD = tbprd;
  EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
  EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm6Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm6Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM6A on Zero
  EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM6A on event A, up count
  EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM6B on Zero
  EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM6B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm6Regs.DBRED = 10;
  EPwm6Regs.DBFED = 10;

  EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm6Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM7_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  EDIS;

  InitEPwm7Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // // Use sync input
  // Allow each timer to be sync'ed
  EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm7Regs.TBPHS.half.TBPHS = 0;
  EPwm7Regs.TBCTR = 0x0000; // Clear counter
  EPwm7Regs.TBPRD = tbprd;
  EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
  EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm7Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm7Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM7A on Zero
  EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM7A on event A, up count
  EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM7B on Zero
  EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM7B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm7Regs.DBRED = 10;
  EPwm7Regs.DBFED = 10;

  EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm7Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void EPWM8_Init(Uint16 tbprd) {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  EDIS;

  InitEPwm8Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // Use sync input
  // Allow each timer to be sync'ed
  EPwm8Regs.TBCTL.bit.PHSEN = TB_ENABLE;
  EPwm8Regs.TBPHS.half.TBPHS = 0;
  EPwm8Regs.TBCTR = 0x0000; // Clear counter
  EPwm8Regs.TBPRD = tbprd;
  EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
  EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm8Regs.CMPA.half.CMPA = tbprd / 2; // Set compare A value
  EPwm8Regs.CMPB = tbprd / 2;           // Set Compare B value

  // Set actions
  EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;   // Set PWM8A on Zero
  EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM8A on event A, up count
  EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM8B on Zero
  EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM8B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm8Regs.DBRED = 10;
  EPwm8Regs.DBFED = 10;

  EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm8Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm8Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}
__interrupt void epwm7_timer_isr(void) {

  // static Uint16 index = 0;
  // static const float step = 2 * PI * SINE_FREQ / PWM_FREQ;
  // i_ref_rt = (1 + sin(step * index)) * 10;
  // index++;
  // if (index >= (PWM_FREQ / SINE_FREQ)) {
  //   index = 0;
  // }

  // if (flag == 1) {
  // EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  //                                            // Set actions

  // // EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  // // EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // if (ref_sinwave > 0) {
  //   EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET; // Set PWM1A on Zero
  //   EPwm8Regs.AQCTLA.bit.CAU = AQ_SET; // Clear PWM1A on event A, up count
  //   sineValue1 = (Uint16)(ref_sinwave * MAX_CMPA);
  //   EPwm7Regs.CMPA.half.CMPA = sineValue1;
  //   EPwm8Regs.CMPA.half.CMPA = MAX_CMPA;
  // }
  // if (ref_sinwave <= 0) {
  //   EPwm8Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // Set PWM1A on Zero
  //   EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM1A on event A, up count
  //   sineValue2 = (Uint16)(MAX_CMPA - (-1 * ref_sinwave * MAX_CMPA));
  //   EPwm7Regs.CMPA.half.CMPA = sineValue2;
  //   EPwm8Regs.CMPA.half.CMPA = 0;
  // }
  // }

  //
  // Clear INT flag for this timer
  //
  EPwm7Regs.ETCLR.bit.INT = 1;

  //
  // Acknowledge this interrupt to receive more interrupts from group 3
  //
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// __interrupt void epwm8_timer_isr(void) {

//   // Set Compare values
//   EPwm8Regs.CMPA.half.CMPA = rectifier_dutycycle; // Set compare A value
//   EPwm8Regs.CMPB = rectifier_dutycycle;           // Set Compare B value

//   //
//   // Clear INT flag for this timer
//   //
//   EPwm8Regs.ETCLR.bit.INT = 1;

//   //
//   // Acknowledge this interrupt to receive more interrupts from group 3
//   //
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }
