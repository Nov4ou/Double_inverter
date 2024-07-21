/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h" // F2806x Headerfile
#include "F2806x_EPwm.h"
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "Solar_F.h"
#include "adc.h"
#include "epwm.h"
#include "key.h"
#include "math.h"
#include "oled.h"
#include "spi.h"
#include "timer.h"

#pragma CODE_SECTION(TIM0_IRQn, "ramfuncs");
#pragma CODE_SECTION(OLED_Update, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

// #define Kp 22.7089
#define Kp 50.7089
// #define Ki 1063.14
#define Ki 5000.0
// #define ISR_FREQUENCY 20000
#define GRID_FREQ 50
#define ISR_FREQUENCY 10000
#define V_DC_REFERENCE 40
#define CURRENT_PEAK 2.2

_Bool flag_inverter = 0;
_Bool prev_flag_inverter = 1;

SPLL_1ph_SOGI_F spll1;
char str[10];
float EPWM2_DutyCycle = 0;
float normalized_voltage = 0;
float compare1, compare2;
float V_mod_inverter = 0;
float ratio = 0.7;
Uint16 counter = 0;
float ref_current = 0;
float error = 0;
float output = 0;
float Kp_set = 12;

extern float grid_inverter_voltage;
extern float grid_inverter_current;
extern float grid_voltage;

#define V_MOD_GRAPH 300
float v_mod_graph[V_MOD_GRAPH];
Uint8 v_mod_index = 0;

void OLED_Update();
void ftoa(float f, int precision);

void LED_Init(void) {
  EALLOW;
  //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

  GpioDataRegs.GPASET.bit.GPIO0 = 1;

  EDIS;
}

int main() {
  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
  InitFlash();
  InitSysCtrl();
  DINT;
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();
  EALLOW;
  PieVectTable.ADCINT1 = &adc_isr;
  EDIS;
  InitAdc(); // For this example, init the ADC
  AdcOffsetSelfCal();
  // ADC
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
  IER |= M_INT1;                     // Enable CPU Interrupt 1
  EINT;
  ERTM;

  ADC_Init();

  LED_Init();
  EPWM2_Init(MAX_CMPA);
  EPWM3_Init(MAX_CMPA);
  EPWM5_Init(MAX_CMPA);
  EPWM6_Init(MAX_CMPA);
  // EPWM7_Init(MAX_CMPA);
  // EPWM8_Init(MAX_CMPA);
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1; 
  
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  // Set actions for rectifier
  EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
  // Set actions for rectifier
  EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
  EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

  KEY_Init();
  SPIB_Init();
  OLED_Init();
  OLED_Clear();

  SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);
  spll1.osg_coeff.osg_k = 1.0;
  spll1.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  spll1.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);

  TIM0_Init(90, 101); // 10K

  while (1) {
    OLED_Update();
    if (KEY_Read() != 0) {
      if (KEY_Read() == 2) {
        flag_inverter = 1 - flag_inverter;
        while (KEY_Read() == 2)
          ;
      }
    }
  }
}

interrupt void TIM0_IRQn(void) {
  GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
  normalized_voltage = grid_voltage / 50;
  spll1.u[0] = normalized_voltage;
  SPLL_1ph_SOGI_F_FUNC(&spll1);
  SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
                               (float)(2 * PI * GRID_FREQ), &spll1);
  spll1.osg_coeff.osg_k = 1.0;

  /********************* Open Loop Test ************************/
  // V_mod_inverter = sin(spll1.theta[0]) * ratio;
  /********************* Open Loop Test ************************/

  /********************* Current Loop ************************/
  ref_current = sin(spll1.theta[0]) * CURRENT_PEAK * 1.414;
  error = ref_current - grid_inverter_current;
  output = Kp_set * error + grid_inverter_voltage;
  V_mod_inverter = output / V_DC_REFERENCE;

  v_mod_graph[v_mod_index] = V_mod_inverter;
  v_mod_index = (v_mod_index + 1) % V_MOD_GRAPH;

  /********************* Current Loop ************************/

  if (flag_inverter != prev_flag_inverter) {
    if (flag_inverter == 0) {
      EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
      // Set actions for rectifier
      EPwm5Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
      EPwm5Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
      EPwm5Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
      // Set actions for rectifier
      EPwm6Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
      EPwm6Regs.AQCTLA.bit.CAU = AQ_NO_ACTION;
      EPwm6Regs.AQCTLA.bit.CAD = AQ_NO_ACTION;
    } else if (flag_inverter == 1) {
      /********************* Inverter SPWM modulation ************************/
      EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
      EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
      // Set actions for inverter
      EPwm5Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
      EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
      EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
      // Set actions for inverter
      EPwm6Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
      EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
      EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
      /********************* Inverter SPWM modulation ************************/
    }
    prev_flag_inverter = flag_inverter;
  }
  compare1 = (Uint16)(V_mod_inverter * MAX_CMPA);
  compare2 = (Uint16)(-1 * V_mod_inverter * MAX_CMPA);
  EPwm5Regs.CMPA.half.CMPA = compare1;
  EPwm6Regs.CMPA.half.CMPA = compare2;

  EPWM2_DutyCycle = V_mod_inverter * MAX_CMPA / 2.0 + MAX_CMPA / 2.0;
  EPwm2Regs.CMPA.half.CMPA = (Uint16)EPWM2_DutyCycle;

  EALLOW;
  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}

void OLED_Update() {
  counter++;
  if (counter == 50000) {
    OLED_ShowString(0, 0, "Inverter: ", 16);
    if (flag_inverter == 1) {
      OLED_ShowString(75, 0, "    1", 16);
    } else {
      OLED_ShowString(75, 0, "    0", 16);
    }

    // OLED_ShowString(0, 2, "Rectifier: ", 16);
    // if (flag_rectifier == 1) {
    //   OLED_ShowString(83, 2, "   1", 16);
    // } else {
    //   OLED_ShowString(83, 2, "   0", 16);
    // }

    // OLED_ShowString(0, 4, "K_RLC: ", 8);
    // if (K_RLC == 1) {
    //   OLED_ShowString(75, 4, "     1", 8);
    // } else {
    //   OLED_ShowString(75, 4, "    -1", 8);
    // }

    // OLED_ShowString(0, 5, "Power factor: ", 8);
    // ftoa(power_factor, 1);
    // OLED_ShowString(95, 6, str, 4);

    // OLED_ShowString(0, 2, "Eff:", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1) {
    //   ftoa(filteredEff * 100, 1);
    //   OLED_ShowString(45, 2, str, 16);
    // }

    // OLED_ShowString(0, 4, "Loop: ", 16);
    // if (Vol1Loop == 1 && Vol2Loop == 1)
    //   OLED_ShowString(45, 4, "ON ", 16);
    // else
    //   OLED_ShowString(45, 4, "OFF", 16);

    counter = 0;
  }
}

// This function converts a float to a string with a specified precision.
void ftoa(float f, int precision) {
  int j;

  // Split the float into integer and fractional parts
  int int_part = (int)f;
  float frac_part = f - int_part;

  int i = 0;
  // If the integer part is 0, add '0' to the string
  if (int_part == 0) {
    str[i++] = '0';
  } else {
    // Convert the integer part to string
    while (int_part) {
      str[i++] = int_part % 10 + '0';
      int_part /= 10;
    }
  }

  // Reverse the string to get the correct order
  for (j = 0; j < i / 2; j++) {
    char temp = str[j];
    str[j] = str[i - j - 1];
    str[i - j - 1] = temp;
  }

  // Add the decimal point
  str[i++] = '.';

  // Convert the fractional part to string with the specified precision
  for (j = 0; j < precision; j++) {
    frac_part *= 10;
    int digit = (int)frac_part;
    str[i++] = digit + '0';
    frac_part -= digit;
  }

  // Terminate the string
  str[i] = '\0';
}
