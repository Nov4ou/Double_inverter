#include "adc.h"
#include "F2806x_Device.h"
#include "F2806x_EPwm_defines.h"
#include "app/epwm/epwm.h"

#define GRID_V_INDEX 100
#define RECTIFIER_CURR_INDEX 150

#define GRID_CURRENT_GRAPH 500

Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];
Uint16 Voltage3[10];
Uint16 Voltage4[10];

Uint8 index = 0;
Uint16 grid_curr_index;


float Vol1 = 0;

float grid_voltage;
float grid_current;
float grid_inverter_current;
float grid_inverter_voltage;

#define VOL_GRAPH_INDEX 300
float voltage_graph[VOL_GRAPH_INDEX];
Uint8 vol_index = 0;

#define CURR_GRAPH_INDEX 300
float current_graph[CURR_GRAPH_INDEX];
Uint8 curr_index = 0;

float filtered_current;

typedef struct {
  float x_est; // Estimated state value
  float P_est; // Estimated state covariance
  float Q;     // Process noise covariance
  float R;     // Measurement noise covariance
} KalmanFilter;
KalmanFilter filtered_vol3;

void kalmanFilter_Init(KalmanFilter *kf);
float kalman_filter(KalmanFilter *kf, float z);

void ADC_Init() {

  // kalmanFilter_Init(&filtered_vol3);
  //
  // Configure ADC
  //
  EALLOW;
  AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

  //
  // ADCINT1 trips after AdcResults latch
  //
  AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  AdcRegs.INTSEL1N2.bit.INT1E = 1;    // Enabled ADCINT1
  AdcRegs.INTSEL1N2.bit.INT1CONT = 0; // Disable ADCINT1 Continuous mode

  //
  // setup EOC1 to trigger ADCINT1 to fire
  //
  AdcRegs.INTSEL1N2.bit.INT1SEL = 1;

  AdcRegs.ADCSOC0CTL.bit.CHSEL =
      5; // set SOC0 channel select to ADCINA5     Grid Inverter Voltage (sin)
  AdcRegs.ADCSOC1CTL.bit.CHSEL =
      3; // set SOC1 channel select to ADCINA3     Grid Voltage (sin reference)
  AdcRegs.ADCSOC2CTL.bit.CHSEL =
      0xA; // set SOC1 channel select to ADCINB2  
  AdcRegs.ADCSOC3CTL.bit.CHSEL =
      0xC; // set SOC1 channel select to ADCINB4   Grid Inverter Output Current
  //
  // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  //
  // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

  //
  // set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

  //
  // set SOC3 start trigger on EPWM1A, due to round-robin SOC0 converts
  // first then SOC1
  //
  AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

  //
  // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
  //
  AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;

  //
  // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
  //
  AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;
  EDIS;

  //
  // Assumes ePWM1 clock is already enabled in InitSysCtrl();
  //
  // Setup Sync
  EPwm1Regs.TBCTL.bit.SYNCOSEL =
      TB_CTR_ZERO; // Send sync signal when counter is zero
  // Allow each timer to be sync'ed
  EPwm1Regs.TBCTL.bit.PHSEN =
      TB_DISABLE; // Disable phase synchronization (Master)
  EPwm1Regs.TBPHS.half.TBPHS = 0;
  EPwm1Regs.TBCTR = 0x0000; // Clear counter

  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;          // Enable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL = 4;         // Select SOC from CMPA on upcount
  EPwm1Regs.ETPS.bit.SOCAPRD = 1;          // Generate pulse on 1st event
  EPwm1Regs.CMPA.half.CMPA = MAX_CMPA / 2; // Set compare A value
  EPwm1Regs.TBPRD = MAX_CMPA;              // Set period for ePWM1
  // EPwm1Regs.CMPA.half.CMPA = 2250; // Set compare A value
  // EPwm1Regs.TBPRD = 4500;   // Set period for ePWM1
  EPwm1Regs.TBCTL.bit.CTRMODE = 0; // count up and start
}

//
// adc_isr -
//
__interrupt void adc_isr(void) {
  Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
  Voltage2[ConversionCount] = AdcResult.ADCRESULT1;
  Voltage3[ConversionCount] = AdcResult.ADCRESULT2;
  Voltage4[ConversionCount] = AdcResult.ADCRESULT3;

  // Vol1 = Voltage1[ConversionCount] * 3.3 / 4095;

  // filtered_current = kalman_filter(&filtered_vol3, Voltage4[ConversionCount]);

  // 20m ohm
  grid_inverter_current = (Voltage4[ConversionCount] * 0.0016 - 2.9939) * 2;
  current_graph[curr_index] = grid_inverter_current;
  curr_index = (curr_index + 1) % CURR_GRAPH_INDEX;

  grid_inverter_voltage = 0.0333 * Voltage1[ConversionCount] - 61.893;
  voltage_graph[vol_index] = grid_inverter_voltage;
  vol_index = (vol_index + 1) % VOL_GRAPH_INDEX;

  // grid_inverter_voltage = (Vol1 - 1.502) * 41.61;
  // rectifier_volt_graph[rectifier_volt_index++] = rectifier_voltage;
  // if (rectifier_volt_index >= GRID_V_INDEX)
  //   rectifier_volt_index = 0;

  grid_voltage = Voltage2[ConversionCount] * 0.0272 - 50.498;
  // grid_vol_graph[gridvindex++] = grid_voltage;
  // if (gridvindex > GRID_V_INDEX)
  //   gridvindex = 0;

  // 10m ohm
  // grid_current = (Voltage3[ConversionCount] * 0.0016 - 2.9816) * 2;
  // grid_current_graph[grid_curr_index++] = grid_current;
  // if (grid_curr_index > GRID_CURRENT_GRAPH)
  //   grid_curr_index = 0;
  //
  // If 20 conversions have been logged, start over
  //
  if (ConversionCount == 9) {
    ConversionCount = 0;
  } else {
    ConversionCount++;
  }

  //
  // Clear ADCINT1 flag reinitialize for next SOC
  //
  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE

  LoopCount++;
  return;
}

void kalmanFilter_Init(KalmanFilter *kf) {
  // Initialize Kalman Filter
  kf->x_est = 0;  // Initial estimate of state value
  kf->P_est = 1;  // Initial estimate of state covariance
  kf->Q = 0.0001; // Process noise covariance
  kf->R = 0.01;   // Measurement noise covariance
}

float kalman_filter(KalmanFilter *kf, float z) {
  // Prediction step
  float x_pred = kf->x_est;
  float P_pred = kf->P_est + kf->Q;

  // Update step
  float K = P_pred / (P_pred + kf->R);
  kf->x_est = x_pred + K * (z - x_pred);
  kf->P_est = (1 - K) * P_pred;

  return kf->x_est;
}
