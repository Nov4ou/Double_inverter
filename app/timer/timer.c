/*
 * timer.c
 *
 *  Created on: Mar 24, 2024
 *      Author: nov4ou
 */

#include "timer.h"


void TIM0_Init(float Freq, float Period)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
    EDIS;

    EALLOW;
    PieVectTable.TINT0 = &TIM0_IRQn;
    EDIS;

    CpuTimer0.RegsAddr = &CpuTimer0Regs;                    // Initialize address pointers to respective timer registers
    CpuTimer0Regs.PRD.all = 0xFFFFFFFF;                     // Initialize timer period to maximum
    CpuTimer0Regs.TPR.all = 0;                              // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CpuTimer0Regs.TPRH.all = 0;
    CpuTimer0Regs.TCR.bit.TSS = 1;                          // Make sure timer is stopped
    CpuTimer0Regs.TCR.bit.TRB = 1;                          // Reload all counter register with period value
    CpuTimer0.InterruptCount = 0;                           // Reset interrupt counters

    ConfigCpuTimer(&CpuTimer0, Freq, Period);

    CpuTimer0Regs.TCR.bit.TSS = 0;                          // Enable timer0
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    EINT;
    ERTM;
}

// interrupt void TIM0_IRQn(void)
// {
//    EALLOW;
//    GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
//    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
//    EDIS;
// }


