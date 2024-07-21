/*
 * spi.c
 *
 *  Created on: Apr 5, 2024
 *      Author: nov4ou
 */

#include "spi.h"

void SPIB_Init(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;   // SPI-A
    EDIS;

    InitSpibGpio();

    SpibRegs.SPICCR.all =0x0007;                 // Reset on, rising edge, 8-bit char bits, disable loopback mode

    SpibRegs.SPICTL.all =0x000E;                 // Enable master mode, SPICLK signal delayed by one half-cycle.
                                                 // Polarity determined by the CLOCK POLARITY bit.
                                                 // enable talk, and SPI int disabled.

    SpibRegs.SPIBRR = 3;                         // SPI Baud Rate = LSPCLK/4. (LSPCLK = 22.5MHz)

    SpibRegs.SPIPRI.bit.TRIWIRE = 0;             // Normal 4-wire SPI mode.
    SpibRegs.SPIFFTX.bit.TXFFIENA = 0;           // Disable TX FIFO SPIFFTX

    SpibRegs.SPICCR.bit.SPISWRESET = 1;          // Wake up SPI
    SpibRegs.SPIPRI.bit.FREE = 0;                // Free run
}

void SPIB_SendReciveData(Uint16 dat)
{
//    // Transmit data
//    SpiaRegs.SPITXBUF=dat;
//
//    // Wait until data is received
//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
//
//    return SpiaRegs.SPIRXBUF;
    SpibRegs.SPITXBUF = dat << 8;
}

void SPIB_SendData(Uint16 data)
{
    SpiaRegs.SPITXBUF = data;                     // Transmit data
//    while (SpiaRegs.SPIFFRX.bit.RXFFST != 1);

//    return SpiaRegs.SPITXBUF;
}
