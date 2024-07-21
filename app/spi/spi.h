/*
 * spi.h
 *
 *  Created on: Apr 5, 2024
 *      Author: nov4ou
 */

#ifndef APP_SPI_H_
#define APP_SPI_H_

#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File

void SPIB_Init(void);
void SPIB_SendReciveData(Uint16 dat);
void SPIB_SendData(Uint16 data);

#endif /* APP_SPI_H_ */
