/*
 * key.h
 *
 *  Created on: Mar 23, 2024
 *      Author: nov4ou
 */

#ifndef APP_KEY_H_
#define APP_KEY_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile

#define KEY1_SETHIGH         (GpioDataRegs.GPACLEAR.bit.GPIO26 = 1)
#define KEY1_SETLOW          (GpioDataRegs.CPASET.bit.GPIO26 = 1)

#define KEY2_SETHIGH         (GpioDataRegs.GPBCLEAR.bit.GPIO20 = 1)
#define KEY2_SETLOW          (GpioDataRegs.CPBSET.bit.GPIO20 = 1)

#define KEY3_SETHIGH         (GpioDataRegs.GPACLEAR.bit.GPIO42 = 1)
#define KEY3_SETLOW          (GpioDataRegs.CPASET.bit.GPIO42 = 1)

#define KEY4_SETHIGH         (GpioDataRegs.GPBCLEAR.bit.GPIO21 = 1)
#define KEY4_SETLOW          (GpioDataRegs.CPBSET.bit.GPIO21 = 1)

#define KEY5_SETHIGH         (GpioDataRegs.GPBCLEAR.bit.GPIO23 = 1)
#define KEY5_SETLOW          (GpioDataRegs.CPBSET.bit.GPIO23 = 1)

#define KEY6_SETHIGH         (GpioDataRegs.GPACLEAR.bit.GPIO22 = 1)
#define KEY6_SETLOW          (GpioDataRegs.CPASET.bit.GPIO22 = 1)

#define KEY7_SETHIGH         (GpioDataRegs.GPACLEAR.bit.GPIO17 = 1)
#define KEY7_SETLOW          (GpioDataRegs.CPASET.bit.GPIO17 = 1)

#define KEY1                 (GpioDataRegs.GPADAT.bit.GPIO26)
#define KEY2                 (GpioDataRegs.GPADAT.bit.GPIO20)
#define KEY3                 (GpioDataRegs.GPBDAT.bit.GPIO42)
#define KEY4                 (GpioDataRegs.GPADAT.bit.GPIO21)
#define KEY5                 (GpioDataRegs.GPADAT.bit.GPIO23)
#define KEY6                 (GpioDataRegs.GPADAT.bit.GPIO22)
#define KEY7                 (GpioDataRegs.GPADAT.bit.GPIO17)

void KEY_Init();
Uint8 KEY_Read();

#endif /* APP_KEY_H_ */
