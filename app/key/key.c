/*
 * key.c
 *
 *  Created on: Mar 23, 2024
 *      Author: nov4ou
 */

#include "key.h"

void KEY_Init()
{
    EALLOW;

    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;

    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;

    EDIS;
}

Uint8 KEY_Read()
{
    if (KEY1 == 0)
        return 1;
    if (KEY2 == 0)
        return 2;
    if (KEY3 == 0)
        return 3;
    if (KEY4 == 0)
        return 4;
    if (KEY5 == 0)
        return 5;
    if (KEY6 == 0)
        return 6;
    if (KEY7 == 0)
        return 7;

    return 0;
}

