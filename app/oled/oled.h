/*
 * oled.h
 *
 *  Created on: Apr 5, 2024
 *      Author: nov4ou
 */

#ifndef APP_OLED_OLED_H_
#define APP_OLED_OLED_H_

#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File

#define SIZE 8
#define XLevelL     					0x00
#define XLevelH     					0x10
#define Max_Column  					128
#define Max_Row     					64
#define Brightness  					0xFF
#define X_WIDTH     					128
#define Y_WIDTH     					64

#define OLED_RST_Clr()  (GpioDataRegs.GPBCLEAR.bit.GPIO57=1)
#define OLED_RST_Set()  (GpioDataRegs.GPBSET.bit.GPIO57=1)          //RES

#define OLED_DC_Clr()   (GpioDataRegs.GPBCLEAR.bit.GPIO58=1)
#define OLED_DC_Set()   (GpioDataRegs.GPBSET.bit.GPIO58=1)          //DC

#define OLED_CS_Clr()   (GpioDataRegs.GPACLEAR.bit.GPIO15=1)
#define OLED_CS_Set()   (GpioDataRegs.GPASET.bit.GPIO15=1)          //CS

#define OLED_CMD  0
#define OLED_DATA 1

void OLED_Gpio_Init(void);
void OLED_WriteByte(Uint8 dat, Uint8 cmd);
void OLED_Set_Pos(Uint8 x, Uint8 y);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_Fill(void);
void OLED_ShowChar(Uint8 x, Uint8 y, Uint8 chr, Uint8 Size);
Uint32 oled_pow(Uint8 m, Uint8 n);
void OLED_ShowNum(Uint8 x, Uint8 y, Uint32 num, Uint8 len, Uint8 size);
void OLED_ShowString(Uint8 x, Uint8 y, Uint8 *chr, Uint8 Size);
void OLED_Init(void);

#endif /* APP_OLED_OLED_H_ */
