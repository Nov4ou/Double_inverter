/*
 * oled.c
 *
 *  Created on: Apr 5, 2024
 *      Author: nov4ou
 */

#include "oled.h"
#include "oledfont.h"
#include "spi.h"

void OLED_Gpio_Init(void) {
  EALLOW;
  GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;
  GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;
  GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0; // RST
  OLED_RST_Clr();

  GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
  GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;
  GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0; // DC
  OLED_DC_Clr();

  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0; // CS
  OLED_CS_Clr();
  EDIS;
}

void OLED_WriteByte(Uint8 dat, Uint8 cmd) {
  if (cmd)
    OLED_DC_Set();
  else
    OLED_DC_Clr();
  DELAY_US(0.5);
  OLED_CS_Clr();
  SPIB_SendReciveData(dat);
  DELAY_US(20);
  OLED_CS_Set();
  OLED_DC_Set();
  DELAY_US(0.5);
}

void OLED_Set_Pos(Uint8 x, Uint8 y) {
  OLED_WriteByte(0xb0 + y, OLED_CMD);
  OLED_WriteByte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
  OLED_WriteByte((x & 0x0f) | 0x01, OLED_CMD);
}

void OLED_Display_On(void) {
  OLED_WriteByte(0X8D, OLED_CMD); // SET DCDC??
  OLED_WriteByte(0X14, OLED_CMD); // DCDC ON
  OLED_WriteByte(0XAF, OLED_CMD); // DISPLAY ON
}

void OLED_Display_Off(void) {
  OLED_WriteByte(0X8D, OLED_CMD); // SET DCDC??
  OLED_WriteByte(0X10, OLED_CMD); // DCDC OFF
  OLED_WriteByte(0XAE, OLED_CMD); // DISPLAY OFF
}

void OLED_Clear(void) {
  Uint8 i, n;
  for (i = 0; i < 8; i++) {
    OLED_WriteByte(0xb0 + i, OLED_CMD);
    OLED_WriteByte(0x00, OLED_CMD);
    OLED_WriteByte(0x10, OLED_CMD);
    for (n = 0; n < 128; n++)
      OLED_WriteByte(0, OLED_DATA);
  }
}

void OLED_Fill(void) {
  Uint8 i, n;
  for (i = 0; i < 8; i++) {
    OLED_WriteByte(0xb0 + i, OLED_CMD);
    OLED_WriteByte(0x00, OLED_CMD);
    OLED_WriteByte(0x10, OLED_CMD);
    for (n = 0; n < 128; n++)
      OLED_WriteByte(0xff, OLED_DATA);
  }
}

void OLED_ShowChar(Uint8 x, Uint8 y, Uint8 chr, Uint8 Size) {
  Uint8 c = 0, i = 0;
  c = chr - ' '; //???????
  if (x > Max_Column - 1) {
    x = 0;
    y = y + 2;
  }
  if (Size == 16) {
    OLED_Set_Pos(x, y);
    for (i = 0; i < 8; i++)
      OLED_WriteByte(F8X16[c * 16 + i], OLED_DATA);
    OLED_Set_Pos(x, y + 1);
    for (i = 0; i < 8; i++)
      OLED_WriteByte(F8X16[c * 16 + i + 8], OLED_DATA);
  } else {
    OLED_Set_Pos(x, y + 1);
    for (i = 0; i < 6; i++)
      OLED_WriteByte(asc2_0806[c][i], OLED_DATA);
  }
}

Uint32 oled_pow(Uint8 m, Uint8 n) {
  Uint32 result = 1;
  while (n--)
    result *= m;
  return result;
}

void OLED_ShowNum(Uint8 x, Uint8 y, Uint32 num, Uint8 len, Uint8 size) {
  Uint8 t, temp;
  Uint8 enshow = 0;
  for (t = 0; t < len; t++) {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        OLED_ShowChar(x + (size / 2) * t, y, ' ', 16);
        continue;
      } else
        enshow = 1;
    }
    OLED_ShowChar(x + (size / 2) * t, y, temp + '0', 16);
  }
}

void OLED_ShowString(Uint8 x, Uint8 y, Uint8 *chr, Uint8 Size) {
  Uint8 j = 0;
  while (chr[j] != '\0') {
    OLED_ShowChar(x, y, chr[j], Size);
    x += 8;
    if (x > 120) {
      x = 0;
      y += 2;
    }
    j++;
  }
}

void OLED_Init(void) {
  OLED_Gpio_Init();

  OLED_RST_Set();
  DELAY_US(200000);
  OLED_RST_Clr();
  DELAY_US(200000);
  OLED_RST_Set();
  DELAY_US(6);

  OLED_WriteByte(0xAE, OLED_CMD); //--turn off oled panel
  OLED_WriteByte(0x00, OLED_CMD); //---set low column address
  OLED_WriteByte(0x10, OLED_CMD); //---set high column address
  OLED_WriteByte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM
                                  // Display Start Line (0x00~0x3F)
  OLED_WriteByte(0x81, OLED_CMD); //--set contrast control register
  OLED_WriteByte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
  OLED_WriteByte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0???? 0xa1??
  OLED_WriteByte(0xC8,
                 OLED_CMD); // Set COM/Row Scan Direction   0xc0???? 0xc8??
  OLED_WriteByte(0xA6, OLED_CMD); //--set normal display
  OLED_WriteByte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
  OLED_WriteByte(0x3f, OLED_CMD); //--1/64 duty
  OLED_WriteByte(
      0xD3,
      OLED_CMD); //-set display offset   Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WriteByte(0x00, OLED_CMD); //-not offset
  OLED_WriteByte(
      0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
  OLED_WriteByte(0x80,
                 OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WriteByte(0xD9, OLED_CMD); //--set pre-charge period
  OLED_WriteByte(
      0xF1, OLED_CMD); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WriteByte(0xDA, OLED_CMD); //--set com pins hardware configuration
  OLED_WriteByte(0x12, OLED_CMD);
  OLED_WriteByte(0xDB, OLED_CMD); //--set vcomh
  OLED_WriteByte(0x40, OLED_CMD); // Set VCOM Deselect Level
  OLED_WriteByte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WriteByte(0x02, OLED_CMD); //
  OLED_WriteByte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
  OLED_WriteByte(0x14, OLED_CMD); //--set(0x10) disable
  OLED_WriteByte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
  OLED_WriteByte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
  OLED_WriteByte(0xAF, OLED_CMD); //--turn on oled panel

  OLED_WriteByte(0xAF, OLED_CMD); /*display ON*/
  OLED_Clear();
  OLED_Set_Pos(0, 0);
}
