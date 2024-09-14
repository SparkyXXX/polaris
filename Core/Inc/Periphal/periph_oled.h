/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_oled.h
 *  Description  : This file contains 0.96 inch OLED driver function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-23 04:53:41
 */


#ifndef OLED_PERIPH_H
#define OLED_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lib_font.h"
#include "lib_gif.h"

#define OLED_I2C_ADDRESS    0x78        // the I2C address of oled

#define MAX_COLUMN      128             // the resolution of oled   128*64
#define MAX_ROW         64

#define X_WIDTH         MAX_COLUMN
#define Y_WIDTH         MAX_ROW

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12


typedef enum {
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION= 0x02,
} Oled_PenEnum;

void Oled_Writebyte(uint8_t date, uint8_t cmd);
void OLED_init(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_OperateGram(Oled_PenEnum pen);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_DrawPoint(int8_t x, int8_t y, Oled_PenEnum pen);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Oled_PenEnum pen);
void OLED_ShowChar(uint8_t row, uint8_t col, char chr);
void OLED_ShowString(uint8_t row, uint8_t col, char *chr);
void OLED_Printf(uint8_t row, uint8_t col, const char *fmt,...);
void OLED_RefreshGram(void);
void OLED_DisplayLogo(void);
void OLED_DisplayBMG(const unsigned char *buf);
void OLED_DisplayGIF(const unsigned char gif[][1024]);

#ifdef __cplusplus
}
#endif

#endif
