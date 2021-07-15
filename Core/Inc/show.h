#ifndef __SHOW_H
#define __SHOW_H
#include "main.h"
#include "oled.h"

void oled_show(void);
void APP_Show(void);
void OLED_DrawPoint_Shu(uint8_t x, uint8_t y, uint8_t t);
void OLED_Show_CCD(void);
void oled_show_once(void);
void DataScope(void);
#endif
