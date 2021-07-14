#ifndef __KEY_H
#define __KEY_H
#include "main.h"

#define KEY PBin(14)
void KEY_Init(void);				  //按键初始化
uint8_t click_N_Double(uint8_t time); //单击按键扫描和双击按键扫描
uint8_t click(void);				  //单击按键扫描
uint8_t Long_Press(void);			  //长按扫描
uint8_t select(int);

void user_Key_EXTI_Init(void); //外部中断初始化
#endif
