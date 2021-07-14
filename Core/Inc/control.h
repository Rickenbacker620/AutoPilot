#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "main.h"

#define PI 3.14159265
#define ZHONGZHI 0
#define DIFFERENCE 100
extern int Balance_Pwm, Velocity_Pwm, Turn_Pwm;
void Kinematic_Analysis(float velocity, float angle);
int EXTI15_10_IRQHandler(void);
void Set_Pwm(int motor_a, int motor_b, int servo);
void Key(void);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A, int amplitude_B);
uint8_t Turn_Off(int voltage);
int myabs(int a);
int Incremental_PI_Left(int Encoder, int Target);
int Incremental_PI_Right(int Encoder, int Target);
void Get_RC(void);
void Find_CCD_Zhongzhi(void);
void Core_Control(void);
#endif
