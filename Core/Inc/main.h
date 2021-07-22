/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "control.h"
#include "delay.h"
#include "oled.h"
#include "pstwo.h"
#include "show.h"
#include "mpu9250.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define JTAG_SWD_DISABLE 0X02
#define SWD_ENABLE 0X01
#define JTAG_SWD_ENABLE 0X00
#define SERVO_INIT 1600

#define PWMA1 TIM8->CCR2
#define PWMA2 TIM8->CCR1

#define SERVO TIM1->CCR1 //????

#define PWMB1 TIM8->CCR4
#define PWMB2 TIM8->CCR3

  extern uint8_t Flag_Left, Flag_Right, Flag_Direction, Flag_Way, Flag_Next, operationMode;
  extern uint8_t Flag_Stop, Flag_Show;
  extern int Encoder_Left, Encoder_Right;
  extern long int Motor_Left, Motor_Right;
  extern long int Target_Left, Target_Right;
  extern int Voltage;
  extern uint8_t delay_50, delay_flag;
  extern uint8_t Run_Flag;
  extern float Velocity, Angle, Servo;
  extern uint8_t rxbuf[8], Urxbuf[8], CAN_ON_Flag, Usart_ON_Flag, Usart_Flag, PID_Send;
  extern uint8_t txbuf[8], txbuf2[8];
  extern float Pitch, Roll, Yaw, Gryo_Z;
  extern float Position_KP, Position_KI, Position_KD;
  extern float Velocity_KP, Velocity_KI;
  extern int RC_Velocity;
  extern int PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY, Accel_Key;
  extern uint16_t CCD_Zhongzhi, CCD_Yuzhi, ADV[128];
  extern int Sensor_Left, Sensor_Middle, Sensor_Right, Sensor;
  extern int Remoter_Ch1, Remoter_Ch2, Remoter_Ch3, Remoter_Ch4;
  extern int Distance_A, Distance_B, Distance_C, Distance_D;
  extern short gyroX, gyroY, gyroZ;
  extern short accelX, accelY, accelZ;
  extern short magX, magY, magZ;
  extern float Tand;

  extern uint8_t testint;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SERVO_PERIOD 10000-1
#define SERVO_PRESCALER 72-1
#define MOTOR_PERIOD 7200-1
#define MOTOR_PRESCALER 1-1
#define ENCODER_PERIOD 65536-1
#define ENCODER_PRESCALER 1-1
#define TIM6_PERIOD 100-1
#define TIM6_PRESCALER 7200-1
#define OLED_SCL_Pin LL_GPIO_PIN_13
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin LL_GPIO_PIN_14
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_RES_Pin LL_GPIO_PIN_15
#define OLED_RES_GPIO_Port GPIOC
#define OLED_DC_Pin LL_GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOC
#define PS2_DO_Pin LL_GPIO_PIN_1
#define PS2_DO_GPIO_Port GPIOC
#define PS2_DI_Pin LL_GPIO_PIN_2
#define PS2_DI_GPIO_Port GPIOC
#define PS2_CS_Pin LL_GPIO_PIN_3
#define PS2_CS_GPIO_Port GPIOC
#define PS2_CLK_Pin LL_GPIO_PIN_4
#define PS2_CLK_GPIO_Port GPIOA
#define RIGHT_MOTOR_ENA_Pin LL_GPIO_PIN_6
#define RIGHT_MOTOR_ENA_GPIO_Port GPIOA
#define RIGHT_MOTOR_ENB_Pin LL_GPIO_PIN_7
#define RIGHT_MOTOR_ENB_GPIO_Port GPIOA
#define MPU9250_SCL_Pin LL_GPIO_PIN_10
#define MPU9250_SCL_GPIO_Port GPIOB
#define MPU9250_SDA_Pin LL_GPIO_PIN_11
#define MPU9250_SDA_GPIO_Port GPIOB
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define KEY_Pin LL_GPIO_PIN_14
#define KEY_GPIO_Port GPIOB
#define LEFT_MOTOR_IN1_Pin LL_GPIO_PIN_6
#define LEFT_MOTOR_IN1_GPIO_Port GPIOC
#define LEFT_MOTOR_IN2_Pin LL_GPIO_PIN_7
#define LEFT_MOTOR_IN2_GPIO_Port GPIOC
#define RIGHT_MOTOR_IN1_Pin LL_GPIO_PIN_8
#define RIGHT_MOTOR_IN1_GPIO_Port GPIOC
#define RIGHT_MOTOR_IN2_Pin LL_GPIO_PIN_9
#define RIGHT_MOTOR_IN2_GPIO_Port GPIOC
#define SERVO_Pin LL_GPIO_PIN_8
#define SERVO_GPIO_Port GPIOA
#define LEFT_MOTOR_ENA_Pin LL_GPIO_PIN_15
#define LEFT_MOTOR_ENA_GPIO_Port GPIOA
#define LEFT_MOTOR_ENB_Pin LL_GPIO_PIN_3
#define LEFT_MOTOR_ENB_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr (GPIOA_BASE + 12) // 0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) // 0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) // 0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) // 0x4001140C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) // 0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) // 0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) // 0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) // 0x40011408

#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
