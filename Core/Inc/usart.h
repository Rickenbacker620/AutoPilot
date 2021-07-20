/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */

  // union carInfo
  // {
  //   uint8_t raw[49];
  //   struct data
  //   {
  //     uint16_t header;
  //     uint8_t len[2];
  //     int re_Encoder_Left;
  //     int re_Encoder_Right;
  //     int Voltage;
  //     int accelX;
  //     int accelY;
  //     int accelZ;
  //     int gyroX;
  //     int gyroY;
  //     int gyroZ;
  //     int magX;
  //     int magY;
  //     int magZ;
  //     int Distance_A;
  //     int Distance_B;
  //     int Distance_C;
  //     int Distance_D;
  //   };
  // } aa;

  /* USER CODE END Private defines */

  void MX_USART1_UART_Init(void);

  /* USER CODE BEGIN Prototypes */

  int Receive_Signal(void);

  void usart1_send(uint8_t data);
  void Send_Signal(void);

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
