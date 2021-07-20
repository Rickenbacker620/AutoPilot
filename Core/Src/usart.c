/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

int send_cnt = 0;
int re_Encoder_Left, re_Encoder_Right;

#pragma pack(push)
#pragma pack(1)
typedef union
{
  uint8_t raw[49];
  struct
  {
    uint16_t header;
    uint8_t len;
    int re_Encoder_Left;
    int re_Encoder_Right;
    int Voltage;
    short accelX;
    short accelY;
    short accelZ;
    short gyroX;
    short gyroY;
    short gyroZ;
    short magX;
    short magY;
    short magZ;
  } data;
} carInfo;
#pragma pack(pop)

/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  LL_USART_EnableIT_RXNE(USART1);

  /* USER CODE END USART1_Init 2 */
}

/* USER CODE BEGIN 1 */

int fputc(int ch, FILE *f)
{
  LL_USART_TransmitData8(USART1, (uint8_t)ch);
  while (!LL_USART_IsActiveFlag_TXE(USART1))
    ;
  return ch;
}

void usart1_send(uint8_t data)
{
  LL_USART_TransmitData8(USART1, data);

  while (!LL_USART_IsActiveFlag_TXE(USART1))
    ;
}

// void Send_Signal()
// {
//   static uint8_t Send_rasberry[60];
//   Send_rasberry[0] = 0xA5;
//   Send_rasberry[1] = 0x5A;
//   Send_rasberry[2] = 0x2E;
//   re_Encoder_Left = -Encoder_Left;
//   re_Encoder_Right = -Encoder_Right;
//   for (send_cnt = 0; send_cnt < 4; send_cnt++)
//   {
//     Send_rasberry[3 + send_cnt] = ((unsigned char *)&re_Encoder_Left)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 4; send_cnt++)
//   {
//     Send_rasberry[7 + send_cnt] = ((unsigned char *)&re_Encoder_Right)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 4; send_cnt++)
//   {
//     Send_rasberry[11 + send_cnt] = ((unsigned char *)&Voltage)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // X 轴加速度计值
//   {
//     Send_rasberry[15 + send_cnt] = ((unsigned char *)&accelX)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Y 轴加速度计值
//   {
//     Send_rasberry[17 + send_cnt] = ((unsigned char *)&accelY)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Y 轴加速度计值
//   {
//     Send_rasberry[19 + send_cnt] = ((unsigned char *)&accelZ)[send_cnt];
//   }
//   //send gyro X Y Z
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // X 轴角速度值
//   {
//     Send_rasberry[21 + send_cnt] = ((unsigned char *)&gyroX)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Y 轴角速度值
//   {
//     Send_rasberry[23 + send_cnt] = ((unsigned char *)&gyroY)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Z 轴角速度值
//   {
//     Send_rasberry[25 + send_cnt] = ((unsigned char *)&gyroZ)[send_cnt];
//   }
//   //send MAG X Y Z
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // X 轴磁力计值
//   {
//     Send_rasberry[27 + send_cnt] = ((unsigned char *)&magX)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Y 轴磁力计值
//   {
//     Send_rasberry[29 + send_cnt] = ((unsigned char *)&magY)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 2; send_cnt++) // Z 轴磁力计值
//   {
//     Send_rasberry[31 + send_cnt] = ((unsigned char *)&magZ)[send_cnt];
//   }
//   //send ultrasonic A B C D
//   for (send_cnt = 0; send_cnt < 4; send_cnt++) // 超测量距离值 A
//   {
//     Send_rasberry[33 + send_cnt] = ((unsigned char *)&Distance_A)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 4; send_cnt++) // 超测量距离值 B
//   {
//     Send_rasberry[37 + send_cnt] = ((unsigned char *)&Distance_B)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 4; send_cnt++) // 超测量距离值 C
//   {
//     Send_rasberry[41 + send_cnt] = ((unsigned char *)&Distance_C)[send_cnt];
//   }
//   for (send_cnt = 0; send_cnt < 4; send_cnt++) // 超测量距离值 D
//   {
//     Send_rasberry[45 + send_cnt] = ((unsigned char *)&Distance_D)[send_cnt];
//   }
//   //send Send_rasberry
//   for (send_cnt = 0; send_cnt < 49; send_cnt++)
//   {
//     usart1_send(Send_rasberry[send_cnt]);
//   }
//   memset(Send_rasberry, 0, sizeof(uint8_t) * 50); //数组清零
// }

void Send_Signal()
{
  static carInfo package;
  package.data.header = 0x5AA5;
  package.data.len = 0x1E;

  re_Encoder_Left = -Encoder_Left;
  re_Encoder_Right = -Encoder_Right;

  package.data.re_Encoder_Left = re_Encoder_Left;
  package.data.re_Encoder_Right = re_Encoder_Right;

  package.data.accelX = accelX;
  package.data.accelY = accelY;
  package.data.accelZ = accelZ;
  package.data.gyroX = gyroX;
  package.data.gyroY = gyroY;
  package.data.gyroZ = gyroZ;
  package.data.magX = magX;
  package.data.magY = magY;
  package.data.magZ = magZ;

  for (send_cnt = 0; send_cnt < 33; send_cnt++)
  {
    usart1_send(package.raw[send_cnt]);
  }
  memset(package.raw, 0, sizeof(uint8_t) * 33); //数组清零
}

int Receive_Signal(void)
{
  if (LL_USART_IsActiveFlag_RXNE(USART1))
  {
    uint8_t temp;
    static uint8_t count, last_data, last_last_data, Usart_ON_Count;
    if (Usart_ON_Flag == 0)
    {
      if (++Usart_ON_Count > 10)
        Usart_ON_Flag = 1;
    }
    temp = USART1->DR;
    if (Usart_Flag == 0)
    {
      if (last_data == 0x5a && last_last_data == 0xa5)
        Usart_Flag = 1, count = 0;
    }
    if (Usart_Flag == 1)
    {
      Urxbuf[count] = temp;
      count++;
      if (count == 8)
      {
        Usart_Flag = 0;
      }
    }
    last_last_data = last_data;
    last_data = temp;
  }
  return 0;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
