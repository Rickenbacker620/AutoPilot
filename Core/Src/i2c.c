/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C2 GPIO Configuration
  PB10   ------> I2C2_SCL
  PB11   ------> I2C2_SDA
  */
  GPIO_InitStruct.Pin = MPU9250_SCL_Pin | MPU9250_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C2);
  LL_I2C_DisableGeneralCall(I2C2);
  LL_I2C_EnableClockStretching(I2C2);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C2, 0);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/* USER CODE BEGIN 1 */

uint8_t I2C_Write_Reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_data)
{
  uint8_t temp = 0;

  slave_addr &= ~0x01;
  //wait for bus is free
  while (LL_I2C_IsActiveFlag_BUSY(I2C2))
    ;

  //transmit slave address
  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2))
    ;

  LL_I2C_TransmitData8(I2C2, slave_addr);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C2))
    ;
  LL_I2C_ClearFlag_ADDR(I2C2);

  //transmit reg to read address
  LL_I2C_TransmitData8(I2C2, reg_addr);
  while (!LL_I2C_IsActiveFlag_TXE(I2C2))
    ;

  //start writing
  LL_I2C_TransmitData8(I2C2, reg_data);
  while (!LL_I2C_IsActiveFlag_BTF(I2C2))
    ;

  LL_I2C_GenerateStopCondition(I2C2);
}

uint8_t I2C_Read_Reg(uint8_t slave_addr, uint8_t reg_addr)
{
  uint8_t temp = 0;

  // slave_addr <<= 1;

  slave_addr &= ~0x01;
  //wait for bus is free
  while (LL_I2C_IsActiveFlag_BUSY(I2C2))
    ;

  //transmit slave address
  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2))
    ;

  LL_I2C_TransmitData8(I2C2, slave_addr);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C2))
    ;
  LL_I2C_ClearFlag_ADDR(I2C2);

  //transmit reg to read address
  LL_I2C_TransmitData8(I2C2, reg_addr);
  while (!LL_I2C_IsActiveFlag_TXE(I2C2))
    ;

  //start reading
  LL_I2C_GenerateStartCondition(I2C2);
  while (!LL_I2C_IsActiveFlag_SB(I2C2))
    ;

  slave_addr |= 0x01;

  //tranmit slave address
  LL_I2C_TransmitData8(I2C2, slave_addr);

  while (!LL_I2C_IsActiveFlag_ADDR(I2C2))
    ;
  LL_I2C_ClearFlag_ADDR(I2C2);

  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);
  LL_I2C_GenerateStopCondition(I2C2);

  while (!LL_I2C_IsActiveFlag_RXNE(I2C2))
    ;

  temp = LL_I2C_ReceiveData8(I2C2);

  return temp;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
