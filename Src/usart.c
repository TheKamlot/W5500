/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "ring_buffer.h"

// UART transmit buffer descriptor
static RingBuffer USART_RingBuffer_Tx;
// UART transmit buffer memory pool
static char RingBufferData_Tx[1024];

// UART receive buffer descriptor
static RingBuffer USART_RingBuffer_Rx;
// UART receive buffer memory pool
static char RingBufferData_Rx[1024];
/* USER CODE END 0 */

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  RingBuffer_Init(&USART_RingBuffer_Tx, RingBufferData_Tx, sizeof(RingBufferData_Tx));
  RingBuffer_Init(&USART_RingBuffer_Rx, RingBufferData_Rx, sizeof(RingBufferData_Rx));
  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
  PD8   ------> USART3_TX
  PD9   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 1 */



bool USART_PutChar(char c){
  __disable_irq();
  if(RingBuffer_PutChar(&USART_RingBuffer_Tx, c)){
    LL_USART_EnableIT_TXE(USART3);
    __enable_irq();
    return true;
  }
  __enable_irq();
  return false;
}


size_t USART_WriteData(const void *data, size_t dataSize){
  if(dataSize == 0 ) return 0;

  const char* charData = (const char*)data;

  size_t count;

  for(count = 0; count < dataSize; count++){
    if(!USART_PutChar(charData[count])){
      return count;
    }
  }
  return count;
}


size_t USART_WriteString(const char *string){
  int i = 0;
  while(string[i]){
    if(!USART_PutChar(string[i])){
      break;
    };
    i++;
  }
  return i;

}

bool USART_GetChar(const char *c){
  __disable_irq();
  bool result = RingBuffer_GetChar(&USART_RingBuffer_Rx, c);
  __enable_irq();
  return result;
}


size_t USART_ReadData(void *data,const size_t maxSize){
  size_t pnt = 0;
  const char* charData = (const char*)data;
  while(USART_GetChar(charData + pnt) && (pnt + 1 <= maxSize)){
    pnt++;
  }
  return pnt;
}

void USART3_IRQHandler(void){
  if (LL_USART_IsActiveFlag_TXE(USART3)) {
    char c;
    if(RingBuffer_GetChar(&USART_RingBuffer_Tx, &c)){
      LL_USART_TransmitData8(USART3, c);
    }else{
      LL_USART_DisableIT_TXE(USART3);
    }

  }
  if (LL_USART_IsActiveFlag_RXNE(USART3)) {
    // the RXNE interrupt has occurred
    RingBuffer_PutChar(&USART_RingBuffer_Rx, USART3->DR);
  }
}

/* USER CODE END 1 */
