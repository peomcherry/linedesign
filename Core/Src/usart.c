/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "test_task.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
//全场定位数据  串口6
#define BUFFERSIZE 255	//可接收的最大数据量
uint8_t Rx_len_Huart6;//串口6接收长度
uint8_t Rx_len_Huart7;//串口7接收长度
uint8_t Rx_len_Huart8;//串口7接收长度
uint8_t ReceiveBuff_Huart6[BUFFERSIZE]; //串口6接收缓冲区
uint8_t ReceiveBuff_Huart7[BUFFERSIZE]; //串口7接收缓冲区
uint8_t ReceiveBuff_Huart8[BUFFERSIZE]; //串口7接收缓冲区
float pos_x=0;//坐标X--ZBx
float pos_y=0;//坐标Y--ZBy
float zangle=0;//航向角
float xangle=0;//俯仰角
float yangle=0;//横滚角
float w_z=0;//航向角速

float set_pos_x;
float set_pos_y;
float set_zangle;
int move_flag;


uint8_t Fd_data[64];
uint8_t Fd_rsimu[64];
uint8_t Fd_rsahrs[56];
int rs_imutype =0;
int rs_ahrstype =0;
extern int Time_count;

IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;

uint8_t aRx_openmv;
//串口屏 串口2
//uint8_t Rx_len_Huart2;//串口2接收长度
//uint8_t ReceiveBuff_Huart2[BUFFERSIZE]; //串口2接收缓冲区
/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart7_rx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}
/* UART8 init function */
void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PF7     ------> UART7_TX
    PF6     ------> UART7_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* UART7 DMA Init */
    /* UART7_RX Init */
    hdma_uart7_rx.Instance = DMA1_Stream3;
    hdma_uart7_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart7_rx.Init.Mode = DMA_NORMAL;
    hdma_uart7_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart7_rx);

    /* UART7 interrupt Init */
    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* UART8 clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* UART8 DMA Init */
    /* UART8_RX Init */
    hdma_uart8_rx.Instance = DMA1_Stream6;
    hdma_uart8_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_uart8_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart8_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart8_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart8_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart8_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart8_rx.Init.Mode = DMA_NORMAL;
    hdma_uart8_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_uart8_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart8_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart8_rx);

    /* UART8 interrupt Init */
    HAL_NVIC_SetPriority(UART8_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD6     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD9     ------> USART3_RX
    PD8     ------> USART3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PF7     ------> UART7_TX
    PF6     ------> UART7_RX
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_6);

    /* UART7 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART7 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART7_IRQn);
  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspDeInit 0 */

  /* USER CODE END UART8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();

    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0);

    /* UART8 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART8_IRQn);
  /* USER CODE BEGIN UART8_MspDeInit 1 */

  /* USER CODE END UART8_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD6     ------> USART2_RX
    PD5     ------> USART2_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6|GPIO_PIN_5);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD9     ------> USART3_RX
    PD8     ------> USART3_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_9|GPIO_PIN_8);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void AHRSData2PC(void)
{
	  printf("**********                **********\r\n");	
	
// 	 printf("AHRS: The RollSpeed =  %f\r\n",AHRSData_Packet.RollSpeed);
//	 printf("AHRS: The PitchSpeed =  %f\r\n",AHRSData_Packet.PitchSpeed);
//   printf("AHRS: The HeadingSpeed =  %f\r\n",AHRSData_Packet.HeadingSpeed);
//   printf("AHRS: The Roll =  %f\r\n",AHRSData_Packet.Roll);
//   printf("AHRS: The Pitch =  %f\r\n",AHRSData_Packet.Pitch);
   printf("AHRS: The Heading =  %f\r\n",(AHRSData_Packet.Heading*57.29578f));
//   printf("AHRS: The Quaternion.Qw =  %f\r\n",AHRSData_Packet.Qw);
//   printf("AHRS: The Quaternion.Qx =  %f\r\n",AHRSData_Packet.Qx);
//   printf("AHRS: The Quaternion.Qy =  %f\r\n",AHRSData_Packet.Qy);
//   printf("AHRS: The Quaternion.Qz =  %f\r\n",AHRSData_Packet.Qz);
//   printf("AHRS: The Timestamp =  %d\r\n",AHRSData_Packet.Timestamp);
//	  printf("**********                **********\r\n");	
	
}
void IMUData2PC(void)
{
   //printf("Now start sending IMU data.\r\n");
	  printf("**********                **********\r\n");	

	 printf("IMU: The gyroscope_x =  %f\r\n",IMUData_Packet.gyroscope_x);
	 printf("IMU:The gyroscope_y =  %f\r\n",IMUData_Packet.gyroscope_y);
   printf("IMU:The gyroscope_z =  %f\r\n",IMUData_Packet.gyroscope_z);
   printf("IMU:The accelerometer_x =  %f\r\n",IMUData_Packet.accelerometer_x);
   printf("IMU:The accelerometer_y =  %f\r\n",IMUData_Packet.accelerometer_y);
   printf("IMU:The accelerometer_z =  %f\r\n",IMUData_Packet.accelerometer_z);
   printf("IMU:The magnetometer_x =  %f\r\n",IMUData_Packet.magnetometer_x);
   printf("IMU:The magnetometer_y =  %f\r\n",IMUData_Packet.magnetometer_y);
   printf("IMU:The magnetometer_z =  %f\r\n",IMUData_Packet.magnetometer_z);
   printf("IMU:The Timestamp =  %d\r\n",IMUData_Packet.Timestamp);
	 //printf("Now the data of IMU has been sent.\r\n");
   printf("**********                **********\r\n");	

}

float DATA_Trans(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}
long long timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4)
{
  uint32_t transition_32;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
	return transition_32;
}

uint8_t TTL_Hex2Dec(uint8_t *Fd_data)
{
  //  uint8 i;
     if(rs_ahrstype==1)
    {
        if(Fd_data[1]==TYPE_AHRS&&Fd_data[2]==AHRS_LEN)
        {
            AHRSData_Packet.RollSpeed=DATA_Trans(Fd_data[7],Fd_data[8],Fd_data[9],Fd_data[10]);       //????????
            AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_data[11],Fd_data[12],Fd_data[13],Fd_data[14]);   //?????????
            AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_data[15],Fd_data[16],Fd_data[17],Fd_data[18]); //????????

            AHRSData_Packet.Roll=DATA_Trans(Fd_data[19],Fd_data[20],Fd_data[21],Fd_data[22]);      //?????
            AHRSData_Packet.Pitch=DATA_Trans(Fd_data[23],Fd_data[24],Fd_data[25],Fd_data[26]);     //??????
            AHRSData_Packet.Heading=DATA_Trans(Fd_data[27],Fd_data[28],Fd_data[29],Fd_data[30]);     //?????

            AHRSData_Packet.Qw=DATA_Trans(Fd_data[31],Fd_data[32],Fd_data[33],Fd_data[34]);  //?????
            AHRSData_Packet.Qx=DATA_Trans(Fd_data[35],Fd_data[36],Fd_data[37],Fd_data[38]);
            AHRSData_Packet.Qy=DATA_Trans(Fd_data[39],Fd_data[40],Fd_data[41],Fd_data[42]);
            AHRSData_Packet.Qz=DATA_Trans(Fd_data[43],Fd_data[44],Fd_data[45],Fd_data[46]);
            AHRSData_Packet.Timestamp=(unsigned long int)timestamp(Fd_data[47],Fd_data[48],Fd_data[49],Fd_data[50]);   //????
            //AHRSData2PC();
        }
        rs_ahrstype=0;
        //debug("t0\r\n",0);
       // yaw_imu=RAD_TO_ANGLE(AHRSData_Packet.Heading);//????????
    }
    if(rs_imutype==1)
    {
        if(Fd_data[1]==TYPE_IMU&&Fd_data[2]==IMU_LEN)
        {
            IMUData_Packet.gyroscope_x=DATA_Trans(Fd_data[7],Fd_data[8],Fd_data[9],Fd_data[10]);  //?????
            IMUData_Packet.gyroscope_y=DATA_Trans(Fd_data[11],Fd_data[12],Fd_data[13],Fd_data[14]);
            IMUData_Packet.gyroscope_z=DATA_Trans(Fd_data[15],Fd_data[16],Fd_data[17],Fd_data[18]);

            IMUData_Packet.accelerometer_x=DATA_Trans(Fd_data[19],Fd_data[20],Fd_data[21],Fd_data[22]);  //??????
            IMUData_Packet.accelerometer_y=DATA_Trans(Fd_data[23],Fd_data[24],Fd_data[25],Fd_data[26]);
            IMUData_Packet.accelerometer_z=DATA_Trans(Fd_data[27],Fd_data[28],Fd_data[29],Fd_data[30]);

            IMUData_Packet.magnetometer_x=DATA_Trans(Fd_data[31],Fd_data[32],Fd_data[33],Fd_data[34]);  //??????????
            IMUData_Packet.magnetometer_y=DATA_Trans(Fd_data[35],Fd_data[36],Fd_data[37],Fd_data[38]);
            IMUData_Packet.magnetometer_z=DATA_Trans(Fd_data[39],Fd_data[40],Fd_data[41],Fd_data[42]);

            IMUData_Packet.Timestamp=(unsigned long int)timestamp(Fd_data[55],Fd_data[56],Fd_data[57],Fd_data[58]);   //????
               //IMUData2PC();
        }
        rs_imutype=0;
        //debug("t1\r\n",0);
        //debug("imu_z %f\r\n",IMUData_Packet.gyroscope_z);
       // extern void give_HeadingSpeed(float value);//???????????????
        //HeadingSpeed=(RAD_TO_ANGLE(IMUData_Packet.gyroscope_z));
 }
    //???????

return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
//	if(huart == &huart8)
//		{
//			 static union
//			{
//					uint8_t date[24];
//					float ActVal[6];
//			} posture;
//		if(huart == &huart8){
//			
//		 for(int i=0; i<24; i++)
//				{
//						posture.date[i]=ReceiveBuff_Huart8[i+2]; //ReceiveBuff_Huart7
//				}
//				zangle=-posture.ActVal[0];
//				xangle=posture.ActVal[1];
//				yangle=posture.ActVal[2];
//				pos_x=posture.ActVal[3];
//				pos_y=posture.ActVal[4];
//				w_z=posture.ActVal[5];
//		}
//	}
	if(huart == &huart2)//如果是串口
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
		//aRx_openmv = aRxBuffer[0];
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	}
//	if(huart == &huart2) //树莓派--串口2
//	{
//		static uint8_t Count=0;
//		static uint8_t last_rsnum=0;
//		static uint8_t rsimu_flag=0;
//		static uint8_t rsacc_flag=0;
//			HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
//			//Usart_Receive = USART_ReceiveData(UART5);//Read the data //读取数据
//			Fd_data[Count]=aRxBuffer[0];  //串口数据填入数组
//			//usart1_send(Usart_Receive);
//			if(((last_rsnum==FRAME_END)&&(aRxBuffer[0] == FRAME_HEAD))||Count>0)
//			{
//			Count++; 
//			if((Fd_data[1]==TYPE_IMU)&&(Fd_data[2]==IMU_LEN))
//				rsimu_flag=1;
//			if((Fd_data[1]==TYPE_AHRS)&&(Fd_data[2]==AHRS_LEN))
//				rsacc_flag=1;
//			}
//			else 
//				Count=0;
//			last_rsnum=aRxBuffer[0];
//			
//		if(rsimu_flag==1 && Count==IMU_RS) //将本帧数据保存至Fd_rsimu数组中
//		{
//			Count=0;
//			rsimu_flag=0;
//			rs_imutype=1;
//			if(Fd_data[IMU_RS-1]==FRAME_END) //帧尾校验
//			{
//					TTL_Hex2Dec(Fd_data);
//			}
//		}
//		if(rsacc_flag==1 && Count==AHRS_RS) //
//		{
//			Count=0;
//			rsacc_flag=0;
//			rs_ahrstype=1;
//			if(Fd_data[AHRS_RS-1]==FRAME_END)
//			{
//					TTL_Hex2Dec(Fd_data);
//			}
//		}
//	}
	
			if(huart == &huart6){
			HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer_K210, RXBUFFERSIZE);
			if((USART_RX_STA_K210&0x8000)==0)//接收未完成
			{
				if(USART_RX_STA_K210&0x4000)//接收到了0x0d
				{
					if(aRxBuffer_K210[0]!=0x0a)USART_RX_STA_K210=0;//接收错误,重新开始
					else USART_RX_STA_K210|=0x8000;	//接收完成了 
				}
				else //还没收到0X0D
				{	
					if(aRxBuffer_K210[0]==0x0d)USART_RX_STA_K210|=0x4000;
					else
					{
						USART_RX_BUF_K210[USART_RX_STA_K210&0X3FFF]=aRxBuffer_K210[0] ;
						USART_RX_STA_K210++;
						if(USART_RX_STA_K210>(USART_REC_LEN_K210-1))USART_RX_STA_K210=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}
		}
			
		
	
}






//int UART2_IRQHandler(void)
//{
//	
//	return 0;
//}
/*******************************
16进制转浮点型数据
*******************************/


/* USER CODE END 1 */
