/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
 #include "bsp_sbus.h"

 //全场定位数据  串口6
#define BUFFERSIZE 255	//可接收的最大数据量
extern uint8_t Rx_len_Huart7;//串口6接收长度
extern uint8_t ReceiveBuff_Huart7[BUFFERSIZE]; //串口6接收缓冲区
extern uint8_t Rx_len_Huart8;//串口6接收长度
extern uint8_t ReceiveBuff_Huart8[BUFFERSIZE]; //串口6接收缓冲区
extern int testcount;
extern float pos_x;//坐标X--ZBx
extern float pos_y;//坐标Y--ZBy
extern float zangle;//航向角
extern float xangle;//俯仰角
extern float yangle;//横滚角
extern float w_z;//航向角速

extern float set_pos_x;
extern float set_pos_y;
extern float set_zangle;
extern int move_flag;

//串口屏 串口2
extern uint8_t Rx_len_Huart2;//串口2接收长度
extern uint8_t ReceiveBuff_Huart2[BUFFERSIZE]; //串口2接收缓冲区







/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  uart_receive_handler(&huart1);
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
//	uint32_t temp;//计算串口接收到的数据个数
//    int re[4] = {0};
//    if(USART2 == huart2.Instance)//判断是否为串口1中断
//    {
//        if(RESET != __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE))//如果为串口1空闲
//        {
//            __HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除中断标志
//            HAL_UART_DMAStop(&huart2);//停止DMA接收
//            temp  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//获取DMA当前还有多少未填充
//            Rx_len_Huart2 =  BUFFERSIZE - temp; //计算串口接收到的数据个数
//            /*************************************************************************/
//            //接收数据处理
//						//头55，aa尾部0d，0A（一组22个）
//            if(ReceiveBuff_Huart2[0]==0x55&&ReceiveBuff_Huart2[1]==0xAA&&ReceiveBuff_Huart2[20]==0x0D&&ReceiveBuff_Huart2[21]==0x0A)
//            {
//                //坐标设置
//                if(ReceiveBuff_Huart2[2]==0x01)
//                {
//                    re[0] = ReceiveBuff_Huart2[7]  << 24 | ReceiveBuff_Huart2[6]  << 16 | ReceiveBuff_Huart2[5]  << 8 | ReceiveBuff_Huart2[4];
//                    re[1] = ReceiveBuff_Huart2[11] << 24 | ReceiveBuff_Huart2[10] << 16 | ReceiveBuff_Huart2[9]  << 8 | ReceiveBuff_Huart2[8];
//                    re[2] = ReceiveBuff_Huart2[15] << 24 | ReceiveBuff_Huart2[14] << 16 | ReceiveBuff_Huart2[13] << 8 | ReceiveBuff_Huart2[12];
//                    re[3] = ReceiveBuff_Huart2[19] << 24 | ReceiveBuff_Huart2[18] << 16 | ReceiveBuff_Huart2[17] << 8 | ReceiveBuff_Huart2[16];
//                    set_pos_y=re[0];
//                    set_pos_x=re[1];
//                    set_zangle=re[2];
//                    move_flag=re[3];
////                    printf("Y=%d    X=%d    yaw=%d  flag=%d\r\n", (int)set_pos_y,(int) set_pos_x, (int)set_zangle, move_flag);
////                    Usart2Printf("n20.val=%d\xff\xff\xff",(int)set_pos_y);
////                    Usart2Printf("n21.val=%d\xff\xff\xff",(int)set_pos_x);
////                    Usart2Printf("n22.val=%d\xff\xff\xff",(int)set_zangle);
////                    Usart2Printf("n26.val=%d\xff\xff\xff",move_flag);
//                }
//            }
//            /*************************************************************************/
//            //重新开启下一次接收
//            //memset(ReceiveBuff_Huart2,0,sizeof(ReceiveBuff_Huart2));
//            Rx_len_Huart2=0;//接收数据长度清零
//            HAL_UART_Receive_DMA(&huart2,ReceiveBuff_Huart2,BUFFERSIZE);//开启下一次接收
//        }
//    }
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 TX interrupts.
  */
void CAN2_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_TX_IRQn 0 */

  /* USER CODE END CAN2_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_TX_IRQn 1 */

  /* USER CODE END CAN2_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */
//uint32_t temp;//计算串口接收到的数据个数
//    static union
//    {
//        uint8_t date[24];
//        float ActVal[6];
//    } posture;
//    if(UART7 == huart7.Instance)//判断是否为串口1中断
//    {
//        if(RESET != __HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE))//如果为串口1空闲
//        {
//            __HAL_UART_CLEAR_IDLEFLAG(&huart7);//清除中断标志
//            HAL_UART_DMAStop(&huart7);//停止DMA接收
//            temp  = __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);//获取DMA当前还有多少未填充
//            Rx_len_Huart7 =  BUFFERSIZE - temp; //计算串口接收到的数据个数
//            /*************************************************************************/
//            //接收数据处理
//            if(Rx_len_Huart7==28)
//            {
//                for(int i=0; i<24; i++)
//                {
//                    posture.date[i]=ReceiveBuff_Huart7[i+2];
//                }
//                zangle=-posture.ActVal[0];
//                xangle=posture.ActVal[1];
//                yangle=posture.ActVal[2];
//                pos_x=posture.ActVal[3];
//                pos_y=posture.ActVal[4];
//                w_z=posture.ActVal[5];
//            }
//            /*************************************************************************/
//            //重新开启下一次接收
//            Rx_len_Huart7=0;//接收数据长度清零
//            HAL_UART_Receive_DMA(&huart7,ReceiveBuff_Huart7,BUFFERSIZE);//开启下一次接收
//        }
//    }
  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */
uint32_t temp;//计算串口接收到的数据个数
    static union
    {
        uint8_t date[24];
        float ActVal[6];
    } posture;
    if(UART8 == huart8.Instance)//判断是否为串口1中断
    {
        if(RESET != __HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE))//如果为串口1空闲
        {
            __HAL_UART_CLEAR_IDLEFLAG(&huart8);//清除中断标志
            HAL_UART_DMAStop(&huart8);//停止DMA接收
            temp  = __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);//获取DMA当前还有多少未填充
            Rx_len_Huart8 =  BUFFERSIZE - temp; //计算串口接收到的数据个数
            /*************************************************************************/
            //接收数据处理
            if(Rx_len_Huart8==28)
            {
                for(int i=0; i<24; i++)
                {
                    posture.date[i]=ReceiveBuff_Huart8[i+2];
                }
                zangle=-posture.ActVal[0];
                xangle=posture.ActVal[1];
                yangle=posture.ActVal[2];
                pos_x=posture.ActVal[3];
                pos_y=posture.ActVal[4];
                w_z=posture.ActVal[5];
            }
            /*************************************************************************/
            //重新开启下一次接收
            Rx_len_Huart8=0;//接收数据长度清零
            HAL_UART_Receive_DMA(&huart8,ReceiveBuff_Huart8,BUFFERSIZE);//开启下一次接收
        }
    }
  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
