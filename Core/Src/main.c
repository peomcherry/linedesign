/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_sbus.h"
#include "stdio.h"
#include "vofa_lower.h"
#include "commend.h"
#include "test_task.h"
//��дprintf
//int fputc(int ch,FILE *f)
//{
//    uint8_t temp[1]= {ch};
//    HAL_UART_Transmit(&huart6,temp,1,2);
//		return 0;
//}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
//int fputc(int ch, FILE *f)
//{
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}
//int fputc(int ch, FILE *f)
//{
//  HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}
#define BUFFERSIZE 255	//�ɽ��յ����������
//uint8_t Rx_len_Huart7;//����6���ճ���
//uint8_t ReceiveBuff_Huart7[BUFFERSIZE]; //����6���ջ�����
//float pos_x=0;//����X--ZBx
//float pos_y=0;//����Y--ZBy
//float zangle=0;//�����
//float xangle=0;//������
//float yangle=0;//�����
//float w_z=0;//�������

//float set_pos_x;
//float set_pos_y;
//float set_zangle;
//int move_flag;
//uint8_t shot_flag=0;
//������ ����2

extern uint8_t ReceiveBuff_Huart7[BUFFERSIZE]; //����7���ջ�����
uint8_t Rx_len_Huart2;//����2���ճ���
uint8_t ReceiveBuff_Huart2[BUFFERSIZE]; //����2���ջ�����


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t cmd = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_TIM2_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
  dbus_uart_init();
	
	HAL_TIM_Base_Start_IT(&htim2);
	//���ｫ����6��shell����ע��
//	extern void userShellInit(void);//���ڸ�rtos��֧�ֺ��Զ���ʼ��,���ֶ���ʼ��
//  userShellInit();
  my_can_filter_init_recv_all(&hcan1);
  can_filter_recv_special(&hcan2);
	HAL_UART_Receive_IT(&huart1, &cmd, 1);
	//HAL_UART_Receive_IT(&huart7, ReceiveBuff_Huart7, 1);

    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);//ʹ�ܴ���7 IDLE�ж�
    //__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);//ʹ�ܴ���7 IDLE�ж�
   // HAL_UART_Receive_DMA(&huart7,ReceiveBuff_Huart7,BUFFERSIZE);//ʹ�ܽ���
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//ʹ�ܴ���2 IDLE�ж�
			HAL_UART_Receive_IT(&huart8, (uint8_t *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
	  HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer_K210, RXBUFFERSIZE_K210);
	 // HAL_UART_Receive_IT(&huart8, (uint8_t *)aRxBuffer_K210, RXBUFFERSIZE_K210);

    /*�ڶ�������ĿǰΪ���� ���Ϊ������Ҫ��ȡ��ַ��*/
   // HAL_UART_Receive_DMA(&huart2,ReceiveBuff_Huart2,BUFFERSIZE);//ʹ�ܽ���
	var_init();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART1){
//		if(cmd == 0xA1){

//			HAL_UART_Transmit(&huart1, "Toggle LED0!\r\n", sizeof("Toggle LED0!\r\n"),10000);
//		}
//		if(cmd == 0xA2){
//			HAL_UART_Transmit(&huart1, "Toggle LED1!\r\n", sizeof("Toggle LED1!\r\n"),10000);
//		}
//		HAL_UART_Receive_IT(&huart1, &cmd, 1);
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */