#ifndef __TEST_TASK_H
#define __TEST_TASK_H

#include "main.h"
#include "struct_typedef.h"

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void test_task(void const * argument);
void LOCATION_RESET(void);

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define USART_REC_LEN  			200  		//定义最大接收字节数 200
#define EN_USART1_RX 			1			//使能（1）/禁止（0）串口1接收

#define M_PI 3.14159
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 	//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         			//接收状态标记	

#define RXBUFFERSIZE   1 					//缓存大小
extern uint8_t aRxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer



#define USART_REC_LEN_K210  			200  		//定义最大接收字节数 200
#define EN_USART1_RX_K210 			1			//使能（1）/禁止（0）串口1接收
	  	
extern uint8_t  USART_RX_BUF_K210[USART_REC_LEN_K210]; 	//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA_K210;         			//接收状态标记	

#define RXBUFFERSIZE_K210   1 					//缓存大小
extern uint8_t aRxBuffer_K210[RXBUFFERSIZE_K210];			//HAL库USART接收Buffer




#endif


