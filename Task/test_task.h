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
  * @brief          led RGB����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void test_task(void const * argument);
void LOCATION_RESET(void);

#define HAL_MAX_DELAY      0xFFFFFFFFU
#define USART_REC_LEN  			200  		//�����������ֽ��� 200
#define EN_USART1_RX 			1			//ʹ�ܣ�1��/��ֹ��0������1����

#define M_PI 3.14159
	  	
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; 	//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA;         			//����״̬���	

#define RXBUFFERSIZE   1 					//�����С
extern uint8_t aRxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer



#define USART_REC_LEN_K210  			200  		//�����������ֽ��� 200
#define EN_USART1_RX_K210 			1			//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern uint8_t  USART_RX_BUF_K210[USART_REC_LEN_K210]; 	//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USART_RX_STA_K210;         			//����״̬���	

#define RXBUFFERSIZE_K210   1 					//�����С
extern uint8_t aRxBuffer_K210[RXBUFFERSIZE_K210];			//HAL��USART����Buffer




#endif


