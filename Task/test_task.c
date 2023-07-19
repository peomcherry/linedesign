#include "stdio.h"
#include "led_task.h"
#include "cmsis_os.h"
#include "test_task.h"
#include "bsp_as5048.h"
#include "vofa_lower.h"
#include "bsp_sbus.h"
#include "usart.h"
extern rc_info_t rc;
extern int aa;
extern int aaa;
char control_flag=1;
extern uint8_t ReceiveBuff_Huart7[255]; //����7���ջ�����

int testcount=0;
extern float pos_x;//����X--ZBx
extern float pos_y;//����Y--ZBy
extern float zangle;//�����
extern float xangle;//������
extern float yangle;//�����
extern float w_z;//�������

extern float set_pos_x;
extern float set_pos_y;
extern float set_zangle;
extern int move_flag;



typedef struct Time
{
	int reset_time;
	int open_time;
	int close_time;
}osDelay_time;
osDelay_time delay_time={1000,1000,1000};
unsigned char len=0;
uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA=0;       //����״̬���	  
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���


//_K210   K210������ض���
uint8_t USART_RX_BUF_K210[USART_REC_LEN_K210];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA_K210=0;       //����״̬���	  

uint8_t aRxBuffer_K210[RXBUFFERSIZE_K210];//HAL��ʹ�õĴ��ڽ��ջ���
/**
  * @brief 	��ȡ���ͻ�װ����Ϊ��ʼ��
  * @param  none
  * @retval none
  */

/**
  * @brief          ȡ�� �ͻ� �ƻ�
  * @brief          ����2����ݮ��
  * @brief          ����6��K210
	* @brief          ����8����λ��
  * @param[in]      none    
  * @retval         none
  */

void test_task(void const * argument)
{
	  
	while(1)
	{
		 AHRSData2PC();
				int i = 0;
		//��ݮ�ɷ��͵�����
//		 if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���

//				if(USART_RX_BUF[0] != 'r' && USART_RX_BUF[0] != 'b')
//			{
//				for(i=0;i<len;i++)
//				{
//					printf("datai=%d\r\n",USART_RX_BUF[i]-48);
//				}
//			}
////			HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
////			while( HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX );
//			USART_RX_STA=0;
//		}
//		
//		//K210���ݽ��մ�������  �ĳ��Լ�ʹ�õĴ��ں�
//		if(USART_RX_STA_K210&0x8000)
//		{					   
//			len=USART_RX_STA_K210&0x3fff;//�õ��˴ν��յ������ݳ���
//		//	printf("\r\n�����͵���ϢΪ:\r\n");
//			HAL_UART_Transmit(&huart6,(uint8_t*)USART_RX_BUF_K210,len,1000);	//���ͽ��յ�������
//			while( HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX );
//			USART_RX_STA_K210=0;
//		}

//		printf("n16.val=%f\r\n",pos_y);
//		printf("n17.val=%f\r\n",pos_x);
//		printf("n18.val=%f\r\n",zangle);
//		printf("n19.val=%f\r\n",w_z);
//		

		osDelay(10);
	}
	
}