#include "stdio.h"
#include "led_task.h"
#include "cmsis_os.h"
#include "test_task.h"
#include "bsp_as5048.h"
#include "vofa_lower.h"
#include "bsp_sbus.h"
#include "usart.h"
#include "chassis_task.h"
#include "pid.h"
#include "math.h"
extern rc_info_t rc;

int lsy_num=0;
int OPMV_ANS=0;

extern speed_wheel c610[8];
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

typedef struct {
    float x;
    float y;
    float n_x;
    float n_y;
} Point;


 extern    int value1;
 extern    int value2;
 extern    int value3;
 extern    int value4;
void  rotatePoint(Point* point, double angle);

typedef struct Time
{
	int reset_time;
	int open_time;
	int close_time;
}osDelay_time;
osDelay_time delay_time={1000,1000,1000};
unsigned char len=0;
uint8_t USART_RX_BUF[USART_REC_LEN]={0,0,0,2,2,0,1,2,2,0,1,1,1,1};     //���ջ���,���USART_REC_LEN���ֽ�.
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
//	int i=0;
	int previousData = 0;  // ��һ�ε�����
    int count = 0;  // ʮ���Լ�ļ�����

    // ģ�����ݽ��գ������յ������ݴ��ڱ���receivedData��
    int receivedData = 0;  // �����յ�������Ϊ85

    // �ж�receivedData�Ƿ����80
    
	while(1)
	{
		if(USART_RX_STA&0x8000)
		{	
			
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			receivedData=USART_RX_BUF[0];
			if (receivedData > 80) 
			{
        // �ж��Ƿ���Ҫ�Լ�ʮ��
        if (count < 10)
				{
            // �ж�receivedData����һ�������Ƿ���ͬ
            if (previousData == USART_RX_BUF[0]) 
					{
                // ��������
                USART_RX_BUF[0] = receivedData;
                previousData = receivedData;
								OPMV_ANS= receivedData;
                count++;
                printf("Data updated. Count: %d\r\n", count);
           } 
					else 
					{
                // ���������ݣ�������һ�ε�����
                previousData=USART_RX_BUF[0]	;
                count=0;
                printf("Data not updated.\r \n");
            }
        }
				else
				{
					OPMV_ANS=receivedData;
            printf("Maximum checks reached.Data=%d\r\n",receivedData);
						count=0;
					previousData=300;
				}
			}
			else 
			{
        printf("Data is not greater than 80.\r\n");
				OPMV_ANS=USART_RX_BUF[0];
			}
				printf("OPMV=%d\r\n",OPMV_ANS);
			
//			HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
//			while( HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX );
			USART_RX_STA=0;
		}
	//	AHRSData2PC();
		osDelay(10);
	}
	
}

void  rotatePoint(Point* point, double angle) {
    double radian = angle * M_PI / 180.0;  // ���Ƕ�ת��Ϊ����

    // ������ת��������
    float newX = point->x * cos(radian) - point->y * sin(radian);
    float newY = point->x * sin(radian) + point->y * cos(radian);

    // ����������ֵ
    point->n_x= newX;
    point->n_y= newY;
	


}

/*5�汾ǰtest������

//			printf("speed1=%lf	speed2=%lf	now1=%f	now2=%f	\n"
//			,c610[1].pid_shudu,motor_can1[1].speed_rpm,
//			c610[1].pid_shudu,motor_can1[3].speed_rpm,
//			value2,
//			value4);
		// AHRSData2PC();
		
				int i = 0;
		//��ݮ�ɷ��͵�����
		 if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���

				if(USART_RX_BUF[0] != 'r' && USART_RX_BUF[0] != 'b')
			{
				for(i=0;i<len;i++)
				{
					printf("datai=%d\r\n",USART_RX_BUF[i]-48);
				}
			}
//			HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
//			while( HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX );
			USART_RX_STA=0;
		}
//		
//		//K210���ݽ��մ�����  �ĳ��Լ�ʹ�õĴ��ں�
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















*/
