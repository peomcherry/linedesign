#include "stdio.h"
#include "led_task.h"
#include "cmsis_os.h"
#include "test_task.h"
#include "bsp_as5048.h"
#include "vofa_lower.h"
#include "bsp_sbus.h"
float data=1.8f;
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
/**
  * @brief 	��ȡ���ͻ�װ����Ϊ��ʼ��
  * @param  none
  * @retval none
  */
void LOCATION_RESET(void)
{
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
}
/**
  * @brief          ȡ�� �ͻ� �ƻ�
  * @brief          ch9==1->ȡ������ʼΪˮƽλ�ã�ң������������������λ�ã���;Ӧ�ô�ץȡ��
  * @brief          ch9==2->�ͻ���ң�������� �ر�ץȡ�������س�ʼλ��
	* @brief          ch9==3->�ƻ���ң�������� �ر�ץȡ�������س�ʼλ��
  * @param[in]      none    
  * @retval         none
  */

void test_task(void const * argument)
{
	  
	while(1)
	{
		
	//HAL_UART_Receive_IT(&huart7, ReceiveBuff_Huart7, 1);
	//	printf("\r\n");
		printf("n16.val=%f\r\n",pos_y);
		printf("n17.val=%f\r\n",pos_x);
		printf("n18.val=%f\r\n",zangle);
		printf("n19.val=%f\r\n",w_z);
		
		
		
		
//		if(aaa==1)
//		{
//		dumiao();
//			aaa=0;
//		}
		osDelay(10);
	}
	
}
