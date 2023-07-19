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
extern uint8_t ReceiveBuff_Huart7[255]; //串口7接收缓冲区

int testcount=0;
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



typedef struct Time
{
	int reset_time;
	int open_time;
	int close_time;
}osDelay_time;
osDelay_time delay_time={1000,1000,1000};
/**
  * @brief 	将取环送环装置设为初始化
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
  * @brief          取环 送环 推环
  * @brief          ch9==1->取环：初始为水平位置，遥控器控制气阀到环的位置，中途应该打开抓取器
  * @brief          ch9==2->送环：遥控器控制 关闭抓取器，返回初始位置
	* @brief          ch9==3->推环：遥控器控制 关闭抓取器，返回初始位置
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
