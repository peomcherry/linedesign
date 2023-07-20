#include "led_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_can.h"
#include "usart.h"
#include "stdio.h"
#include "test_task.h"
#define straight_speed 3500
#define small_turn 700
#define big_turn  1000
#define ABS(x) ((x) < 0 ? -(x) : (x))
extern motor_measure_t motor_can1[];
extern speed_wheel c610[8];
extern 	int flag;

extern unsigned char station;
char time_cnt=0;
uint8_t turn_cnt=0;
speed_value base_speed={1000,1000,1000,1000};

int value1=2000;
int value2=2000;
int value3=2000;
int value4=2000;
unsigned char start_flag=1;
unsigned char data[200];
unsigned char direction=0;
unsigned char K210_data[200];
uint8_t send_flag=1;
unsigned char judge_flag=1;
extern uint8_t USART_RX_BUF_K210[USART_REC_LEN_K210]; 
extern AHRSData_Packet_t AHRSData_Packet;
double Last_ANGLE_AHR298=0;
	int i = 0;
double Angle_AHR;
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

void LED_Delay(uint32_t time)
{
    //HAL_Delay(time);
    osDelay(time);
}
void LED_A(uint8_t flag)
{
    if(flag)
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);
}
void LED_B(uint8_t flag)
{
    if(flag)
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
}
void LED_ALL_OFF()
{
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_SET);
}
void RUN_LED(void)
{
    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
    LED_Delay(100);

    LED_ALL_OFF();
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);
    LED_Delay(100);

}
void line_walking(void);
void judge(void);
/**
  * @brief          led RGB����
* @param[in]      pvParameters: NULL F1:L F0:C  C2:Right
  * @retval         none
  */
unsigned int t;


void led_task(void const * argument)
{
	Last_ANGLE_AHR298=ANGLE_AHR298;
	while(1){
		osDelay(5);
	line_walking();
		judge();
	}
}


/**
  * @brief        K210����
* @param[in]      K210_data = 5: ��ɫ���汦��
* @param[in]      K210_data = 6: ��ɫ���ٱ���
* @param[in]      K210_data = 7: ��ɫ���汦��
* @param[in]      K210_data = 8: ��ɫ���ٱ���
  * @retval         none
  */


/**
  * @brief        ��ݮ������
* @param[in]      
* @param[in]      K210_data = 6: ��ɫ���ٱ���
* @param[in]      K210_data = 7: ��ɫ���汦��
* @param[in]      K210_data = 8: ��ɫ���ٱ���
  * @retval         none
  */
void judge(void)
{
	//ֻ����ʼʱ����һ����㷽
	if(USART_RX_BUF[0] == 'r' && judge_flag == 1)
	{
		direction = Red;
		judge_flag =0;
	}else if(USART_RX_BUF[0] == 'b' && judge_flag == 1)
	{
		direction = Blue;
		judge_flag =0;
	}

}

void judge_treasure(void)
{
	//K210���͵�Ϊ����     ����ΪK210_data
	//K210�������� 5��6��7��8
	if(direction == Red){
		 if(USART_RX_BUF_K210[0] == '5' && send_flag == 1)
		 {
			 //С�������źţ���  ����ֻ��Ҫ����һ���ź�
			 HAL_UART_Transmit_IT(&huart1,(uint8_t *)('1'),1);
			 //��ת180��
			 
			 //��ʼ����
			 send_flag = 0;
		 }else if(USART_RX_BUF_K210[0] == '6' || USART_RX_BUF_K210[0] == '8')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'0',1);
			 //��ת180
			 
			 //����Ѳ��
		 }else if(USART_RX_BUF_K210[0] == '7')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'0',1);	 
		 }	
	}else if(direction == Blue)
	{
		 if(USART_RX_BUF_K210[0] == '5' && send_flag == 1)
		 {
			 //С�������źţ�  ����ֻ��Ҫ���ͼ�һ���ź�
//			 HAL_UART_Transmit_IT(&huart1,'0',1);
			 send_flag = 0;
		 }else if(USART_RX_BUF_K210[0] == '6' || USART_RX_BUF_K210[0] == '8')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'0',1);
		 }else if(USART_RX_BUF_K210[0] == '7')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'1',1);	 
		 }		
	}
}
/*
		180
			|
			|
90----------270
			|
			|
			|
			0(360)
ÿ�������� ָ��Ϊ0��
*/
void ll(void)
{
//	if(front_middle() == 1)
//	{
//	  	value2 = value4 =2000;
//	}
		if(left_1() == 0 || right_1() == 0)																	//ֱ��
	{
		value2 = \
		 value4 =3000;
		station = 1;
	}
	else if( right_2() == 0) //ƫ���ұߣ�Ҫ����right_1() == 0 || 						//��2
	{
		value4 =4000;
		value2 = 3000;
		station = 1;
	}
	else if( (left_2() == 0)) //ƫ����ߣ�Ҫ����left_1() == 0 ||				//��2
	{
		value2 =4000;
		value4 = 3000;

		station = 1;
	}
	else if( (right_3() == 0)) //ƫ����ߣ�Ҫ����															//��3
	{
		value4 =4500;
		value2 = 3200;

		station = 1;
	}
	else if( (left_3() == 0)) //ƫ����ߣ�Ҫ����1241										//��3
	{
		value2 =4500;
		value4 = 3200;

		station = 1;
	}
	else if( (right_4() == 0)) //ƫ����ߣ�Ҫ����															//��4
	{
		value4 =5000;
		value2 = 4000;
		station = 1;
	}
	else if( (left_4() == 0)) //ƫ����ߣ�Ҫ����												//��4
	{
		value2 =5000;
		value4 = 4000;
		station = 1;
	}
	else
	{
	value2=0;
	value4=0;
	}
}
void line_walking(void)
{//0�����⵽
	if(front_left()==0)																					//��ǰ
	{
			Last_ANGLE_AHR298=ANGLE_AHR298;
		printf("last=%f\r\n",Last_ANGLE_AHR298);
		printf("this=%f\r\n",ANGLE_AHR298);
				if(0<Last_ANGLE_AHR298||Last_ANGLE_AHR298<90){
			Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
			Angle_AHR -=360;
		}
		while(__fabs(ANGLE_AHR298-Last_ANGLE_AHR298)<=90)									
		{
			value2=3500;
			value4=-800;	
			printf("last11=%f\r\n",Last_ANGLE_AHR298);
		  printf("this11=%f\r\n",ANGLE_AHR298);
		}
		value2=3000;
		value4=3000;	
	}
	else if(front_right()==0)																									//��ǰ
	{
		
		Last_ANGLE_AHR298=ANGLE_AHR298;
		if(270<Last_ANGLE_AHR298||Last_ANGLE_AHR298<360){
			Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
			Angle_AHR +=360;
		}

		printf("yylast=%f\r\n",Last_ANGLE_AHR298);
		printf("yythis=%f\r\n",ANGLE_AHR298);
		while(ABS(ANGLE_AHR298-Last_ANGLE_AHR298)<=90)
		{
			value2=-800;
			value4=3500;
		}
		value2=3000;
		value4=3000;
	}
	else{
		ll();
	}


 
	


}
/*��ǰд��һЩʺɽ
���һЩһ��Ҫд��ʺɽ//
	// if((right_3() == 0 && left_3() == 0) || left_s() == 1)//ǰ��һ�ŵ����,�м䶼��⵽�� ��Ҫ����
//	if(    (front_left() == 0 && front_middle() == 0)  \
//			|| (front_right() == 0 && front_middle() == 0) \
//	  || (front_right() == 0 && front_middle() == 0 && front_left() == 0))
//	{
//		//�ж�ֱ�л�����ת
//		if(USART_RX_BUF [i] == '1') //��ת
//		{
//			value1 = value2 = -1500;
//			value3 = value4 =  8000;
//			// set = AHRSData_Packet.Heading*57.29578f - 90;
////			while(AHRSData_Packet.Heading*57.29578f > 270)
////			{
////				
////			}
//		}else if(USART_RX_BUF[i] == '2'){  //ֱ��
//			value1 = value2 = \
//			value3 = value4 =12000;
//		}else if(USART_RX_BUF[i] == '0'){ //����
//			value1 = value2 = 8000;
//			value3 = value4 =  -1500;	
//		}
//		HAL_Delay(50);
//		//���ܼ�⵽��������������
//		i++;
//	}else if(front_left() == 0 && front_middle() == 1 ) // ǰ���⵽ ǰ��δ��⵽  //ֻ����ת
//	{
//		value1 = value2 = -1500;
//		value3 = value4 =  8000;
//	}else if(front_right() == 0 && front_middle() == 1)
//	{
//			value1 = value2 = 8000;
//			value3 = value4 =  -1500;			
//	}
	
// if((right_4() == 0 && left_4() == 0))
//	{
//		value1 = value2 = 0;
//		value3 = value4 =  0;
//		station =1;
//	}else  if(right_4() == 0 )
//	{
//		value1 = value2 = 8000;
//		value3 = value4 =  -1500;
//		HAL_Delay(150);
//		station =1;
//		
//	}else  if(left_4() == 0  )
//	{
//		printf("cnt=%d\r\n",time_cnt);
//		value1 = value2 = -1500;
//		value3 = value4 =  8000;
//		HAL_Delay(150);
//		station =1;
//		
//	}
//	else 

*/

/*��������Ҫ�õ���һЩʺɽ

*/

