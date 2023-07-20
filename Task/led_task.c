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
  * @brief          led RGB任务
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
  * @brief        K210任务
* @param[in]      K210_data = 5: 红色方真宝藏
* @param[in]      K210_data = 6: 红色方假宝藏
* @param[in]      K210_data = 7: 蓝色方真宝藏
* @param[in]      K210_data = 8: 蓝色方假宝藏
  * @retval         none
  */


/**
  * @brief        树莓派任务
* @param[in]      
* @param[in]      K210_data = 6: 红色方假宝藏
* @param[in]      K210_data = 7: 蓝色方真宝藏
* @param[in]      K210_data = 8: 蓝色方假宝藏
  * @retval         none
  */
void judge(void)
{
	//只有起始时发送一次起点方
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
	//K210发送的为串口     变量为K210_data
	//K210发送数据 5，6，7，8
	if(direction == Red){
		 if(USART_RX_BUF_K210[0] == '5' && send_flag == 1)
		 {
			 //小车发送信号：真  我们只需要发送一次信号
			 HAL_UART_Transmit_IT(&huart1,(uint8_t *)('1'),1);
			 //旋转180度
			 
			 //开始后退
			 send_flag = 0;
		 }else if(USART_RX_BUF_K210[0] == '6' || USART_RX_BUF_K210[0] == '8')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'0',1);
			 //旋转180
			 
			 //继续巡线
		 }else if(USART_RX_BUF_K210[0] == '7')
		 {
//			 HAL_UART_Transmit_IT(&huart1,'0',1);	 
		 }	
	}else if(direction == Blue)
	{
		 if(USART_RX_BUF_K210[0] == '5' && send_flag == 1)
		 {
			 //小车发送信号：  我们只需要发送假一次信号
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
每次重启后 指向即为0度
*/
void ll(void)
{
//	if(front_middle() == 1)
//	{
//	  	value2 = value4 =2000;
//	}
		if(left_1() == 0 || right_1() == 0)																	//直走
	{
		value2 = \
		 value4 =3000;
		station = 1;
	}
	else if( right_2() == 0) //偏向右边，要往左right_1() == 0 || 						//右2
	{
		value4 =4000;
		value2 = 3000;
		station = 1;
	}
	else if( (left_2() == 0)) //偏向左边，要往右left_1() == 0 ||				//左2
	{
		value2 =4000;
		value4 = 3000;

		station = 1;
	}
	else if( (right_3() == 0)) //偏向左边，要往右															//右3
	{
		value4 =4500;
		value2 = 3200;

		station = 1;
	}
	else if( (left_3() == 0)) //偏向左边，要往右1241										//左3
	{
		value2 =4500;
		value4 = 3200;

		station = 1;
	}
	else if( (right_4() == 0)) //偏向左边，要往右															//右4
	{
		value4 =5000;
		value2 = 4000;
		station = 1;
	}
	else if( (left_4() == 0)) //偏向左边，要往右												//左4
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
{//0代表检测到
	if(front_left()==0)																					//左前
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
	else if(front_right()==0)																									//右前
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
/*以前写的一些屎山
存放一些一会要写的屎山//
	// if((right_3() == 0 && left_3() == 0) || left_s() == 1)//前面一排的左边,中间都检测到了 需要减速
//	if(    (front_left() == 0 && front_middle() == 0)  \
//			|| (front_right() == 0 && front_middle() == 0) \
//	  || (front_right() == 0 && front_middle() == 0 && front_left() == 0))
//	{
//		//判断直行还是左转
//		if(USART_RX_BUF [i] == '1') //左转
//		{
//			value1 = value2 = -1500;
//			value3 = value4 =  8000;
//			// set = AHRSData_Packet.Heading*57.29578f - 90;
////			while(AHRSData_Packet.Heading*57.29578f > 270)
////			{
////				
////			}
//		}else if(USART_RX_BUF[i] == '2'){  //直行
//			value1 = value2 = \
//			value3 = value4 =12000;
//		}else if(USART_RX_BUF[i] == '0'){ //右行
//			value1 = value2 = 8000;
//			value3 = value4 =  -1500;	
//		}
//		HAL_Delay(50);
//		//不能检测到此情况后计数增加
//		i++;
//	}else if(front_left() == 0 && front_middle() == 1 ) // 前左检测到 前中未检测到  //只有左转
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

/*接下来需要用到的一些屎山

*/

