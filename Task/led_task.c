#include "led_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_can.h"
#include "usart.h"
#include "stdio.h"
#include "test_task.h"
#include "cmath"
#include "string.h"
#define straight_speed 3500
#define small_turn 700
#define big_turn  1000
#define ABS(x) ((x) < 0 ? -(x) : (x))
extern motor_measure_t motor_can1[];
extern speed_wheel c610[8];
extern 	int flag;

extern float pos_x;//坐标X--ZBx
extern float pos_y;//坐标Y--ZBy
extern float zangle;//航向角
extern float xangle;//俯仰角
extern float yangle;//横滚角
extern float w_z;//航向角速

extern unsigned char station;
extern int lsy_num;
char time_cnt=0;
uint8_t turn_cnt=0;
speed_value base_speed={1000,1000,1000,1000};

int value1=2000;
int value2=2000;
int value3=2000;
int value4=2000;
int last_misssion;
int result[8]={0};
unsigned char start_flag=1;
unsigned char data[200];
unsigned char direction=0;
unsigned char K210_data[200];
uint8_t send_flag=1;
unsigned char judge_flag=1;
extern uint8_t USART_RX_BUF_K210[USART_REC_LEN_K210]; 
extern AHRSData_Packet_t AHRSData_Packet;
extern uint8_t USART_RX_BUF[USART_REC_LEN];
double Last_ANGLE_AHR298=0;
int Turn_left_flag=0;
int Turn_right_flag=0;
int Turn_straight_flag=0;

int Steering_Judgment_Flag=0;

	int i = 0;
double Angle_AHR;
  /* USER CODE BEGIN 2 */
int grayscale_sensor_judging(void);
void Left_Handed_Rotation(void);
void Right_Handed_Rotation(void);

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
int Test_Attitude_judgment(void);
void Patrol_execution(int mission);
/**
  * @brief          led RGB任务
* @param[in]      pvParameters: NULL F1:L F0:C  C2:Right
  * @retval         none
  */
unsigned int t;

/*
主要任务执行的地方

*/




void led_task(void const * argument)
{
	//int mission;
	//Last_ANGLE_AHR298=ANGLE_AHR298;
	Steering_Judgment_Flag=0;
	while(1)
	{
	//	printf("Steering_Judgment_Flag=%d\r\n",Steering_Judgment_Flag);
	//	printf("delta_angle=%d\r\n",delta_angle);
		//lsy_num=grayscale_sensor_judging();
		osDelay(5);
		Patrol_execution(Test_Attitude_judgment());
		judge();
	}
}
/*主函数曾经的屎山

		//mission=Test_Attitude_judgment();
		//line_walking();
*/




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
/*	
	K210发送的为串口     变量为K210_data
	K210发送数据 5，6，7，8

*/
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
	if(grayscale_sensor_judging()>=4)
	{
		value2 = \
		 value4 =Straight_Speed;
		station = 1;
	}
	else if(left_1() == 0 || right_1() == 0)																	//直走
	{
		value2 = \
		 value4 =Straight_Speed;
		station = 1;
	}
	else if( right_2() == 0) //偏向右边，要往左right_1() == 0 || 						//右2
	{
		value4 =Straight_Speed+level1_Differential_speed;
		value2 = Straight_Speed;
		station = 1;
	}
	else if( (left_2() == 0)) //偏向左边，要往右left_1() == 0 ||				//左2
	{
		value2 =Straight_Speed+level1_Differential_speed;
		value4 = Straight_Speed;

		station = 1;
	}
	else if( (right_3() == 0)) //偏向左边，要往右															//右3
	{
		value4 =Straight_Speed+level2_Differential_speed;
		value2 = Straight_Speed+200;

		station = 1;
	}
	else if( (left_3() == 0)) //偏向左边，要往右1241										//左3
	{
		value2 =Straight_Speed+level2_Differential_speed;
		value4 = Straight_Speed+200;

		station = 1;
	}
	else if( (right_4() == 0)) //偏向左边，要往右															//右4
	{
		value4 =Straight_Speed+level3_Differential_speed;
		value2 = Straight_Speed+level2_Differential_speed;
		station = 1;
	}
	else if( (left_4() == 0)) //偏向左边，要往右												//左4
	{
		value2 =Straight_Speed+level3_Differential_speed;
		value4 = Straight_Speed+level2_Differential_speed;
		station = 1;
	}
	else
	{
	value2=0;
	value4=0;
	}
}
void line_walking(void)
{
	//0代表检测到
	if(front_left()==0)																					//左前
	{
			Last_ANGLE_AHR298=ANGLE_AHR298;
//		int delta_angle = ((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);

    // 如果差值超过90度，则触发提示信息
    if (delta_angle <= 80 && delta_angle >= 280) 
		{
	   		while(delta_angle <= 80 && delta_angle >= 280)//ABS(ANGLE_AHR298-Last_ANGLE_AHR298)<=85||ABS(ANGLE_AHR298-Angle_AHR)<=85||ANGLE_AHR298<=90)									
	   	{
	   		value2=3500;
	   		value4=-500;	
	   		printf("last11=%f\r\n",Last_ANGLE_AHR298);
	   	  printf("this11=%f\r\n",ANGLE_AHR298);
	   	}
		}
//			Last_ANGLE_AHR298=ANGLE_AHR298;
//		printf("last=%f\r\n",Last_ANGLE_AHR298);
//		printf("this=%f\r\n",ANGLE_AHR298);
//				if(0<Last_ANGLE_AHR298&&Last_ANGLE_AHR298<90){
//			Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
//			Angle_AHR +=360;
//		}
		
		value2=3000;
		value4=3000;	
	}
	else if(front_right()==0)																									//右前
	{
			Last_ANGLE_AHR298=ANGLE_AHR298;
//			int delta_angle = ((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);//340-330=10//应该是转到90，才跳出循环
			if (delta_angle <= 80 || delta_angle >= 280) 
					{
									while(delta_angle <= 90 || delta_angle >= 280)//ABS(ANGLE_AHR298-Last_ANGLE_AHR298)<=82||ABS(ANGLE_AHR298-Angle_AHR)<=82||(ANGLE_AHR298<=360&&ANGLE_AHR298>=270))
									{
										value2=-500;
										value4=4500;
									}
					}
					value2=3000;
					value4=3000;
    // 如果差值超过90度，则触发提示信息
		//碰到左边，需要转90，
 //  Last_ANGLE_AHR298=ANGLE_AHR298;
//					if(270<Last_ANGLE_AHR298&&Last_ANGLE_AHR298<360){
//						Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
//						Angle_AHR -=360;
//					}
			
//					printf("yylast=%f\r\n",Last_ANGLE_AHR298);
//					printf("yythis=%f\r\n",ANGLE_AHR298);
						//差值，达到90度，应该跳出去
						//其中有较大跳变‘350-10=340，这种，虽然没有达到90度，但是差值大于90，需要且来约束
	}
	else{
		ll();
	}
}

/*
状态判断函数
下面有几个状态，首先在别处判断，前两个是否被检测到，如果被检测到了，就turn，没检测到就linefollow
规定一下，如果是转弯就是
1					单侧转弯
2					是两侧都检测到了
3					只有左检测到了//左转标志位
4					只有右检测到了//右转标志位
5					直走
6					8灰多路检测到，开始转
7					前左识别
8					前右识别
9					十字识别
其他


*/
int Test_Attitude_judgment(void)
{
	int judgment_mission=0;
	if((front_left()==0||front_right()==0))//&&front_middle()!=0)//z左右，但没有直线
	{
		if(front_left()==0&&front_right()==0)																//左右
		{
			Turn_left_flag=1;
			Turn_right_flag=1;
			judgment_mission=2;
			printf("double\r\n");
		}
		else if(front_left()==0&&front_right())															//左转------
		{
			Turn_left_flag=1;
			judgment_mission=3;
			printf("left\r\n");
		}	
		else if(front_left()&&front_right()==0)															//右转
			{
			Turn_right_flag=1;
			judgment_mission=4;
			printf("right\r\n");
		}
		
	}
	else if(grayscale_sensor_judging()==1||grayscale_sensor_judging()==2)//巡线							||grayscale_sensor_judging()==3)//||grayscale_sensor_judging()==4) 	//!(left_1()&&left_2()&&left_3()&&left_4()&&right_1()&&right_2()&&right_3()&&right_4()))//如果检测到了，那么其中有一个是0，若全且，则有一个有
	{
		judgment_mission=5;
	}
	else if(grayscale_sensor_judging()>=3)																//多路灰度
	{
			judgment_mission=6;
	}
//	else if(front_left()==0&&front_middle()==0)														//前左判断
//	{
//		Turn_straight_flag=1;
//		Turn_left_flag=1;
//		printf("str_left");
//		judgment_mission=7;
//	}
//	else if(front_right()==0&&front_middle()==0)													//前右判断
//	{
//		Turn_straight_flag=1;
//		Turn_right_flag=1;
//		printf("str_right");
//		judgment_mission=8;
//	}
//	else if(front_right()==0&&front_middle()==0&&front_left()==0)					//十字判断
//	{
//		Turn_straight_flag=1;	
//		Turn_left_flag=1;
//		Turn_right_flag=1;
//		printf("str_double");
//		judgment_mission=9;
//	}
	else
	{
		judgment_mission=0;
	}
	if(judgment_mission!=last_misssion)
	{
	printf("mission=%d\r\n",judgment_mission);
	}
	 last_misssion=judgment_mission;
//	printf("num=%d\r\n",grayscale_sensor_judging());
	return judgment_mission;
}
/*
首先需要一个函数判断他现在是左转还是右转，也就是上面那个是状态判断，下面这个用于检测标志位


*/
void Patrol_execution(int mission)
{
	//int 
	//grayscale_sensor_judging()
	
	
	switch(mission)
	{
		case 2://先写成左转 
		{
	  		if(grayscale_sensor_judging()>=4)
	  		{
         Left_Handed_Rotation();
				}
		    	break;
		}			
		case 3://左转
		{
			if(grayscale_sensor_judging()>=4)
			{
				Left_Handed_Rotation();
			}				
			break;
		}		
		case 4://右转
		{
			if(grayscale_sensor_judging()>=4)
			{
				Right_Handed_Rotation();
			}
			break;
		}		
		case 5://巡线
		{
			ll();
			break;
		}		
		case 6://多路灰，
		{
			if(Turn_left_flag&&Turn_right_flag)//两路，现在还是向左
			{
				printf("1\r\n");
					if(USART_RX_BUF[Steering_Judgment_Flag]==0)
					{
						Left_Handed_Rotation();
					}
					else if(USART_RX_BUF[Steering_Judgment_Flag]==2)
					{
						Right_Handed_Rotation();
					}
					Steering_Judgment_Flag++;
					printf("double turn over");
			}
			else if(Turn_left_flag==1&&Turn_right_flag==0)
			{
				printf("2\r\n");
							Left_Handed_Rotation();
			}
			else if(Turn_right_flag==1&&Turn_left_flag==0)
			{
				printf("3\r\n");
							Right_Handed_Rotation();
			}
			else
			{
		    	    Turn_left_flag=0;
					    Turn_right_flag=0;
					    ll();
			}
			break;
		}		
		case 7://前左
		{
//			if(USART_RX_BUF[Steering_Judgment_Flag]=='1')
//	  	{
//	  		Turn_left_flag=0;
//				ll();
//	  	}
//			else if(USART_RX_BUF[Steering_Judgment_Flag]=='0')
//			{
				Left_Handed_Rotation();
//				ll();
//			}
//			Steering_Judgment_Flag++;
			break;
		}
		case 8://前右
		{
//	  	if(USART_RX_BUF[Steering_Judgment_Flag]=='1')
//	  	{
//	  		Turn_right_flag=0;
//				ll();
//	  	}
//			else if(USART_RX_BUF[Steering_Judgment_Flag]=='2')
//			{
				Right_Handed_Rotation();
//				ll();
//			}
//			Steering_Judgment_Flag++;
			break;
		}
		case 9://十字
		{
//			if(USART_RX_BUF[Steering_Judgment_Flag]=='1')
//	  	{
//	  		Turn_left_flag=0;
//				ll();
//	  	}
//			else if(USART_RX_BUF[Steering_Judgment_Flag]=='0')
//			{
//				Left_Handed_Rotation();
//				ll();
//			}
//			else if(USART_RX_BUF[Steering_Judgment_Flag]=='2')
//			{
				Right_Handed_Rotation();
//				ll();
//			}
//			Steering_Judgment_Flag++;
			break;
		}
		case 0://停止
		{
				ll();
			break;
		}		

	}
}
//这个函数用于检测哪几个灰度被调用了grayscale_sensor_judging()的返回值是个数
/*原本于2.2写的一些屎山，现在用来判断灰度个数
		


*/
int grayscale_sensor_judging(void)
{
		int i=0;
		memset(result, 0, sizeof(result));
		if (left_1() == 0) {
        result[3]= 1;
    }
    if (right_1() == 0) {
        result[4]= 1;
    }
    if (left_2() == 0) {
        result[2]= 1;
    }
    if (right_2() == 0) {
        result[5]= 1;
    }
    if (left_3() == 0) {
        result[1]= 1;
    }
    if (right_3() == 0) {
        result[6]= 1;
    }
    if (left_4() == 0) {
        result[0]= 1;
    }
    if (right_4() == 0) {
        result[7]= 1;
    }
		static int fit=0;
		fit=0;
		for(i=0;i<8;i++)
		{
			fit+=result[i];
		}
		//left_1()+left_2()+left_3()+left_4()+right_4()+right_3()+right_2()+right_1();//首先对整个灰度进行一个判断，理论上一定会有灰度的识别，如果都没扫到应该是0，正常巡线情况应该是6，遇到路口应该是4或3
		return fit;
}

void Left_Handed_Rotation(void)
{
								Last_ANGLE_AHR298=ANGLE_AHR298;
									printf("ok1\r\n");
		printf("delta_angle=%d\r\n",delta_angle);
//             		int delta_angle = (int)(fmod((ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0), 360.0));//((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);
                // 如果差值超过90度，则触发提示信息
                if (delta_angle <= Turn_quart || delta_angle >= Turn_quart_270) 
             		{
									
									printf("ok2\r\n");
									while(delta_angle <= Turn_quart || delta_angle >= Turn_quart_270)//ABS(ANGLE_AHR298-Last_ANGLE_AHR298)<=85||ABS(ANGLE_AHR298-Angle_AHR)<=85||ANGLE_AHR298<=90)									
             	   	{
             	   		value2=Straight_Speed;
             	   		value4=(-1)*Straight_Speed;	
//             	   		printf("last1_1eft=%f\r\n",Last_ANGLE_AHR298);
//             	   	  printf("this1_left=%f\r\n",ANGLE_AHR298);
										printf("delta_angle=%d\r\n",delta_angle);
             	   	}
		            	Turn_left_flag=0;
				        	Turn_right_flag=0;
									printf("left_turn_over");
									
             		}
								printf("yuan_x=%f\r\n",pos_x);
								printf("yuan_y=%f\r\n",pos_y);
								value2=Straight_Speed;
								value4=Straight_Speed;
}
void Right_Handed_Rotation(void)
{
			
	       	Last_ANGLE_AHR298=ANGLE_AHR298;
	//	    	      		int delta_angle = (int)(fmod((ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0), 360.0));//340-330=10//应该是转到90，才跳出循环
		    	if (delta_angle <= Turn_quart || delta_angle >= Turn_quart_270) 
		    	{
		    					while(delta_angle <= Turn_quart || delta_angle >= Turn_quart_270)//ABS(ANGLE_AHR298-Last_ANGLE_AHR298)<=82||ABS(ANGLE_AHR298-Angle_AHR)<=82||(ANGLE_AHR298<=360&&ANGLE_AHR298>=270))
		    					{
		    						value2=(-1)*Straight_Speed;
		    						value4=Straight_Speed;
//										printf("last1_right=%f\r\n",Last_ANGLE_AHR298);
//             	   	  printf("this1_right=%f\r\n",ANGLE_AHR298);
//										printf("delta_angle=%d\r\n",delta_angle);
		    					}
		           	Turn_right_flag=0;
				       	Turn_right_flag=0;
								printf("right_turn_over");
		    	}
					printf("yuan_x=%f\r\n",pos_x);
					printf("yuan_y=%f\r\n",pos_y);
		    	value2=Straight_Speed;
		    	value4=Straight_Speed;
			
}

/*以前写的一些屎山
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


if(fit==0||fit==8)//没巡到线
	{
		turn_flag=10;
		//stop
	//	line_walking(turn_flag);
	}
	else if(fit==2||fit==1)//沿直线巡线模式
	{
		if(result[3]||result[4])//如果他俩寻到线了，则他俩应该都是0
		{
			//直走
			turn_flag=8;//表示直走
		}
		else
		{
		   for(i=0;i<8;i++)//历遍8次找到哪路找到了线，将此路数据传回
		   {
		   	if(result[i])
		   	{
		   			turn_flag=i;//转弯等级
					break;
		   	}
		   }
		}
	//	line_walking(turn_flag);
		//station = 2;
	}
	else if(fit>=4||fit<=6)
	{
		if(result[0]&&result[1]&&result[2])
		{
			turn_flag=11;
			//左大转
		}else if(result[5]&&result[6]&&result[7])
		{
			turn_flag=9;
			//右大转
		}
		else
		{
			if(result[3]||result[4])//如果他俩寻到线了，则他俩应该都是0
		{
			//直走
			turn_flag=8;//表示直走
		}
		else
		{
		   for(i=0;i<8;i++)//历遍8次找到哪路找到了线，将此路数据传回
		   {
		   	if(result[i])
		   	{
		   			turn_flag=i;//转弯等级
					break;
		   	}
		   }
		}
	//	line_walking(turn_flag);
		//station = 2;
		}
		//station = 2;
	//	line_walking(turn_flag);
	}
	
*/

/*接下来需要用到的一些屎山
//			Last_ANGLE_AHR298=ANGLE_AHR298;
//		printf("last=%f\r\n",Last_ANGLE_AHR298);
//		printf("this=%f\r\n",ANGLE_AHR298);
//				if(0<Last_ANGLE_AHR298&&Last_ANGLE_AHR298<90){
//			Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
//			Angle_AHR +=360;
//		}
			
//					Last_ANGLE_AHR298=ANGLE_AHR298;
////	      	printf("last=%f\r\n",Last_ANGLE_AHR298);
////	      	printf("this=%f\r\n",ANGLE_AHR298);
//				if(0<Last_ANGLE_AHR298||Last_ANGLE_AHR298<90)
//				{
//		      	Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
//						Angle_AHR+=360;	//Angle_AHR -=360;
//				}else
//		    {
//		    	Angle_AHR =(AHRSData_Packet.Heading*57.29578f);			
//		    }
//				while(__fabs(ANGLE_AHR298-Last_ANGLE_AHR298)<=80)									
//		    {
//		    	  value2=3000;
//		    	  value4=-3000;	
////		    	  printf("last11=%f\r\n",Last_ANGLE_AHR298);
////		        printf("this11=%f\r\n",ANGLE_AHR298);
//		    }
//		    value2=3000;
//		    value4=3000;	
//				Turn_left_flag=0;
*/

