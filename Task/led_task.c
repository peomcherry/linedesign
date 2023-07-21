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

extern float pos_x;//����X--ZBx
extern float pos_y;//����Y--ZBy
extern float zangle;//�����
extern float xangle;//������
extern float yangle;//�����
extern float w_z;//�������

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
  * @brief          led RGB����
* @param[in]      pvParameters: NULL F1:L F0:C  C2:Right
  * @retval         none
  */
unsigned int t;

/*
��Ҫ����ִ�еĵط�

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
/*������������ʺɽ

		//mission=Test_Attitude_judgment();
		//line_walking();
*/




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
/*	
	K210���͵�Ϊ����     ����ΪK210_data
	K210�������� 5��6��7��8

*/
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
	if(grayscale_sensor_judging()>=4)
	{
		value2 = \
		 value4 =Straight_Speed;
		station = 1;
	}
	else if(left_1() == 0 || right_1() == 0)																	//ֱ��
	{
		value2 = \
		 value4 =Straight_Speed;
		station = 1;
	}
	else if( right_2() == 0) //ƫ���ұߣ�Ҫ����right_1() == 0 || 						//��2
	{
		value4 =Straight_Speed+level1_Differential_speed;
		value2 = Straight_Speed;
		station = 1;
	}
	else if( (left_2() == 0)) //ƫ����ߣ�Ҫ����left_1() == 0 ||				//��2
	{
		value2 =Straight_Speed+level1_Differential_speed;
		value4 = Straight_Speed;

		station = 1;
	}
	else if( (right_3() == 0)) //ƫ����ߣ�Ҫ����															//��3
	{
		value4 =Straight_Speed+level2_Differential_speed;
		value2 = Straight_Speed+200;

		station = 1;
	}
	else if( (left_3() == 0)) //ƫ����ߣ�Ҫ����1241										//��3
	{
		value2 =Straight_Speed+level2_Differential_speed;
		value4 = Straight_Speed+200;

		station = 1;
	}
	else if( (right_4() == 0)) //ƫ����ߣ�Ҫ����															//��4
	{
		value4 =Straight_Speed+level3_Differential_speed;
		value2 = Straight_Speed+level2_Differential_speed;
		station = 1;
	}
	else if( (left_4() == 0)) //ƫ����ߣ�Ҫ����												//��4
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
	//0�����⵽
	if(front_left()==0)																					//��ǰ
	{
			Last_ANGLE_AHR298=ANGLE_AHR298;
//		int delta_angle = ((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);

    // �����ֵ����90�ȣ��򴥷���ʾ��Ϣ
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
	else if(front_right()==0)																									//��ǰ
	{
			Last_ANGLE_AHR298=ANGLE_AHR298;
//			int delta_angle = ((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);//340-330=10//Ӧ����ת��90��������ѭ��
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
    // �����ֵ����90�ȣ��򴥷���ʾ��Ϣ
		//������ߣ���Ҫת90��
 //  Last_ANGLE_AHR298=ANGLE_AHR298;
//					if(270<Last_ANGLE_AHR298&&Last_ANGLE_AHR298<360){
//						Angle_AHR =(AHRSData_Packet.Heading*57.29578f);
//						Angle_AHR -=360;
//					}
			
//					printf("yylast=%f\r\n",Last_ANGLE_AHR298);
//					printf("yythis=%f\r\n",ANGLE_AHR298);
						//��ֵ���ﵽ90�ȣ�Ӧ������ȥ
						//�����нϴ����䡮350-10=340�����֣���Ȼû�дﵽ90�ȣ����ǲ�ֵ����90����Ҫ����Լ��
	}
	else{
		ll();
	}
}

/*
״̬�жϺ���
�����м���״̬�������ڱ��жϣ�ǰ�����Ƿ񱻼�⵽���������⵽�ˣ���turn��û��⵽��linefollow
�涨һ�£������ת�����
1					����ת��
2					�����඼��⵽��
3					ֻ�����⵽��//��ת��־λ
4					ֻ���Ҽ�⵽��//��ת��־λ
5					ֱ��
6					8�Ҷ�·��⵽����ʼת
7					ǰ��ʶ��
8					ǰ��ʶ��
9					ʮ��ʶ��
����


*/
int Test_Attitude_judgment(void)
{
	int judgment_mission=0;
	if((front_left()==0||front_right()==0))//&&front_middle()!=0)//z���ң���û��ֱ��
	{
		if(front_left()==0&&front_right()==0)																//����
		{
			Turn_left_flag=1;
			Turn_right_flag=1;
			judgment_mission=2;
			printf("double\r\n");
		}
		else if(front_left()==0&&front_right())															//��ת------
		{
			Turn_left_flag=1;
			judgment_mission=3;
			printf("left\r\n");
		}	
		else if(front_left()&&front_right()==0)															//��ת
			{
			Turn_right_flag=1;
			judgment_mission=4;
			printf("right\r\n");
		}
		
	}
	else if(grayscale_sensor_judging()==1||grayscale_sensor_judging()==2)//Ѳ��							||grayscale_sensor_judging()==3)//||grayscale_sensor_judging()==4) 	//!(left_1()&&left_2()&&left_3()&&left_4()&&right_1()&&right_2()&&right_3()&&right_4()))//�����⵽�ˣ���ô������һ����0����ȫ�ң�����һ����
	{
		judgment_mission=5;
	}
	else if(grayscale_sensor_judging()>=3)																//��·�Ҷ�
	{
			judgment_mission=6;
	}
//	else if(front_left()==0&&front_middle()==0)														//ǰ���ж�
//	{
//		Turn_straight_flag=1;
//		Turn_left_flag=1;
//		printf("str_left");
//		judgment_mission=7;
//	}
//	else if(front_right()==0&&front_middle()==0)													//ǰ���ж�
//	{
//		Turn_straight_flag=1;
//		Turn_right_flag=1;
//		printf("str_right");
//		judgment_mission=8;
//	}
//	else if(front_right()==0&&front_middle()==0&&front_left()==0)					//ʮ���ж�
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
������Ҫһ�������ж�����������ת������ת��Ҳ���������Ǹ���״̬�жϣ�����������ڼ���־λ


*/
void Patrol_execution(int mission)
{
	//int 
	//grayscale_sensor_judging()
	
	
	switch(mission)
	{
		case 2://��д����ת 
		{
	  		if(grayscale_sensor_judging()>=4)
	  		{
         Left_Handed_Rotation();
				}
		    	break;
		}			
		case 3://��ת
		{
			if(grayscale_sensor_judging()>=4)
			{
				Left_Handed_Rotation();
			}				
			break;
		}		
		case 4://��ת
		{
			if(grayscale_sensor_judging()>=4)
			{
				Right_Handed_Rotation();
			}
			break;
		}		
		case 5://Ѳ��
		{
			ll();
			break;
		}		
		case 6://��·�ң�
		{
			if(Turn_left_flag&&Turn_right_flag)//��·�����ڻ�������
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
		case 7://ǰ��
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
		case 8://ǰ��
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
		case 9://ʮ��
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
		case 0://ֹͣ
		{
				ll();
			break;
		}		

	}
}
//����������ڼ���ļ����Ҷȱ�������grayscale_sensor_judging()�ķ���ֵ�Ǹ���
/*ԭ����2.2д��һЩʺɽ�����������жϻҶȸ���
		


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
		//left_1()+left_2()+left_3()+left_4()+right_4()+right_3()+right_2()+right_1();//���ȶ������ҶȽ���һ���жϣ�������һ�����лҶȵ�ʶ�������ûɨ��Ӧ����0������Ѳ�����Ӧ����6������·��Ӧ����4��3
		return fit;
}

void Left_Handed_Rotation(void)
{
								Last_ANGLE_AHR298=ANGLE_AHR298;
									printf("ok1\r\n");
		printf("delta_angle=%d\r\n",delta_angle);
//             		int delta_angle = (int)(fmod((ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0), 360.0));//((int)(ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0) % 360);
                // �����ֵ����90�ȣ��򴥷���ʾ��Ϣ
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
	//	    	      		int delta_angle = (int)(fmod((ANGLE_AHR298 - Last_ANGLE_AHR298 + 360.0), 360.0));//340-330=10//Ӧ����ת��90��������ѭ��
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

/*��ǰд��һЩʺɽ
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


if(fit==0||fit==8)//ûѲ����
	{
		turn_flag=10;
		//stop
	//	line_walking(turn_flag);
	}
	else if(fit==2||fit==1)//��ֱ��Ѳ��ģʽ
	{
		if(result[3]||result[4])//�������Ѱ�����ˣ�������Ӧ�ö���0
		{
			//ֱ��
			turn_flag=8;//��ʾֱ��
		}
		else
		{
		   for(i=0;i<8;i++)//����8���ҵ���·�ҵ����ߣ�����·���ݴ���
		   {
		   	if(result[i])
		   	{
		   			turn_flag=i;//ת��ȼ�
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
			//���ת
		}else if(result[5]&&result[6]&&result[7])
		{
			turn_flag=9;
			//�Ҵ�ת
		}
		else
		{
			if(result[3]||result[4])//�������Ѱ�����ˣ�������Ӧ�ö���0
		{
			//ֱ��
			turn_flag=8;//��ʾֱ��
		}
		else
		{
		   for(i=0;i<8;i++)//����8���ҵ���·�ҵ����ߣ�����·���ݴ���
		   {
		   	if(result[i])
		   	{
		   			turn_flag=i;//ת��ȼ�
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

/*��������Ҫ�õ���һЩʺɽ
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

