#include "led_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_can.h"
#include "usart.h"
#include "stdio.h"
#define straight_speed 3500
#define small_turn 700
#define big_turn  1000
extern motor_measure_t motor_can1[];
extern speed_wheel c610[8];
extern 	int flag;
extern int aa;
extern unsigned char station;

speed_value base_speed={1000,1000,1000,1000};

int value1=3670;
int value2=3500;
int value3=3670;
int value4=3500;
unsigned char start_flag=1;
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
void line_walking();
void turn_judging(void);
/**
  * @brief          led RGBÈÎÎñ
* @param[in]      pvParameters: NULL F1:L F0:C  C2:Right
  * @retval         none
  */
unsigned int t;


void led_task(void const * argument)
{

	while(1){

		line_walking();
		osDelay(5);
	
	}
}



void line_walking()
{
	 if(left_4() == 0)
	{
//		if(left_4()+left_3()==0)
//		{
//			for(int i=0;i<100000;i++)
//			{
//				
//	      	value1 = value2 = -1000;
//	      	value3 = value4 =  4000;
//			}
//		}
		value1 = value2 =  -2000;
		value3 = value4 =  3000;
		station =1;
	}else if(right_4() == 0 )
	{
//		if(right_4()+right_3()==0)
//		{
			for(int i=0;i<100000;i++)
			{
	     	value1 = value2 =  5000;
	     	value3 = value4 =  -1000;
			}
//		}
	     	value1 = value2 =  5000;
	     	value3 = value4 =  -1000;
		station =1;
	}else if(right_4() == 0 && left_4() == 0)
	{
		value1 = value2 =  5000;
		value3 = value4 =  -1000;
		station =1;
	}
	else if(left_1() == 0 && right_1() == 0)
		{
			value1 = value2 = \
			value3 = value4 =4000;
			station = 1;
		}else if(right_1() == 0 && right_2() == 0) //Æ«ÏòÓÒ±ß£¬ÒªÍù×ó
		{
			value1 = value2 =4500;
			station = 1;
		}else if(left_1() == 0 && left_2() == 0) //Æ«Ïò×ó±ß£¬ÒªÍùÓÒ
		{
			value3 = value4 =4500;
			station = 1;
		}
}
//	else
//	{
//		value1 = value2 = \
//		value3 = value4 =0;	
//		
////		station = 2;
//	}

