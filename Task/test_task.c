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
uint8_t USART_RX_BUF[USART_REC_LEN]={0,0,0,2,2,0,1,2,2,0,1,1,1,1};     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       //接收状态标记	  
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲


//_K210   K210数据相关定义
uint8_t USART_RX_BUF_K210[USART_REC_LEN_K210];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA_K210=0;       //接收状态标记	  

uint8_t aRxBuffer_K210[RXBUFFERSIZE_K210];//HAL库使用的串口接收缓冲
/**
  * @brief 	将取环送环装置设为初始化
  * @param  none
  * @retval none
  */

/**
  * @brief          取环 送环 推环
  * @brief          串口2：树莓派
  * @brief          串口6：K210
	* @brief          串口8：定位轮
  * @param[in]      none    
  * @retval         none
  */

void test_task(void const * argument)
{ 
//	int i=0;
	int previousData = 0;  // 上一次的数据
    int count = 0;  // 十次自检的计数器

    // 模拟数据接收，假设收到的数据存在变量receivedData中
    int receivedData = 0;  // 假设收到的数据为85

    // 判断receivedData是否大于80
    
	while(1)
	{
		if(USART_RX_STA&0x8000)
		{	
			
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			receivedData=USART_RX_BUF[0];
			if (receivedData > 80) 
			{
        // 判断是否需要自检十次
        if (count < 10)
				{
            // 判断receivedData与上一次数据是否相同
            if (previousData == USART_RX_BUF[0]) 
					{
                // 更新数据
                USART_RX_BUF[0] = receivedData;
                previousData = receivedData;
								OPMV_ANS= receivedData;
                count++;
                printf("Data updated. Count: %d\r\n", count);
           } 
					else 
					{
                // 不更新数据，保持上一次的数据
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
			
//			HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
//			while( HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX );
			USART_RX_STA=0;
		}
	//	AHRSData2PC();
		osDelay(10);
	}
	
}

void  rotatePoint(Point* point, double angle) {
    double radian = angle * M_PI / 180.0;  // 将角度转换为弧度

    // 计算旋转后的坐标点
    float newX = point->x * cos(radian) - point->y * sin(radian);
    float newY = point->x * sin(radian) + point->y * cos(radian);

    // 更新坐标点的值
    point->n_x= newX;
    point->n_y= newY;
	


}

/*5版本前test主函数

//			printf("speed1=%lf	speed2=%lf	now1=%f	now2=%f	\n"
//			,c610[1].pid_shudu,motor_can1[1].speed_rpm,
//			c610[1].pid_shudu,motor_can1[3].speed_rpm,
//			value2,
//			value4);
		// AHRSData2PC();
		
				int i = 0;
		//树莓派发送的数据
		 if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度

				if(USART_RX_BUF[0] != 'r' && USART_RX_BUF[0] != 'b')
			{
				for(i=0;i<len;i++)
				{
					printf("datai=%d\r\n",USART_RX_BUF[i]-48);
				}
			}
//			HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
//			while( HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX );
			USART_RX_STA=0;
		}
//		
//		//K210数据接收处理函数  改成自己使用的串口号
//		if(USART_RX_STA_K210&0x8000)
//		{					   
//			len=USART_RX_STA_K210&0x3fff;//得到此次接收到的数据长度
//		//	printf("\r\n您发送的消息为:\r\n");
//			HAL_UART_Transmit(&huart6,(uint8_t*)USART_RX_BUF_K210,len,1000);	//发送接收到的数据
//			while( HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX );
//			USART_RX_STA_K210=0;
//		}

//		printf("n16.val=%f\r\n",pos_y);
//		printf("n17.val=%f\r\n",pos_x);
//		printf("n18.val=%f\r\n",zangle);
//		printf("n19.val=%f\r\n",w_z);
//		















*/
