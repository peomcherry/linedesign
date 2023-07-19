
#include "chassis_task.h"
#include "bsp_can.h"
#include "math.h"
#include "bsp_sbus.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "pid.h"
#include "led_task.h"
#define CHASSIS_CAN hcan1

extern int value1;
extern int value2;
extern int value3;
extern int value4;
extern speed_value base_speed;
static void chassis_init(chassis_move_t *Chassis_Move_Init);

static void chassis_set_mode(chassis_move_t *Chassis_Move_Mode);
//static void chassis_mode_change_control_transit(chassis_move_t *Chassis_Move_Transit);
static void chassis_feedback_update(chassis_move_t *Chassis_Move_Update);
static void chassis_set_contorl(chassis_move_t *Chassis_Move_Control);
static void  chassis_set_kinematics(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set,chassis_move_t *Chassis_Move_Kinematics);
static void chassis_control_loop(chassis_move_t *Chassis_Move_Control_Loop);

static void chassis_info_print(chassis_move_t *Chassis_Info);
void straight_line();
void turn();
void go_a_little();
//底盘运动数据
chassis_move_t Chassis_Move;
motor_measure_t motor[8];
extern motor_measure_t motor_can1[];
speed_wheel c610[8] = {0};
unsigned char station=1;
float p=0;

float d=0;
int flag=1;
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const * argument)//task
{
	int i=0;


	for(i=0;i<4;i++)
	{
		PID_struct_init(&c610[i].pid_weizhi,POSITION_PID,10000, 2000,15,0,0);//25,0.5,0.01		
	}

	for(i=0;i<4;i++)
	{
		PID_struct_init(&c610[i].pid_shudu,POSITION_PID,10000, 2000,5,0.06,0.1);
	}//5,0.06,0.1
	
    while(1)
    {

			switch(station)
			{
				case 1: 
					straight_line();
					break;
				case 2: 
				{
					turn();
//					dumiao();
				}
					break;
				case 3:
					go_a_little();
				default:
					break;
			}


        osDelay(5);

    }
}
void straight_line()
{
	
		  pid_calc(&c610[0].pid_shudu,motor_can1[0].speed_rpm,value1);
			pid_calc(&c610[1].pid_shudu,motor_can1[1].speed_rpm,-value2);
			pid_calc(&c610[2].pid_shudu,motor_can1[2].speed_rpm,value3);
			pid_calc(&c610[3].pid_shudu,motor_can1[3].speed_rpm,value4);
    //发送控制电流
	set_motor(&CHASSIS_CAN,
          //驱动轮
          c610[0].pid_shudu.pos_out,
					c610[1].pid_shudu.pos_out, 
					c610[2].pid_shudu.pos_out,
					c610[3].pid_shudu.pos_out,
          
          0,
          0,
          0,
          0
         );	
}

void turn()
{
//		  pid_calc(&c610[0].pid_shudu,motor_can1[0].speed_rpm,2000);
//			pid_calc(&c610[1].pid_shudu,motor_can1[1].speed_rpm,2000);
//			pid_calc(&c610[2].pid_shudu,motor_can1[2].speed_rpm,2000);
//			pid_calc(&c610[3].pid_shudu,motor_can1[3].speed_rpm,2000);
			
	int i=0;
  for ( i = 0; i < 4; i++)
    {
		  pid_calc(&c610[i].pid_shudu,motor_can1[i].speed_rpm,2000);
			pid_calc(&c610[i].pid_shudu,motor_can1[i].speed_rpm,c610[i].pid_weizhi.pos_out);		
    }
    //发送控制电流
	set_motor(&CHASSIS_CAN,
          //驱动轮
          c610[0].pid_shudu.pos_out,
					c610[1].pid_shudu.pos_out, 
					c610[2].pid_shudu.pos_out,
					c610[3].pid_shudu.pos_out,
          
          0,
          0,
          0,
          0
         );
}

	
void go_a_little()
{
	
	int i;
		for(i=0;i<4;i++)
	{
	 motor_can1[i].round_cnt=0;	
	}
	pid_calc(&c610[0].pid_weizhi, 	 motor_can1[0].round_cnt, 	 -100);
	pid_calc(&c610[0].pid_shudu,     motor_can1[0].speed_rpm,     c610[0].pid_weizhi.pos_out);
	//二号（右前）                   
	pid_calc(&c610[1].pid_weizhi,		 motor_can1[1].round_cnt, 		100);
	pid_calc(&c610[1].pid_shudu,     motor_can1[1].speed_rpm,      c610[1].pid_weizhi.pos_out);
	//三号（左后）
	pid_calc(&c610[2].pid_weizhi,		 motor_can1[2].round_cnt, 	 -100);
	pid_calc(&c610[2].pid_shudu,     motor_can1[2].speed_rpm,     c610[2].pid_weizhi.pos_out);
	//四号（右后）
	pid_calc(&c610[3].pid_weizhi,	 	 motor_can1[3].round_cnt, 	 100);
	pid_calc(&c610[3].pid_shudu,     motor_can1[3].speed_rpm,     c610[3].pid_weizhi.pos_out);
			for(i=0;i<4;i++)
	{
	 motor_can1[i].round_cnt=0;	
	}
	
		set_motor(&CHASSIS_CAN,
          //驱动轮
          c610[0].pid_shudu.pos_out,
					c610[1].pid_shudu.pos_out, 
					c610[2].pid_shudu.pos_out,
					c610[3].pid_shudu.pos_out,
          
          0,
          0,
          0,
          0
         );
}


/*屎山存放处

屎山存放处






	//速度环初始化
	PID_struct_init(&c610[1].pid_shudu,POSITION_PID,10000, 2000,5,0.06,0.1);
	PID_struct_init(&c610[2].pid_shudu,POSITION_PID,10000, 2000,5,0.06,0.1);
	PID_struct_init(&c610[3].pid_shudu,POSITION_PID,10000, 2000,5,0.06,0.1);


					主循环里的垃圾

//	for(i=0;i<4;i++)
//	{
//	  PID_struct_init(&Chassis_Move.Wheel_Dir[i].pid_pos,POSITION_PID, 10000, 2000,
//                        400.0f, 0.00f, 10.0);
//		PID_struct_init(&Chassis_Move.Wheel_Speed[i].pid_speed,POSITION_PID, 10000, 2000,
//                        5.0f, 0.04f, 0.0f);
//	}


//			pid_calc(&c610[0].pid_weizhi, 	 motor_can1[0].round_cnt,200);	
//		  pid_calc(&c610[0].pid_shudu,motor_can1[0].speed_rpm,c610[0].pid_weizhi.pos_out);
//			
//			pid_calc(&c610[1].pid_weizhi, 	 motor_can1[1].round_cnt,200);	
//		  pid_calc(&c610[1].pid_shudu,motor_can1[1].speed_rpm,c610[1].pid_weizhi.pos_out);	
//			
//			pid_calc(&c610[2].pid_weizhi, 	 motor_can1[2].round_cnt,200);	
//		  pid_calc(&c610[2].pid_shudu,motor_can1[2].speed_rpm,c610[2].pid_weizhi.pos_out);
//			
//			pid_calc(&c610[3].pid_weizhi, 	 motor_can1[3].round_cnt,200);	
//		  pid_calc(&c610[3].pid_shudu,motor_can1[3].speed_rpm,c610[3].pid_weizhi.pos_out);		
//			printf("%d\r\n",motor_can1[0].round_cnt);
//    //发送控制电流
//	set_motor(&CHASSIS_CAN,
//          //驱动轮
//          c610[0].pid_shudu.pos_out,
//					c610[1].pid_shudu.pos_out, 
//					c610[2].pid_shudu.pos_out,
//					c610[3].pid_shudu.pos_out,
//          
//          0,
//          0,
//          0,
//          0
//         );	
//			
			
//	PID_struct_init(&c610[0].pid_weizhi,POSITION_PID,10000, 2000,kp,ki,kd);//25,0.5,0.01
//	PID_struct_init(&c610[1].pid_weizhi,POSITION_PID,10000, 2000,kp,ki,kd);
//	PID_struct_init(&c610[2].pid_weizhi,POSITION_PID,10000, 2000,kp,ki,kd);
//	PID_struct_init(&c610[3].pid_weizhi,POSITION_PID,10000, 2000,kp,ki,kd);
//	


























*/















































//下面的代码基本不用看了












/*
1.设置底盘模式:       chassis_set_mode(&chassis_move);

2.底盘模式改变过程:   chassis_mode_change_control_transit(&chassis_move);
		更新参数
3.底盘数据更新:       chassis_feedback_update(&chassis_move);
		获取当前电机,角度,坐标信息
4.设置底盘控制量:     chassis_set_contorl(&chassis_move);

6.底盘运动学模式选择:	chassis_set_kinematics(&chassis_move);

7.底盘PID计算:       	chassis_control_loop(&chassis_move);

8.can发送电机电流
delay
*/

#define NO_INFO           0  //无信息
#define INFO_MOTOR        1 //驱动信息
#define INFO_WHEEL_DIR    2 //舵轮角度信息
#define INFO_WHEEL_SPEED  3 //舵轮速度信息
#define INFO_RC           4 //遥控器的信息
#define INFO_V_SET        5 //V_X V_Y V_Z的设置的信息
#define VOFA_WHEEL_DIR    6 //实际角度和计算出来的期望角度
#define VOFA_WHEEL_SPEED  7 //实际速度和计算出来的期望速度

#define INFO_PRINT NO_INFO //无信息


//#define INFO_PRINT  VOFA_WHEEL_DIR  //修改这里的宏定义即可修改打印的信息

static void chassis_info_print(chassis_move_t *Chassis_Info)
{

#if INFO_PRINT == INFO_MOTOR
    printf("speed_rpm:%6d,angle:%6d,last_angle:%6d,total_angle:%6d 角度:%6d\n",
           (int)Chassis_Info->Chassis_Motor_Measure[i]->speed_rpm,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->angle,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->last_angle,
           (int)Chassis_Info->Chassis_Motor_Measure[i]->total_angle,
           //Motor_Angle_Cal(&motor_can1[0],294912)
           (int)get_total_angle(&motor_can1[i])
          );
#elif  INFO_PRINT == INFO_WHEEL_DIR
    printf("speed_rpm:%6d  angle:%6.2f angle_set:%6.2f  give_current=%6d \r\n",
           (int)Chassis_Info->Wheel_Dir[i].speed,
           Chassis_Info->Wheel_Dir[i].angle,
           Chassis_Info->Wheel_Dir[i].angle_set,
           Chassis_Info->Wheel_Dir[i].give_current);
#elif INFO_PRINT == INFO_WHEEL_SPEED
    printf("speed_rpm:%6d speed_set:%6d give_current=%6d\r\n",
           (int)Chassis_Info->Wheel_Speed[i].speed,
           (int)Chassis_Info->Wheel_Speed[i].speed_set,
           Chassis_Info->Wheel_Speed[i].give_current);
#elif INFO_PRINT == INFO_RC
    printf("ch1:%4d ch2:%4d ch3:%4d ch4:%4d ch5:%4d ch6:%4d \r\n",
           Chassis_Info->Chassis_RC->ch1,
           Chassis_Info->Chassis_RC->ch2,
           Chassis_Info->Chassis_RC->ch3,
           Chassis_Info->Chassis_RC->ch4,
           Chassis_Info->Chassis_RC->ch5,
           Chassis_Info->Chassis_RC->ch6);
#elif INFO_PRINT == INFO_V_SET
    printf("Vx_Set:%6d Vy_Set:%6d Vw_Set:%6d  \r\n",
           (int)Chassis_Info->Vx_Set,
           (int)Chassis_Info->Vy_Set,
           (int)Chassis_Info->Vw_Set);
#elif INFO_PRINT == VOFA_WHEEL_DIR
    printf("%6f,%6f\r\n",
           Chassis_Info->Wheel_Dir[i].angle,
           Chassis_Info->Wheel_Dir[i].angle_set);

#elif INFO_PRINT == VOFA_WHEEL_SPEED
    printf("%6f,%6f\r\n",
           Chassis_Info->Wheel_Speed[i].speed,
           Chassis_Info->Wheel_Speed[i].speed_set);

#endif

}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     Chassis_Move_Init:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *Chassis_Move_Init)
{
    if (Chassis_Move_Init == NULL)
    {
        return;
    }
    //底盘速度环pid值
    //const static fp32 motor_speed_pid[3] = {WHEEL_SPEED_PID_SPEED_KP, WHEEL_SPEED_PID_SPEED_KI, WHEEL_SPEED_PID_SPEED_KD};
    //底盘开机状态为原始
    Chassis_Move_Init->Chassis_Mode = CHASSIS_ZERO_FORCE;
    //获取遥控器指针
    Chassis_Move_Init->Chassis_RC = get_remote_control_point();
    //获取定位指针

    //获取底盘电机指针
    for(uint8_t i=0; i<8; i++)
    {
        Chassis_Move_Init->Chassis_Motor_Measure[i] = get_chassis_motor_measure_point(i);
    }

    for (uint8_t i = 0; i < 4; i++)
    {
		 
        PID_struct_init(&Chassis_Move_Init->Wheel_Speed[i].pid_speed,POSITION_PID, 10000, 2000,
                        5.0f, 0.04f, 0.0f);
        // 5 0.04 0
        PID_struct_init(&Chassis_Move_Init->Wheel_Dir[i].pid_pos,POSITION_PID, 10000, 2000,
                        400.0f, 0.00f, 10.0);
        PID_struct_init(&Chassis_Move_Init->Wheel_Dir[i].pid_speed,POSITION_PID, 8000, 2000,
                        4.5f, 0.08f, 1.5f);
    }
    //初始化底盘角度PID
    PID_struct_init(&Chassis_Move_Init->chassis_angle_pid, POSITION_PID, 10000, 2000,
                    1.50f, 0.00f, 0.01);

    //最大 最小速度
    Chassis_Move_Init->Vx_Max_Speed = 10;
    Chassis_Move_Init->Vx_Min_Speed = -10;
    Chassis_Move_Init->Vy_Max_Speed = 10;
    Chassis_Move_Init->Vy_Min_Speed = -10;

    //更新一下数据
    chassis_feedback_update(Chassis_Move_Init);


}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     Chassis_Move_Mode:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *Chassis_Move_Mode)
{
    if (Chassis_Move_Mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(Chassis_Move_Mode);
}

/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
//以下函数待实现或不再使用
//static void chassis_mode_change_control_transit(chassis_move_t *Chassis_Move_Transit)
//{
//    if (Chassis_Move_Transit == NULL)
//    {
//        return;
//    }
//}


/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，坐标,机器人速度
  * @param[out]     Chassis_Move_Update:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *Chassis_Move_Update)
{
    if (Chassis_Move_Update == NULL)
    {
        return;
    }
    //底盘电机参数跟新
    for(uint8_t i=0; i<4; i++)
    {
        //驱动轮速度更新
        Chassis_Move_Update->Wheel_Speed[i].speed = Chassis_Move_Update->Chassis_Motor_Measure[i]->speed_rpm;
        //转型轮速度角度更新
        Chassis_Move_Update->Wheel_Dir[i].speed = Chassis_Move_Update->Chassis_Motor_Measure[i+4]->speed_rpm;
        Chassis_Move_Update->Wheel_Dir[i].angle = Chassis_Move_Update->Chassis_Motor_Measure[i+4]->total_angle*360.0/1091174;//294876
    }
    //底盘位置更新
    Chassis_Move_Update->Postion_X = 0;
    Chassis_Move_Update->Postion_Y = 0;

    //底盘角度更新
    Chassis_Move_Update->Chassis_Yaw=0;


    Chassis_Move_Update->Postion_X_Set = 0;
    Chassis_Move_Update->Postion_Y_Set=0;
    Chassis_Move_Update->Chassis_Yaw_Set=0;



}


#define sin60 0.866f  //sin60
#define cos60 0.5f    //cos60
#define PI 3.1415926f
#define L 0.11f      //长度
#define sin45 0.707107f  //sin45
#define cos45 0.707107f  //cos45
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     Chassis_Move_Control:"chassis_move_t"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *Chassis_Move_Control)
{
    if (Chassis_Move_Control == NULL)
    {
        return;
    }
    fp32 Vx_Set = 0.0f, Vy_Set = 0.0f, Vw_Set=0.0f;
    //获取三个控制设置值
    chassis_behaviour_control_set(&Vx_Set, &Vy_Set, &Vw_Set, Chassis_Move_Control);  //求得Vx,Vy,Angle

    Chassis_Move_Control->Vx_Set =Vx_Set;
    Chassis_Move_Control->Vy_Set =Vy_Set;
    Chassis_Move_Control->Vw_Set =Vw_Set;


}




static void chassis_kinematics_omni3_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    /*运动学逆解*/
    Wheel_Speed[0] = -Vx_set + L * Vw_Set;
    Wheel_Speed[1] = cos60 * Vx_set - sin60 * Vy_set + L * Vw_Set;
    Wheel_Speed[2] = cos60 * Vx_set + sin60 * Vy_set + L * Vw_Set;
    Wheel_Speed[3] = 0;
}
static void chassis_kinematics_omni4_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    /*运动学逆解*/
    Wheel_Speed[0] = -cos45 * Vx_set - sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[1] = -cos45 * Vx_set + sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[2] =  cos45 * Vx_set - sin45 * Vy_set + L * Vw_Set;
    Wheel_Speed[3] =  cos45 * Vx_set + sin45 * Vy_set + L * Vw_Set;
}
#define WHEEL_PERIMETER      (0.2f)  //车轮周长
#define CHASSIS_DECELE_RATIO (19)   //底盘减速比
#define Radius  (40)

static void chassis_kinematics_wheel3_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4], fp32 Wheel_Angle[4])
{

}
static void chassis_kinematics_wheel4_speed(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set, fp32 Wheel_Speed[4], fp32 Wheel_Angle[4])
{
    float wheel_rpm_ratio; //车轮转速比
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_rpm_ratio=1;
    //                 60  / (0.2*3.14)  *19*1000z                         pik
    //速度计算
    Wheel_Speed[0] = sqrt(pow(Vy_set + Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set - Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[1] = sqrt(pow(Vy_set - Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set - Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[2] = sqrt(pow(Vy_set - Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set + Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    Wheel_Speed[3] = sqrt(pow(Vy_set + Vw_Set * Radius * 0.707107f, 2)
                          + pow(Vx_set + Vw_Set * Radius * 0.707107f, 2)
                         ) * wheel_rpm_ratio;
    //角度计算

    Wheel_Angle[0] = atan2((Vx_set - Vw_Set * Radius * 0.707107f),(Vy_set + Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[1] = atan2((Vx_set - Vw_Set * Radius * 0.707107f),(Vy_set - Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[2] = atan2((Vx_set + Vw_Set * Radius * 0.707107f),(Vy_set - Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
    Wheel_Angle[3] = atan2((Vx_set + Vw_Set * Radius * 0.707107f),(Vy_set + Vw_Set * Radius * 0.707107f))
                     * 180.0f / PI;
										 
		for(int i = 0; i<4 ;i++)    //模型结算后，有些转向看着不舒服，自行加if else判断修改
		{
			if(Wheel_Angle[i] < -90 && Wheel_Angle[i] >-181)
			{
				Wheel_Angle[i] += 180;
				Wheel_Speed[i] = -Wheel_Speed[i];
			}
			else if(Wheel_Angle[i] > 90 && Wheel_Angle[i] <=181)
			{
				Wheel_Angle[i] -= 180;
				Wheel_Speed[i] = -Wheel_Speed[i];
			}
		}
}


static void  chassis_set_kinematics(const fp32 Vx_set, const fp32 Vy_set, const fp32 Vw_Set,chassis_move_t *Chassis_Move_Kinematics)
{
    if (Chassis_Move_Kinematics == NULL)
    {
        return;
    }
    float Wheel_Speed[4]= {0};
    float Wheel_Angle[4]= {0};

    if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Omni3_Car)
    {         
        chassis_kinematics_omni3_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Omni4_Car)
    {
        chassis_kinematics_omni4_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Wheel3_Car)
    {
        chassis_kinematics_wheel3_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed,Wheel_Angle);
    }
    else if (Chassis_Move_Kinematics->Chassis_Kinematics_Mode == Wheel4_Car)
    {
        chassis_kinematics_wheel4_speed(Vx_set,Vy_set,Vw_Set,Wheel_Speed,Wheel_Angle);
    }
    for(uint8_t i=0; i<4; i++)
    {
        Chassis_Move_Kinematics->Wheel_Speed[i].speed_set = Wheel_Speed[i];
        Chassis_Move_Kinematics->Wheel_Dir[i].angle_set = Wheel_Angle[i];
    }

}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *Chassis_Move_Control_Loop)
{
    //运动分解
    chassis_set_kinematics(Chassis_Move_Control_Loop->Vx_Set,
                           Chassis_Move_Control_Loop->Vy_Set,
                           Chassis_Move_Control_Loop->Vw_Set,
                           Chassis_Move_Control_Loop);

    uint8_t i = 0;
    //计算pid
    //驱动轮
    for ( i = 0; i < 4; i++)
    {
        pid_calc(&Chassis_Move_Control_Loop->Wheel_Speed[i].pid_speed,
                 Chassis_Move_Control_Loop->Wheel_Speed[i].speed,
                 Chassis_Move_Control_Loop->Wheel_Speed[i].speed_set);
    }
    //转向轮
    for ( i = 0; i < 4; i++)
    {

        pid_calc(&Chassis_Move_Control_Loop->Wheel_Dir[i].pid_pos,
                 Chassis_Move_Control_Loop->Wheel_Dir[i].angle,
                 Chassis_Move_Control_Loop->Wheel_Dir[i].angle_set);

        pid_calc(&Chassis_Move_Control_Loop->Wheel_Dir[i].pid_speed,
                 Chassis_Move_Control_Loop->Wheel_Dir[i].speed,
                 Chassis_Move_Control_Loop->Wheel_Dir[i].pid_pos.pos_out);
    }

    //赋值电流值
    for ( i = 0; i < 4; i++)
    {

        Chassis_Move_Control_Loop->Wheel_Speed[i].give_current =
            (int16_t)Chassis_Move_Control_Loop->Wheel_Speed[i].pid_speed.pos_out;
        Chassis_Move_Control_Loop->Wheel_Dir[i].give_current =
            (int16_t)Chassis_Move_Control_Loop->Wheel_Dir[i].pid_speed.pos_out;
    }

    //发送控制电流
    if(Chassis_Move_Control_Loop->Chassis_Mode == CHASSIS_ZERO_FORCE)
    {
        set_motor(&CHASSIS_CAN,0,0,0,0,0,0,0,0);
    }
    else {
        set_motor(&CHASSIS_CAN,
                  //驱动轮
                  Chassis_Move_Control_Loop->Wheel_Speed[0].give_current,
                  Chassis_Move_Control_Loop->Wheel_Speed[1].give_current,
                  Chassis_Move_Control_Loop->Wheel_Speed[2].give_current,
                  Chassis_Move_Control_Loop->Wheel_Speed[3].give_current,
                  //转向轮
                  Chassis_Move_Control_Loop->Wheel_Dir[0].give_current,
                  Chassis_Move_Control_Loop->Wheel_Dir[1].give_current,
                  Chassis_Move_Control_Loop->Wheel_Dir[2].give_current,
                  Chassis_Move_Control_Loop->Wheel_Dir[3].give_current
                 );
    }
}























