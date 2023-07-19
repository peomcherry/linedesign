#ifndef __LED_TASK_H
#define __LED_TASK_H

#include "main.h"
#include "struct_typedef.h"

typedef struct speed{
	uint32_t speed1;
	uint32_t speed2;
	uint32_t speed3;
	uint32_t speed4;
}speed_value;


/*
pc2-pc5  --led£º8 7 6 5 --L1(you4) M1 N1 O1
PF0 -> LED4 ->I2
PF1 -> LED3 -> I1
PF10-> LED2 ->Q1
PE4 -> led1 -> j2
*/
#define left_1()  (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0) == 1)
#define right_1() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5) == 1)

#define left_2() (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == 1)
#define right_2() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4) == 1)

#define left_3() (HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_10) == 1)
#define right_3() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == 1)

#define left_4() (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == 1)
#define right_4() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == 1)

#define middle_left() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5))
#define middle_right() (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5))


/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGBÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void led_task(void const * argument);

#endif


