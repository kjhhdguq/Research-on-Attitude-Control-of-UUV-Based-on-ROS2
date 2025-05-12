#include "motor.h"

/* 电机占空比为5%-7.5%-10% */
/* 对应定时器CCR值为2000-3000-4000 */
/* 7.5%为中位停转 */


/* 电机电调初始化函数 */
void Motor_Init(void)
{
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,3000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,3000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,3000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,3000);
	
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,3000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,3000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,3000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,3000);
	HAL_Delay(2000);
	
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,270);
	
}


/* 电机设置速度函数 */
/* 范围[-90,90] */
void Motor_Set_Speed(uint8_t id, int16_t speed)
{
	switch(id)
	{
		case 1:
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,speed+3000);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,speed+3000);
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,speed+3000);
			break;
		case 4:
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,speed+3000);
			break;
		case 5:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,speed+3000);
			break;
		case 6:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,speed+3000);
			break;
		case 7:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,speed+3000);
			break;
		case 8:
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,speed+3000);
			break;
		default:
			break;
	}
}
