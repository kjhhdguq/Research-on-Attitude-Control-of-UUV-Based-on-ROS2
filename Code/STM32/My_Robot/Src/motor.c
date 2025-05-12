#include "motor.h"

/* ���ռ�ձ�Ϊ5%-7.5%-10% */
/* ��Ӧ��ʱ��CCRֵΪ2000-3000-4000 */
/* 7.5%Ϊ��λͣת */


/* ��������ʼ������ */
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


/* ��������ٶȺ��� */
/* ��Χ[-90,90] */
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
