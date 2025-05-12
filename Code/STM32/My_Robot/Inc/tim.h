/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN Private defines */
#define COUNT_BEAT_ID        1

typedef volatile struct
{
	unsigned char Buzzer_Num;
	unsigned char Buzzer_Flag;
	uint32_t Buzzer_Inter_On;
	uint32_t Buzzer_Inter_Off;
	unsigned char Buzzer_times;
}Buzzer;


extern Buzzer Start_Robot;
extern Buzzer Open_Con;
extern Buzzer Closed_Con;
extern Buzzer *(pBuzzer[]);

extern uint32_t Buzzer_cnt;
extern unsigned char Buzzer_time;

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

extern Buzzer Start_Robot;

u16 Timer_Get_Count(u8 id);
void Timer_Set_Count(u8 id, u16 value);
void Timer_Count_Auto_Reduce(u8 id);
void BUZZER_ON(void);
void BUZZER_OFF(void);
void Buzzer_Reset(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

