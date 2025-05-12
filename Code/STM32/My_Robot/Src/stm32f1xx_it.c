/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "JY901.h"
#include "protocol.h"
#include "tim.h"
#include "spi.h"
#include "pid.h"
#include "motion_control.h"
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
//extern uint8_t TxBuffer1[256];
//extern uint8_t TxBuffer3[256];
//extern unsigned char TxCounter1=0;
//extern unsigned char TxCounter3=0;
//extern unsigned char count1=0;
//extern unsigned char count3=0;
static uint8_t cnt_5ms = 0;
static uint8_t cnt_10ms = 0;
static uint8_t cnt_motor = 0;
uint32_t sys_tick = 0;

int8_t updown_num = 0;
int8_t rightleft_num = 0;
int8_t goback_num = 0;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	if (HAL_TIM_Base_GetState(&htim1) != RESET)
	{
		cnt_5ms++;
		cnt_10ms++;
		
		// PID控制
		if(openclose_rev == 1)					// 闭环控制
		{
			if(cnt_5ms == pid_interval_cnt)
			{
				cnt_5ms = 0;
				Robot_PidControl(pid_interval);
				MotorControl();
//				PIDGyroOut_Send_Data();
				Led2_Flag = 1;
			}
		}
		
		else
		{
			Open_Motion_Control_It();			// 开环控制（缓慢增减速1ms）
			pidRest(pPidObject, 6);
			Led2_Flag = 0;
		}
		
		// 上报IMU数据
		if(cnt_10ms == 10)
		{
			cnt_10ms = 0;
			if(Cali_Flag == 0)
			{
//				Gyro_Send_Data();   //上报角速度数据
			
//				Angle_Send_Data();  //上报角度数据
			}
		}
		
		// 处理LED1
		if(Led1_Flag == 1)
			LED1_ON();
		else
			LED1_OFF();
		// 处理LED2
		if(Led2_Flag == 1)
			LED2_ON();
		else
			LED2_OFF();
		// 处理LEDF
		if(Ledf_Flag == 1)
			LEDF_ON();
		else
			LEDF_OFF();
		
		// 处理蜂鸣器
		Buzzer_Disp(pBuzzer);
		
		
    // ����жϱ�־λ
		__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
  }
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */
	
  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) != RESET)
	{
    HAL_UART_Transmit_IT(&huart1, &TxBuffer1[TxCounter1], sizeof(TxBuffer1[TxCounter1]));
		TxCounter1++;
    if (TxCounter1 == count1)
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    
    __HAL_UART_CLEAR_FLAG(&huart1, UART_IT_TXE);
  }
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	if(__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TXE) != RESET)
	{
    HAL_UART_Transmit_IT(&huart3, &TxBuffer3[TxCounter3], sizeof(TxBuffer3[TxCounter3]));
		TxCounter3++;
    if (TxCounter3 == count3) 
      __HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
		
		__HAL_UART_CLEAR_FLAG(&huart3, UART_IT_TXE);
		
  }
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
	if(__HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TXE) != RESET)
	{
    HAL_UART_Transmit_IT(&huart4, &TxBuffer4[TxCounter4], sizeof(TxBuffer4[TxCounter4]));
		TxCounter4++;
    if (TxCounter4 == count4)
      __HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
		
		__HAL_UART_CLEAR_FLAG(&huart4, UART_IT_TXE);
  }
  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Upper_Data_Receive(Rx1_Temp);
		HAL_UART_Receive_IT(&huart1, &Rx1_Temp, 1);
	}
	
	if(huart->Instance == USART3)
	{
		CopeSerial3Data(Rx3_Temp);
		HAL_UART_Receive_IT(&huart3, &Rx3_Temp, 1);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI2)
	{
		LED1_ON();
		HAL_SPI_TransmitReceive_IT(&hspi2, &Txs_Temp, &Rxs_Temp, 1);
	}
}


void Open_Motion_Control_It(void)
{
	if(New_Speed_Flag == 1)
	{
		if(goback_num != pitch_rev)
		{
			if(goback_num < pitch_rev)
			{
				goback_num++;
			}
			else
			{
				goback_num--;
			}
			Robot_GoBack(goback_num);
		}
		if(rightleft_num != yaw_rev)
		{
			if(rightleft_num < yaw_rev)
			{
				rightleft_num++;
			}
			else
			{
				rightleft_num--;
			}
			Robot_RightLeft(rightleft_num);
		}
		if(updown_num != updown_rev)
		{
			if(updown_num < updown_rev)
			{
				updown_num++;
			}
			else
			{
				updown_num--;
			}
			Robot_UpDown(updown_num);
		}
		if(updown_num == updown_rev && rightleft_num == yaw_rev && goback_num == pitch_rev)
		{
			New_Speed_Flag = 0;
			cnt_motor = 0;
		}
	}
}


void Buzzer_Disp(Buzzer **Buzzer_Process)
{
	if(Buzzer_Process[0]->Buzzer_Flag == 1)
	{
		if(Buzzer_time%2 == 0)
		{
			BUZZER_ON();
			if(Buzzer_cnt == Buzzer_Process[0]->Buzzer_Inter_On)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		else
		{
			BUZZER_OFF();
			if(Buzzer_cnt == Buzzer_Process[0]->Buzzer_Inter_Off)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		Buzzer_cnt++;
		if(Buzzer_time == Buzzer_Process[0]->Buzzer_times)
		{
			BUZZER_OFF();
			Buzzer_Process[0]->Buzzer_Flag = 0;
			Buzzer_time = 0;
			Buzzer_cnt = 0;
		}
	}
	
	if(Buzzer_Process[1]->Buzzer_Flag == 1)
	{
		if(Buzzer_time%2 == 0)
		{
			BUZZER_ON();
			if(Buzzer_cnt == Buzzer_Process[1]->Buzzer_Inter_On)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		else
		{
			BUZZER_OFF();
			if(Buzzer_cnt == Buzzer_Process[1]->Buzzer_Inter_Off)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		Buzzer_cnt++;
		if(Buzzer_time == Buzzer_Process[1]->Buzzer_times)
		{
			BUZZER_OFF();
			Buzzer_Process[1]->Buzzer_Flag = 0;
			Buzzer_time = 0;
			Buzzer_cnt = 0;
		}
	}
	
	if(Buzzer_Process[2]->Buzzer_Flag == 1)
	{
		if(Buzzer_time%2 == 0)
		{
			BUZZER_ON();
			if(Buzzer_cnt == Buzzer_Process[2]->Buzzer_Inter_On)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		else
		{
			BUZZER_OFF();
			if(Buzzer_cnt == Buzzer_Process[2]->Buzzer_Inter_Off)
			{
				Buzzer_cnt = 0;
				Buzzer_time++;
			}
		}
		Buzzer_cnt++;
		if(Buzzer_time == Buzzer_Process[2]->Buzzer_times)
		{
			BUZZER_OFF();
			Buzzer_Process[2]->Buzzer_Flag = 0;
			Buzzer_time = 0;
			Buzzer_cnt = 0;
		}
	}
}

/* USER CODE END 1 */
