/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol.h"
#include "JY901.h"
#include "motor.h"
#include "oled.h"
#include "motion_control.h"
#include "pid.h"
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char oled_show_str[] = {"温度"};
//uint8_t i;
//uint8_t j;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int Call_10ms = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	
	/* ��ʼ����ʱ���жϺ�PWM */
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
	
	LED1_OFF();
	LED2_OFF();
	LEDF_OFF();
	BUZZER_OFF();
	
	OLED_Init();
	OLED_CLS();
	OLED_ShowStr(45,2,"R O V",2);
	HAL_Delay(1000);
	
	// 惯导模块使用串口3（波特率9600）
	jy901_init();
	HAL_Delay(1000);
	OLED_CLS();
	OLED_ShowStr(24,2,"JY901-Init",2);
	
	Motor_Init();
	OLED_CLS();
	OLED_ShowStr(26,2,"Motor-Init",2);
	HAL_Delay(500);
	
	OLED_CLS();
	OLED_ShowStr(32,2,"PidParam",2);
	OLED_ShowStr(36,4,"Loading",2);
	AT24C256_ReadByte(0x0000, 38, Flash_Pid);		//读取AT24C256存储的pid参数
	pidParamLoad(pPidObject, Flash_Pid);				// pid参数初始化
//	PID_ParamInit();
//	PID_Desired_Init();													// pid初始期望
	pidRest(pPidObject, 6);
	HAL_Delay(500);
	
	OLED_CLS();
	OLED_ShowStr(28,2,"B E G I N",2);
	
	// 打开定时器
	// 在IMU初始化之后，否则也无法初始化
	HAL_TIM_Base_Start_IT(&htim1);
	
	// 开机提示音
	Buzzer_Reset();
	Start_Robot.Buzzer_Flag = 1;
	
	// 串口接收中断在IMU初始化之后开，否则无法进行IMU初始化
	// 最后再开串口接收中断
	HAL_UART_Receive_IT(&huart1, &Rx1_Temp, 1);
	HAL_UART_Receive_IT(&huart3, &Rx3_Temp, 1);
	
	
//	UART4_Put_Char(Flash_Pid_R[1]);
//	UART4_Put_Char(Flash_Pid_R[0]);
//	UART4_Put_Char(Flash_Pid_R[37]);
//	UART4_Put_Char(Flash_Pid_R[36]);
//	UART4_Put_Char(FlashRBuff[2]);
//	OLED_ShowStr(30,2,&FlashRBuff[0],2);
//	OLED_ShowStr(30,6,&FlashRBuff[1],2);
//	OLED_ShowStr(30,10,&FlashRBuff[2],2);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Is_Recv_New_Cmd())
		{
			Parse_Cmd_Data(Get_RxBuffer(), Get_CMD_Length());
			Clear_CMD_Flag();
    }
	}
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
