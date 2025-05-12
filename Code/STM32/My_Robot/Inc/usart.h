/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
extern uint8_t TxBuffer1[256];
extern uint8_t TxBuffer3[256];
extern uint8_t TxBuffer4[256];
extern unsigned char TxCounter1;
extern unsigned char TxCounter3;
extern unsigned char TxCounter4;
extern unsigned char count1;
extern unsigned char count3;
extern unsigned char count4;
extern unsigned char Rx1_Temp;
extern unsigned char Rx3_Temp;
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART1_Put_Char(unsigned char DataToSend);
void UART1_Put_String(unsigned char *Str);
void UART3_Put_Char(unsigned char DataToSend);
void UART3_Put_String(unsigned char *Str);
void UART4_Put_Char(unsigned char DataToSend);
void UART4_Put_String(unsigned char *Str);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

