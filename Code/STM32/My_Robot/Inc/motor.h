#ifndef __MOTOR_H_
#define __MOTOR_H_


#include "main.h"
#include "tim.h"
#include "stm32f1xx_it.h"


void Motor_Init(void);
void Motor_Set_Speed(uint8_t id, int16_t speed);

#endif
