#ifndef __MOTION_CONTROL_H_
#define __MOTION_CONTROL_H_


#include "main.h"
#include "tim.h"
#include "motor.h"
#include "JY901.h"
#include "protocol.h"
#include "usart.h"
#include "pid.h"

#define LIMIT( x,min,max )	( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
//#define LIMIT(x,min,max)((x)<(min)?(min):((x)>(max)?(max):(x)))
#define M_PI		3.14159265358979323846	/* pi */
#define OPEN_LOOP			1
#define CLOSED_LOOP		0

void Robot_UpDown(int degree);
void Robot_RightLeft(int degree);
void Robot_GoBack(int degree);

double degToRad(double deg);
void Robot_PidControl(float dt);
void MotorControl(void);
void PIDGyroOut_Send_Data(void);

#endif
