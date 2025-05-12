#include "motion_control.h"


int16_t motor[8];
#define MOTOR1 motor[0]
#define MOTOR2 motor[1]
#define MOTOR3 motor[2]
#define MOTOR4 motor[3]
#define MOTOR5 motor[4]
#define MOTOR6 motor[5]
#define MOTOR7 motor[6]
#define MOTOR8 motor[7]

//   M
//05   06
//07   08
//58正转，67反转

//   M
//01   02
//03   04
//12反转，67正转

//不带IMU基本控制

//升降姿态
//初始入水推力
void Robot_UpDown(int degree)
{
	//升降同步加减量一致
	Motor_Set_Speed(5,degree);
	Motor_Set_Speed(8,degree);
	Motor_Set_Speed(6,-degree);
	Motor_Set_Speed(7,-degree);
}


//移动和旋转后电机转速可以复位为0，但是升降要克服浮力，所以不行
//旋转姿态
//初始入水推力
//左转-2增1减3增4减(degree为正)
//右转-2减1增3减4增(degree为负)
void Robot_RightLeft(int degree)
{
	Motor_Set_Speed(2,degree);
	Motor_Set_Speed(1,-degree);
	Motor_Set_Speed(3,degree);
	Motor_Set_Speed(4,-degree);
}


//移动姿态
//34转矩方向相反同步量变化，12转矩方向相反同步量变化，保证合转矩不变
//前进-则12减小速度，34增加速度
//后退-则12增加速度，34减小速度
//即正浆同步加减，反浆同步加减，两变量记录
void Robot_GoBack(int degree)
{
	Motor_Set_Speed(1,-degree);
	Motor_Set_Speed(2,-degree);
	Motor_Set_Speed(3,degree);
	Motor_Set_Speed(4,degree);
}


double degToRad(double deg)
{
    return deg / 180.0 * M_PI;
}


// 带IMU姿态调整（使用串级PID，角速度为内环，角度为外环）
// dt为测量时间5ms
void Robot_PidControl(float dt)
{
	pidAccX.measured = stcAcc.a[0] / 32768.0f;
	pidAccY.measured = stcAcc.a[1] / 32768.0f;
	pidAccZ.measured = stcAcc.a[2] / 32768.0f;
	
	// 内环测量值 单位：角度/秒
	pidRateX.measured = stcGyro.w[0] / 32768.0f * 2000;
	pidRateY.measured = stcGyro.w[1] / 32768.0f * 2000;
	pidRateZ.measured = stcGyro.w[2] / 32768.0f * 2000;

	// 外环测量值 单位：角度
	pidRoll.measured = stcAngle.Angle[0] / 32768.0f * 180;		//roll
	pidPitch.measured = stcAngle.Angle[1] / 32768.0f * 180;	//pitch
	pidYaw.measured = stcAngle.Angle[2] / 32768.0f * 180;		//yaw
	
	// 正反转PID区分
//	if(updown_rev >= 0)
//	{
	
//		pidUpdate(&pidAccX,dt);    //调用PID处理函数来处理外环	横滚角PID
//		pidUpdate(&pidAccY,dt);    //调用PID处理函数来处理外环	横滚角PID
//		pidUpdate(&pidAccZ,dt);    //调用PID处理函数来处理外环	横滚角PID
//		pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
//		pidUpdate(&pidRateX,dt);  //再调用内环
		
		pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
		pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
		pidUpdate(&pidRateX,dt);  //再调用内环

		pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID
		pidRateY.desired = pidPitch.out;
		pidUpdate(&pidRateY,dt); //再调用内环
//	}
//	else
//	{
//		pidUpdate(&pidRoll_F,dt);    //调用PID处理函数来处理外环	横滚角PID
//		pidRateX.desired = pidRoll_F.out; //将外环的PID输出作为内环PID的期望值即为串级PID
//		pidUpdate(&pidRateX_F,dt);  //再调用内环
//		
//		pidUpdate(&pidPitch_F,dt);    //调用PID处理函数来处理外环	俯仰角PID
//		pidRateY.desired = pidPitch_F.out;
//		pidUpdate(&pidRateY_F,dt); //再调用内环
//	}
	
	pidUpdate(&pidYaw,dt);    //调用PID处理函数来处理外环	俯仰角PID	
	pidRateZ.desired = pidYaw.out;
	pidUpdate(&pidRateZ,dt); //再调用内环
}


void MotorControl(void)
{
	int16_t temp;
	
	temp = updown_rev; //油门+定深输出值
	
	if(updown_rev > 0)
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = LIMIT(temp,0,600); //留30给姿态控制
	}
	else if(updown_rev < 0)
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = LIMIT(temp,-600,0); //留30给姿态控制
	}
	else
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = 0;
	}
	
	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
//	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,10,90);
	
	// 串级PID内环的输出才给予电机控制量
	
	// Roll和Pitch
	// 正转
	if(updown_rev > 0)
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// 姿态输出分配给各个电机的控制量
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
//		// 输出限制
//		MOTOR5 = LIMIT(MOTOR5,0,900);
//		MOTOR6 = LIMIT(MOTOR6,0,900);
//		MOTOR7 = LIMIT(MOTOR7,0,900);
//		MOTOR8 = LIMIT(MOTOR8,0,900);
	}
	
	// 反转
	else if(updown_rev < 0)
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// 姿态输出分配给各个电机的控制量
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
//		// 输出限制
//		MOTOR5 = LIMIT(MOTOR5,-900,0);
//		MOTOR6 = LIMIT(MOTOR6,-900,0);
//		MOTOR7 = LIMIT(MOTOR7,-900,0);
//		MOTOR8 = LIMIT(MOTOR8,-900,0);
	}
	
	else
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// 姿态输出分配给各个电机的控制量
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
		MOTOR5 = LIMIT(MOTOR5,-900,900);
		MOTOR6 = LIMIT(MOTOR6,-900,900);
		MOTOR7 = LIMIT(MOTOR7,-900,900);
		MOTOR8 = LIMIT(MOTOR8,-900,900);
		
//		if(pidRateX.out < 0)//57小于0
//		{
//			MOTOR5 = LIMIT(MOTOR5,-MOTOR6,900);
//			MOTOR6 = LIMIT(MOTOR6,-900,900);
//			MOTOR7 = LIMIT(MOTOR7,-MOTOR8,900);
//			MOTOR8 = LIMIT(MOTOR8,-900,900);
//		}
//		else
//		{
//			MOTOR5 = LIMIT(MOTOR5,-900,900);
//			MOTOR6 = LIMIT(MOTOR6,-MOTOR5,900);
//			MOTOR7 = LIMIT(MOTOR7,-900,900);
//			MOTOR8 = LIMIT(MOTOR8,-MOTOR7,900);
//		}
//		// 输出限制
//		MOTOR5 = LIMIT(MOTOR5,0,900);
//		MOTOR6 = LIMIT(MOTOR6,0,900);
//		MOTOR7 = LIMIT(MOTOR7,0,900);
//		MOTOR8 = LIMIT(MOTOR8,0,900);
	}
	
	// 输出限制
	MOTOR5 = LIMIT(MOTOR5,-900,900);
	MOTOR6 = LIMIT(MOTOR6,-900,900);
	MOTOR7 = LIMIT(MOTOR7,-900,900);
	MOTOR8 = LIMIT(MOTOR8,-900,900);
	
	// Yaw由矢量电机单独控制
	MOTOR1 +=    - pidRateZ.out + goback_rev - rightleft_rev;// 姿态输出分配给各个电机的控制量
	MOTOR2 +=    + pidRateZ.out + goback_rev + rightleft_rev;//
	MOTOR3 +=    + pidRateZ.out - goback_rev - rightleft_rev;//
	MOTOR4 +=    - pidRateZ.out - goback_rev + rightleft_rev;//
	
	MOTOR1 = LIMIT(MOTOR1,-900,900);
	MOTOR2 = LIMIT(MOTOR2,-900,900);
	MOTOR3 = LIMIT(MOTOR3,-900,900);
	MOTOR4 = LIMIT(MOTOR4,-900,900);
	
	// 更新PWM
	Motor_Set_Speed(5,MOTOR5);
	Motor_Set_Speed(6,-MOTOR6);
	Motor_Set_Speed(7,-MOTOR7);
	Motor_Set_Speed(8,MOTOR8);
	
	Motor_Set_Speed(1,-MOTOR1);
	Motor_Set_Speed(2,-MOTOR2);
	Motor_Set_Speed(3,MOTOR3);
	Motor_Set_Speed(4,MOTOR4);
}

// 上报PID内环输出量
void PIDGyroOut_Send_Data(void)
{
  #define PidOutLEN        7
  uint8_t data_buffer[PidOutLEN] = {0};
  uint8_t i, checknum = 0;
  
  // 低字节在前，高字节在后
  // Roll
  data_buffer[0] = (int)(pidRateX.out)&0xFF;
  data_buffer[1] = (((int)pidRateX.out)>>8)&0xFF;
	// Pitch
  data_buffer[2] = (int)(pidRateY.out)&0xFF;
  data_buffer[3] = (((int)pidRateY.out)>>8)&0xFF;
  // Yaw
  data_buffer[4] = (int)(pidRateZ.out)&0xFF;
  data_buffer[5] = (((int)pidRateZ.out)>>8)&0xFF;
	
//	data_buffer[0] = (int)((pPidObject[5]->kp)*100)&0xFF;
//  data_buffer[1] = ((int)((pPidObject[5]->kp)*100) >> 8)&0xFF;
//	
//	data_buffer[2] = (int)((pPidObject[5]->ki)*100)&0xFF;
//  data_buffer[3] = ((int)((pPidObject[5]->ki)*100) >> 8)&0xFF;

//  data_buffer[4] = (int)((pPidObject[5]->kd)*100)&0xFF;
//  data_buffer[5] = ((int)((pPidObject[5]->kd)*100) >> 8)&0xFF;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < PidOutLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[PidOutLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x13); // 标识位
  UART1_Put_Char(0x06); // 数据位长度(字节数)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // 帧尾
}
