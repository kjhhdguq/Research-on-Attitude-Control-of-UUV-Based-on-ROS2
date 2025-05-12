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
//58��ת��67��ת

//   M
//01   02
//03   04
//12��ת��67��ת

//����IMU��������

//������̬
//��ʼ��ˮ����
void Robot_UpDown(int degree)
{
	//����ͬ���Ӽ���һ��
	Motor_Set_Speed(5,degree);
	Motor_Set_Speed(8,degree);
	Motor_Set_Speed(6,-degree);
	Motor_Set_Speed(7,-degree);
}


//�ƶ�����ת����ת�ٿ��Ը�λΪ0����������Ҫ�˷����������Բ���
//��ת��̬
//��ʼ��ˮ����
//��ת-2��1��3��4��(degreeΪ��)
//��ת-2��1��3��4��(degreeΪ��)
void Robot_RightLeft(int degree)
{
	Motor_Set_Speed(2,degree);
	Motor_Set_Speed(1,-degree);
	Motor_Set_Speed(3,degree);
	Motor_Set_Speed(4,-degree);
}


//�ƶ���̬
//34ת�ط����෴ͬ�����仯��12ת�ط����෴ͬ�����仯����֤��ת�ز���
//ǰ��-��12��С�ٶȣ�34�����ٶ�
//����-��12�����ٶȣ�34��С�ٶ�
//������ͬ���Ӽ�������ͬ���Ӽ�����������¼
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


// ��IMU��̬������ʹ�ô���PID�����ٶ�Ϊ�ڻ����Ƕ�Ϊ�⻷��
// dtΪ����ʱ��5ms
void Robot_PidControl(float dt)
{
	pidAccX.measured = stcAcc.a[0] / 32768.0f;
	pidAccY.measured = stcAcc.a[1] / 32768.0f;
	pidAccZ.measured = stcAcc.a[2] / 32768.0f;
	
	// �ڻ�����ֵ ��λ���Ƕ�/��
	pidRateX.measured = stcGyro.w[0] / 32768.0f * 2000;
	pidRateY.measured = stcGyro.w[1] / 32768.0f * 2000;
	pidRateZ.measured = stcGyro.w[2] / 32768.0f * 2000;

	// �⻷����ֵ ��λ���Ƕ�
	pidRoll.measured = stcAngle.Angle[0] / 32768.0f * 180;		//roll
	pidPitch.measured = stcAngle.Angle[1] / 32768.0f * 180;	//pitch
	pidYaw.measured = stcAngle.Angle[2] / 32768.0f * 180;		//yaw
	
	// ����תPID����
//	if(updown_rev >= 0)
//	{
	
//		pidUpdate(&pidAccX,dt);    //����PID�������������⻷	�����PID
//		pidUpdate(&pidAccY,dt);    //����PID�������������⻷	�����PID
//		pidUpdate(&pidAccZ,dt);    //����PID�������������⻷	�����PID
//		pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
//		pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�
		
		pidUpdate(&pidRoll,dt);    //����PID�������������⻷	�����PID		
		pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
		pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�

		pidUpdate(&pidPitch,dt);    //����PID�������������⻷	������PID
		pidRateY.desired = pidPitch.out;
		pidUpdate(&pidRateY,dt); //�ٵ����ڻ�
//	}
//	else
//	{
//		pidUpdate(&pidRoll_F,dt);    //����PID�������������⻷	�����PID
//		pidRateX.desired = pidRoll_F.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
//		pidUpdate(&pidRateX_F,dt);  //�ٵ����ڻ�
//		
//		pidUpdate(&pidPitch_F,dt);    //����PID�������������⻷	������PID
//		pidRateY.desired = pidPitch_F.out;
//		pidUpdate(&pidRateY_F,dt); //�ٵ����ڻ�
//	}
	
	pidUpdate(&pidYaw,dt);    //����PID�������������⻷	������PID	
	pidRateZ.desired = pidYaw.out;
	pidUpdate(&pidRateZ,dt); //�ٵ����ڻ�
}


void MotorControl(void)
{
	int16_t temp;
	
	temp = updown_rev; //����+�������ֵ
	
	if(updown_rev > 0)
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = LIMIT(temp,0,600); //��30����̬����
	}
	else if(updown_rev < 0)
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = LIMIT(temp,-600,0); //��30����̬����
	}
	else
	{
		MOTOR5 = MOTOR6 = MOTOR7 = MOTOR8 = 0;
	}
	
	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
//	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(temp,10,90);
	
	// ����PID�ڻ�������Ÿ�����������
	
	// Roll��Pitch
	// ��ת
	if(updown_rev > 0)
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// ��̬����������������Ŀ�����
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
//		// �������
//		MOTOR5 = LIMIT(MOTOR5,0,900);
//		MOTOR6 = LIMIT(MOTOR6,0,900);
//		MOTOR7 = LIMIT(MOTOR7,0,900);
//		MOTOR8 = LIMIT(MOTOR8,0,900);
	}
	
	// ��ת
	else if(updown_rev < 0)
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// ��̬����������������Ŀ�����
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
//		// �������
//		MOTOR5 = LIMIT(MOTOR5,-900,0);
//		MOTOR6 = LIMIT(MOTOR6,-900,0);
//		MOTOR7 = LIMIT(MOTOR7,-900,0);
//		MOTOR8 = LIMIT(MOTOR8,-900,0);
	}
	
	else
	{
		MOTOR5 +=    + pidRateX.out - pidRateY.out;// ��̬����������������Ŀ�����
		MOTOR6 +=    - pidRateX.out - pidRateY.out;//
		MOTOR7 +=    + pidRateX.out + pidRateY.out;//
		MOTOR8 +=    - pidRateX.out + pidRateY.out;//
		
		MOTOR5 = LIMIT(MOTOR5,-900,900);
		MOTOR6 = LIMIT(MOTOR6,-900,900);
		MOTOR7 = LIMIT(MOTOR7,-900,900);
		MOTOR8 = LIMIT(MOTOR8,-900,900);
		
//		if(pidRateX.out < 0)//57С��0
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
//		// �������
//		MOTOR5 = LIMIT(MOTOR5,0,900);
//		MOTOR6 = LIMIT(MOTOR6,0,900);
//		MOTOR7 = LIMIT(MOTOR7,0,900);
//		MOTOR8 = LIMIT(MOTOR8,0,900);
	}
	
	// �������
	MOTOR5 = LIMIT(MOTOR5,-900,900);
	MOTOR6 = LIMIT(MOTOR6,-900,900);
	MOTOR7 = LIMIT(MOTOR7,-900,900);
	MOTOR8 = LIMIT(MOTOR8,-900,900);
	
	// Yaw��ʸ�������������
	MOTOR1 +=    - pidRateZ.out + goback_rev - rightleft_rev;// ��̬����������������Ŀ�����
	MOTOR2 +=    + pidRateZ.out + goback_rev + rightleft_rev;//
	MOTOR3 +=    + pidRateZ.out - goback_rev - rightleft_rev;//
	MOTOR4 +=    - pidRateZ.out - goback_rev + rightleft_rev;//
	
	MOTOR1 = LIMIT(MOTOR1,-900,900);
	MOTOR2 = LIMIT(MOTOR2,-900,900);
	MOTOR3 = LIMIT(MOTOR3,-900,900);
	MOTOR4 = LIMIT(MOTOR4,-900,900);
	
	// ����PWM
	Motor_Set_Speed(5,MOTOR5);
	Motor_Set_Speed(6,-MOTOR6);
	Motor_Set_Speed(7,-MOTOR7);
	Motor_Set_Speed(8,MOTOR8);
	
	Motor_Set_Speed(1,-MOTOR1);
	Motor_Set_Speed(2,-MOTOR2);
	Motor_Set_Speed(3,MOTOR3);
	Motor_Set_Speed(4,MOTOR4);
}

// �ϱ�PID�ڻ������
void PIDGyroOut_Send_Data(void)
{
  #define PidOutLEN        7
  uint8_t data_buffer[PidOutLEN] = {0};
  uint8_t i, checknum = 0;
  
  // ���ֽ���ǰ�����ֽ��ں�
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

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < PidOutLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[PidOutLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x13); // ��ʶλ
  UART1_Put_Char(0x06); // ����λ����(�ֽ���)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // ֡β
}
