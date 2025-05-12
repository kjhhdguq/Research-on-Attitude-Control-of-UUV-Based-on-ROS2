#include "pid.h"


// pid���ʱ��
double pid_interval = 0.001f;
uint8_t pid_interval_cnt;

// ���ٶȼ�PID
PidObject pidAccX; //�ڻ�PID����
PidObject pidAccY;
PidObject pidAccZ;

// X���ǹ�ת��y���Ǹ�����z���Ǻ���
PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidRateX_F;
PidObject pidRateY_F;
PidObject pidRoll_F;
PidObject pidPitch_F;

PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //�ṹ�����飬��ÿһ�������һ��pid�ṹ�壬�����Ϳ���������������PID��������  �������ʱ������λpid�������ݣ�����������仰�����þͿ�����
		,&pidRateX_F,&pidRateY_F,&pidRoll_F,&pidPitch_F
};

void PID_ParamInit(void)
{
	// �ڻ�PD
	pidRateX_F.kp = 0.8f;
	pidRateY_F.kp = 0.8f;

	pidRateX_F.ki = 0.0f;
	pidRateY_F.ki = 0.0f;

	pidRateX_F.kd = 0.0f;
	pidRateY_F.kd = 0.0f;
	
	// �⻷P
	pidPitch_F.kp	= 0.5f;
	pidRoll_F.kp	= 0.5f;
	
	pidPitch_F.ki	= 0.0f;
	pidRoll_F.ki	= 0.0f;
	
	pidPitch_F.kd = 0.0f;
	pidRoll_F.kd = 0.0f;
	
	pidAccX.kp = 0.0f;
	pidAccY.kp = 2.0f;
	pidAccZ.kp = 0.0f;
}

void PID_Desired_Init(void)
{
	// ��ʼ����ֵ
	pidRoll.desired = 0;
	pidPitch.desired = 0;
	pidYaw.desired = 0;
	
	pidRateX.desired = 0;
	pidRateY.desired = 0;
	pidRateZ.desired = 0;
	
	// �ṹ���ڲ���ز���
	pidRoll.measured = pidRoll.out = pidRoll.integ = 0;
	pidPitch.measured = pidPitch.out = pidPitch.integ = 0;
	pidYaw.measured = pidYaw.out = pidYaw.integ = 0;
	
	pidRateX.measured = pidRateX.out = pidRateX.integ = 0;
	pidRateY.measured = pidRateY.out = pidRateY.integ = 0;
	pidRateZ.measured = pidRateZ.out = pidRateZ.integ = 0;
	
	pid_interval_cnt = (uint8_t)(pid_interval * 1000);
}


void pidRest(PidObject **pid,const uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
			pid[i]->desired = 0;
			pid[i]->measured = 0;
	  	pid[i]->integ = 0;
	    pid[i]->prevError = 0;
	    pid[i]->out = 0;
			pid[i]->offset = 0;
	}
	pid_interval_cnt = (uint8_t)(pid_interval * 1000);
}


void pidUpdate(PidObject* pid,const float dt)
{
	float error;
	float deriv;

	error = pid->desired - pid->measured; //��ǰ�Ƕ���ʵ�ʽǶȵ����
	pid->integ += error * dt;	 //�������ۼ�ֵ
//	pid->integ = LIMIT(pid->integ,pid->IntegLimitLow,pid->IntegLimitHigh); //���л����޷�
	deriv = (error - pid->prevError)/dt;  //ǰ�����������΢��
	pid->out = pid->kp * error + pid->ki * pid->integ + pid->kd * deriv;//PID���
//	pid->out = LIMIT(pid->out,pid->OutLimitLow,pid->OutLimitHigh); //����޷�
	pid->prevError = error;  //�����ϴε����	
}


void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt)  //����PID
{	 
	pidUpdate(pidAngE,dt);    //�ȼ����⻷
	pidRate->desired = pidAngE->out;
	pidUpdate(pidRate,dt);    //�ټ����ڻ�	
}


void pidParamUpdate(PidObject* pid, unsigned char Pid_Id, unsigned char *data_buf)
{
	// ֻ��ʹ�ܱ�־λ�򿪲��ܽ��и���
	if(pidflag_recv == 1)
	{
		// pid�������µ��������
		
		u16 kp_recv = *(data_buf + 4);
    kp_recv = (kp_recv << 8) | *(data_buf + 3);

    u16 ki_recv = *(data_buf + 6);
    ki_recv = (ki_recv << 8) | *(data_buf + 5);

    u16 kd_recv = *(data_buf + 8);
    kd_recv = (kd_recv << 8) | *(data_buf + 7);

    float kp = (float)kp_recv / 10000.0;
    float ki = (float)ki_recv / 10000.0;
    float kd = (float)kd_recv / 10000.0;
		
		pid->kp = kp;
		pid->ki = ki;
		pid->kd = kd;
		
		// pid�������浽д������
		uint8_t i = (Pid_Id-1)*6;
		uint8_t j = 3;
		
		for(j=3; j<= 8; j++)
		{
			Flash_Pid[i] = *(data_buf + j);
			i++;
		}
		
		Out_Send_Data(kp_recv, ki_recv, kd_recv);
//		Led1_Flag = 0;
//		OLED_CLS();
//		
//		OLED_ShowStr(10, 2, "Kp:", 1);
//		OLED_ShowNum(30,2,((int)(pid->kp))/10,1,2);
//		OLED_ShowNum(40,2,((int)(pid->kp))%10,1,2);
//		OLED_ShowNum(50,2,(int)((pid->kp)*100)/10%10,1,2);
//		OLED_ShowNum(60,2,(int)((pid->kp)*100)%10,1,2);
//		
//		OLED_ShowStr(10, 4, "Ki:", 1);
//		OLED_ShowNum(30,4,((int)(pid->ki))/10,1,2);
//		OLED_ShowNum(40,4,((int)(pid->ki))%10,1,2);
//		OLED_ShowNum(50,4,(int)((pid->ki)*100)/10%10,1,2);
//		OLED_ShowNum(60,4,(int)((pid->ki)*100)%10,1,2);
//		
//		OLED_ShowStr(10, 6, "Kd:", 1);
//		OLED_ShowNum(30,6,((int)(pid->kd))/10,1,2);
//		OLED_ShowNum(40,6,((int)(pid->kd))%10,1,2);
//		OLED_ShowNum(50,6,(int)((pid->kd)*100)/10%10,1,2);
//		OLED_ShowNum(60,6,(int)((pid->kd)*100)%10,1,2);
	}
}


void pidParamLoad(PidObject** pid, unsigned char *data_buf)
{
	for(uint8_t i=0;i<6;i++)
	{
		u16 kp_recv = *(data_buf + 6*i + 1);
		kp_recv = (kp_recv << 8) | *(data_buf + 6*i);

		u16 ki_recv = *(data_buf + 6*i + 3);
		ki_recv = (ki_recv << 8) | *(data_buf + 6*i + 2);

		u16 kd_recv = *(data_buf + 6*i + 5);
		kd_recv = (kd_recv << 8) | *(data_buf + 6*i + 4);
		
		pid[i]->kp = (float)kp_recv / 10000.0;
		pid[i]->ki = (float)ki_recv / 10000.0;
		pid[i]->kd = (float)kd_recv / 10000.0;
	}
	u16 pid_interval_rev = *(data_buf + 37);
	pid_interval_rev = (pid_interval_rev << 8) | *(data_buf + 36);
	
	pid_interval = (double)pid_interval_rev / 1000.0;
}
