#ifndef __PID_H_
#define __PID_H_


#include "main.h"
#include "protocol.h"

typedef volatile struct
{
	float desired;				//����ֵ
	float offset;					//���ñ�־λ
	float prevError;			//��һ��ƫ��
	float integ;					//������
	float kp;							//����ϵ��
	float ki;							//����ϵ��
	float kd;							//΢������
	float IntegLimitHigh;	//�����޷�����
	float IntegLimitLow;	//�����޷�����
	float measured;				//����ֵ
	float out;						//���ֵ
	float OutLimitHigh;		//����޷�����
	float OutLimitLow;		//����޷�����
}PidObject;

extern double pid_interval;
extern uint8_t pid_interval_cnt;

extern PidObject pidAccX; //�ڻ�PID����
extern PidObject pidAccY;
extern PidObject pidAccZ;

extern PidObject pidRateX; //�ڻ�PID����
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidRoll; //�⻷PID����
extern PidObject pidPitch;
extern PidObject pidYaw;

extern PidObject pidRateX_F; //�ڻ�PID����
extern PidObject pidRateY_F;

extern PidObject pidRoll_F; //�⻷PID����
extern PidObject pidPitch_F;

extern PidObject *(pPidObject[]);

void PID_ParamInit(void);
void PID_Desired_Init(void);
extern void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //����PID
extern void pidRest(PidObject **pid,const uint8_t len); //pid���ݸ�λ
extern void pidUpdate(PidObject* pid,const float dt);  //PID
extern void pidParamUpdate(PidObject* pid, unsigned char Pid_Id, unsigned char *data_buff);
extern void pidParamLoad(PidObject** pid, unsigned char *data_buf);

#endif
