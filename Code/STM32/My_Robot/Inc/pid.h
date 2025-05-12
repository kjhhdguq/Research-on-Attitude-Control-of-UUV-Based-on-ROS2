#ifndef __PID_H_
#define __PID_H_


#include "main.h"
#include "protocol.h"

typedef volatile struct
{
	float desired;				//期望值
	float offset;					//设置标志位
	float prevError;			//上一次偏差
	float integ;					//积分项
	float kp;							//比例系数
	float ki;							//积分系数
	float kd;							//微分增益
	float IntegLimitHigh;	//积分限幅上限
	float IntegLimitLow;	//积分限幅下限
	float measured;				//测量值
	float out;						//输出值
	float OutLimitHigh;		//输出限幅上限
	float OutLimitLow;		//输出限幅下限
}PidObject;

extern double pid_interval;
extern uint8_t pid_interval_cnt;

extern PidObject pidAccX; //内环PID数据
extern PidObject pidAccY;
extern PidObject pidAccZ;

extern PidObject pidRateX; //内环PID数据
extern PidObject pidRateY;
extern PidObject pidRateZ;

extern PidObject pidRoll; //外环PID数据
extern PidObject pidPitch;
extern PidObject pidYaw;

extern PidObject pidRateX_F; //内环PID数据
extern PidObject pidRateY_F;

extern PidObject pidRoll_F; //外环PID数据
extern PidObject pidPitch_F;

extern PidObject *(pPidObject[]);

void PID_ParamInit(void);
void PID_Desired_Init(void);
extern void CascadePID(PidObject* pidRate,PidObject* pidAngE,const float dt);  //串级PID
extern void pidRest(PidObject **pid,const uint8_t len); //pid数据复位
extern void pidUpdate(PidObject* pid,const float dt);  //PID
extern void pidParamUpdate(PidObject* pid, unsigned char Pid_Id, unsigned char *data_buff);
extern void pidParamLoad(PidObject** pid, unsigned char *data_buf);

#endif
