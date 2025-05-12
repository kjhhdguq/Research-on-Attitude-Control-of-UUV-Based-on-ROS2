#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_


#include "main.h"
#include "gpio.h"
#include "JY901.h"
#include "tim.h"
#include "motion_control.h"
#include "eeprom.h"

#define FUNC_MOTION									0x01			//
#define FUNC_BEEP_LED								0x07			// LED-BEEP控制帧

#define FUNC_CONMODE								0x08			// 控制模式帧
#define FUNC_MOVECON								0x09			// 控制帧

#define FUNC_SET_PID_FLAG						0x0A			// PID调试标志位
#define FUNC_SET_PIDRATEX						0x0B			// X轴角速度PID参数帧
#define FUNC_SET_PIDRATEY						0x0C			// Y轴角速度PID参数帧
#define FUNC_SET_PIDRATEZ						0x0D			// Z轴角速度PID参数帧

#define FUNC_SET_PIDROLL						0x0E			// 滚转角PID参数帧
#define FUNC_SET_PIDPITCH						0x0F			// 俯仰角PID参数帧
#define FUNC_SET_PIDYAW							0x10			// 航向角PID参数帧

#define FUNC_SET_PID_INTERVAL				0x11			// PID间隔时间

#define FUNC_SET_MOTOR							0x12			// 调节单个电机帧

#define PTO_MAX_BUF_LEN           20

extern u8 New_CMD_flag;
extern u8 New_Speed_Flag;

extern unsigned char openclose_rev;
extern unsigned char Led1_Flag;
extern unsigned char Led2_Flag;
extern unsigned char Ledf_Flag;
extern unsigned char Cali_Flag;
//extern unsigned char Buzzer_Flag;
//extern unsigned char Buzzer_times;
//extern uint32_t Buzzer_Inter_On;
//extern uint32_t Buzzer_Inter_Off;

extern int16_t goback_rev;
extern int16_t rightleft_rev;
extern int16_t updown_rev;
extern int16_t roll_rev;
extern int16_t yaw_rev;
extern int16_t pitch_rev;

extern u8 pidflag_recv;

extern uint8_t Flash_Pid[38];

void Upper_Data_Receive(u8 data);
void Parse_Cmd_Data(u8 *data_buf, u8 num);
void Out_Send_Data(uint16_t data1,uint16_t data2, uint16_t data3);

void Clear_CMD_Flag(void);
void Clear_RxBuffer(void);
u8* Get_RxBuffer(void);
u8 Get_CMD_Length(void);
u8 Is_Recv_New_Cmd(void);

#endif
