#include "protocol.h"


// 命令接收缓存
u8 RxBuffer[PTO_MAX_BUF_LEN];
// 接收数据下标
u8 RxIndex = 0;
// 接收状态机
u8 RxFlag = 0;
// 新命令接收标志
u8 New_CMD_flag;
// 新命令数据长度
u8 New_CMD_length;

u8 New_Speed_Flag = 0;

int16_t goback_rev = 0;
int16_t rightleft_rev = 0;
int16_t updown_rev = 0;
int16_t pitch_rev = 0;
int16_t roll_rev = 0;
int16_t yaw_rev = 0;

u8 pidflag_recv = 0;
u8 flash_flag = 0;


// 开闭环控制使能（0为开环，1为闭环）
uint8_t openclose_rev = 1;

// LED、BEEP中断控制位
uint8_t Led1_Flag = 0;
uint8_t Led2_Flag = 0;
uint8_t Ledf_Flag = 0;
uint8_t Cali_Flag = 0;

// pid读写数组
uint8_t Flash_Pid[38] = {0};


// 获取接收的数据
u8* Get_RxBuffer(void)
{
  return (u8*)RxBuffer;
}

// 获取命令长度
u8 Get_CMD_Length(void)
{
  return New_CMD_length;
}

// 获取命令标志
u8 Is_Recv_New_Cmd(void)
{
  return New_CMD_flag;
}

// 清除命令数据和相关标志
void Clear_CMD_Flag(void)
{
  #if ENABLE_CLEAR_RXBUF
  for (u8 i = 0; i < New_CMD_length; i++)
    RxBuffer[i] = 0;
  #endif
  New_CMD_length = 0;
  New_CMD_flag = 0;
}

// RxBuffer置0
void Clear_RxBuffer(void)
{
  for (u8 i = 0; i < PTO_MAX_BUF_LEN; i++)
    RxBuffer[i] = 0;
}

// 指令解析，传入接收到的完整指令，及其长度
void Parse_Cmd_Data(u8 *data_buf, u8 num)
{
  #if ENABLE_CHECKSUM
  // 计算校验
  int sum = 0;
  for (u8 i = 3; i < (num - 2); i++)
    sum += *(data_buf + i);
  sum = sum & 0xFF;

  u8 recvSum = *(data_buf + num - 2);
  if (!(sum == recvSum))
    return;
  #endif

  // 判断帧头
  if (!(*(data_buf) == 0x55))
    return;

  u8 func_id = *(data_buf + 1);
  switch (func_id) {
  // 判断功能字：速度控制
  case FUNC_MOTION:
  {
    u8 index_l = *(data_buf + 3);
    u16 left = *(data_buf + 5);
    left = (left << 8) | (*(data_buf + 4));

    u8 index_r = *(data_buf + 6);
    u16 right = *(data_buf + 8);
    right = (right << 8) | (*(data_buf + 7));

//    Motion_Test_SpeedSet(index_l, left, index_r, right);
    break;
  }
  
	// 控制方式（开环、闭环）
	case FUNC_CONMODE:
	{
		u8 openclose_ctrl_en = *(data_buf + 7); // 使能控制字段
    u8 openclose = *(data_buf + 8);         // 状态字段
    if (openclose_ctrl_en)
		{
			if(openclose)
			{
				openclose_rev = 1;
				Buzzer_Reset();
				Open_Con.Buzzer_Flag = 1;
			}
			else
			{
				openclose_rev = 0;
				Buzzer_Reset();
				Closed_Con.Buzzer_Flag = 1;
			}
		}
		
		break;
	}
	
	// 运动控制
	case FUNC_MOVECON:
	{
		updown_rev = *(data_buf + 4);
		updown_rev = (updown_rev << 8) | (*(data_buf + 3));
		
		yaw_rev = *(data_buf + 6);
		yaw_rev = (yaw_rev << 8) | (*(data_buf + 5));
		
		pitch_rev = *(data_buf + 8);
		pitch_rev = (pitch_rev << 8) | (*(data_buf + 7));
		
		roll_rev = *(data_buf + 10);
		roll_rev = (roll_rev << 8) | (*(data_buf + 9));
		
		goback_rev = *(data_buf + 12);
		goback_rev = (goback_rev << 8) | (*(data_buf + 11));
		
		rightleft_rev = *(data_buf + 14);
		rightleft_rev = (rightleft_rev << 8) | (*(data_buf + 13));
		
		// 闭环期望角度赋值
		if(openclose_rev == 1)
		{
			pidRoll.desired = roll_rev / 1.0;		// <>
			pidPitch.desired = pitch_rev / 1.0;	// WS
			pidYaw.desired = yaw_rev / 1.0;		// AD
			
			goback_rev = goback_rev / 1.0;		// <>
			rightleft_rev = rightleft_rev / 1.0;		// <>
		}
		// 开环新速度使能
		else
		{
			New_Speed_Flag = 1;
		}
		
		// 指示灯
		if((yaw_rev != 0) || (pitch_rev != 0))
			Led1_Flag = 1;
		else
			Led1_Flag = 0;
		
		break;
	}
	
  // 判断功能字：LED、蜂鸣器状态、imu校准
  case FUNC_BEEP_LED:
  {
    u8 led_ctrl_en = *(data_buf + 3); // 使能控制字段
    u8 led = *(data_buf + 4);         // 状态字段
    if (led_ctrl_en) {
      if (led)
			{
        Led1_Flag = 1;
			}
      else
        Led1_Flag = 0;
    }

    u8 buzzer_ctrl_en = *(data_buf + 5); // 使能控制字段
    u8 buzzer = *(data_buf + 6);         // 状态字段
    if (buzzer_ctrl_en) {
      if (buzzer)
        BUZZER_ON();
      else
        BUZZER_OFF();
    }

    u8 calibration_ctrl_en = *(data_buf + 7); // 使能控制字段
    u8 calibration = *(data_buf + 8);         // 状态字段
    if (calibration_ctrl_en && calibration)
		{
			Led1_Flag = 1;					// 指示灯
			//HAL_Delay(500);
			openclose_rev = 0;
			__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);		// 关闭串口3接收
			HAL_TIM_Base_Stop_IT(&htim1);										// 关闭定时器1中断
      jy901_calibration();														// IMU校准
			HAL_Delay(500);
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_UART_Receive_IT(&huart3, &Rx3_Temp, 1);
			pidRest(pPidObject, 6);
			roll_rev = pitch_rev = yaw_rev = goback_rev = rightleft_rev = 0;		// <>
			openclose_rev = 1;
			Led1_Flag = 0;
		}
		
		u8 ledf_ctrl_en = *(data_buf + 9); // 使能控制字段
    u8 ledf = *(data_buf + 10);         // 状态字段
    if (ledf_ctrl_en) {
      if (ledf)
			{
        Ledf_Flag = 1;
			}
      else
        Ledf_Flag = 0;
    }
		
    break;
  }
  
	// 判断功能字：pidflag--PID参数
	case FUNC_SET_PID_FLAG:
  {
    pidflag_recv = *(data_buf + 8);
		if(pidflag_recv == 0)
		{
			Led1_Flag = 0;
			if(flash_flag == 1)
			{
				HAL_TIM_Base_Stop(&htim1);
				BUZZER_ON();
				AT24C256_WriteByte(0x0000, 38, Flash_Pid);
				HAL_Delay(500);
				HAL_TIM_Base_Start_IT(&htim1);
				flash_flag = 0;
				BUZZER_OFF();
			}
		}
		else
		{
			Led1_Flag = 1;
			flash_flag = 1;
			BUZZER_OFF();
		}
    
    break;
  }
	
  // 判断功能字：pidRateX--PID参数
  case FUNC_SET_PIDRATEX:
  {
		
    pidParamUpdate(&pidRateX, 1, data_buf);
    break;
  }
	
	// 判断功能字：pidRateY--PID参数
  case FUNC_SET_PIDRATEY:
  {
		
    pidParamUpdate(&pidRateY, 2, data_buf);
    break;
  }
	
	// 判断功能字：pidRateZ--PID参数
  case FUNC_SET_PIDRATEZ:
  {
		
    pidParamUpdate(&pidRateZ, 3, data_buf);
    break;
  }
	
	// 判断功能字：pidRoll--PID参数
  case FUNC_SET_PIDROLL:
  {
		
    pidParamUpdate(&pidRoll, 4, data_buf);
    break;
  }
	
	// 判断功能字：pidPitch--PID参数
  case FUNC_SET_PIDPITCH:
  {
		
    pidParamUpdate(&pidPitch, 5, data_buf);
    break;
  }
	
	// 判断功能字：pidYaw--PID参数
  case FUNC_SET_PIDYAW:
  {
		
    pidParamUpdate(&pidYaw, 6, data_buf);
    break;
  }
	
	case FUNC_SET_PID_INTERVAL:
  {
    u16 pid_interval_rev = *(data_buf + 8);
    pid_interval_rev = (pid_interval_rev << 8) | *(data_buf + 7);
		
    pid_interval = (double)pid_interval_rev / 1000.0;
		pid_interval_cnt = pid_interval_rev;
		
		Flash_Pid[36] = *(data_buf + 7);
		Flash_Pid[37] = *(data_buf + 8);
    break;
  }
	
	case FUNC_SET_MOTOR:
  {
    int8_t motor_id = *(data_buf + 6);
    int16_t motor_speed = *(data_buf + 8);
    motor_speed = (motor_speed << 8) | (*(data_buf + 7));

    Motor_Set_Speed(motor_id, motor_speed);
    break;
  }
	
  default:
		break;
  }
}


// 打印到上位机数据
void Out_Send_Data(uint16_t data1,uint16_t data2, uint16_t data3)
{
  #define OutLEN        7
  uint8_t data_buffer[OutLEN] = {0};
  uint8_t i, checknum = 0;
  
  // 低字节在前，高字节在后
  // data1
  data_buffer[0] = data1&0xFF;
  data_buffer[1] = (data1>>8)&0xFF;
	// data2
  data_buffer[2] = data2&0xFF;
  data_buffer[3] = (data2>>8)&0xFF;
  // data3
  data_buffer[4] = data3&0xFF;
  data_buffer[5] = (data3>>8)&0xFF;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < OutLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[OutLEN-1] = checknum & 0xFF;
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



// 接收串口单字节数据接收并保存
void Upper_Data_Receive(u8 Rx_Temp)
{
  switch (RxFlag) {
  // 帧头
  case 0:
  {
    if (Rx_Temp == 0x55) {
      RxBuffer[0] = 0x55;
      RxFlag = 1;
    } else {
      RxFlag = 0;
      RxBuffer[0] = 0x0;
    }
    break;
  }

  // 标识位
  case 1:
  {
    if (Rx_Temp == 0x01 || Rx_Temp == 0x07 || Rx_Temp == 0x08 
				|| Rx_Temp == 0x09 || Rx_Temp == 0x10 || Rx_Temp == 0x11 || Rx_Temp == 0x12
				|| Rx_Temp == 0x0A || Rx_Temp == 0x0B || Rx_Temp == 0x0C || Rx_Temp == 0x0D
				|| Rx_Temp == 0x0E || Rx_Temp == 0x0F) {
      RxBuffer[1] = Rx_Temp;
      RxFlag = 2;
      RxIndex = 2;
    } else {
      RxFlag = 0;
      RxBuffer[0] = 0;
      RxBuffer[1] = 0;
    }
    break;
  }

  // 数据位长度
  case 2:
  {
    // New_CMD_length为数据帧总字节数 = 帧头+标识位+长度+校验位+帧尾(5 bytes)+数据位
    New_CMD_length = Rx_Temp+5;
    if (New_CMD_length >= PTO_MAX_BUF_LEN) {
      RxIndex = 0;
      RxFlag = 0;
      RxBuffer[0] = 0;
      RxBuffer[1] = 0;
      New_CMD_length = 0;
      break;
    }
    RxBuffer[RxIndex] = Rx_Temp;
    RxIndex++;
    RxFlag = 3;
    break;
  }

  // 读取完剩余的所有字段
  case 3:
  {
    RxBuffer[RxIndex] = Rx_Temp;
    RxIndex++;
    if (RxIndex >= New_CMD_length && RxBuffer[New_CMD_length-1] == 0xBB) {
      New_CMD_flag = 1;
      RxIndex = 0;
      RxFlag = 0;
    }
    break;
  }

  default:
    break;
  }
}

