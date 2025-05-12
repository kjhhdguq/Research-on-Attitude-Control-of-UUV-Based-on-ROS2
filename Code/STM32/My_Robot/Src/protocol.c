#include "protocol.h"


// ������ջ���
u8 RxBuffer[PTO_MAX_BUF_LEN];
// ���������±�
u8 RxIndex = 0;
// ����״̬��
u8 RxFlag = 0;
// ��������ձ�־
u8 New_CMD_flag;
// ���������ݳ���
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


// ���ջ�����ʹ�ܣ�0Ϊ������1Ϊ�ջ���
uint8_t openclose_rev = 1;

// LED��BEEP�жϿ���λ
uint8_t Led1_Flag = 0;
uint8_t Led2_Flag = 0;
uint8_t Ledf_Flag = 0;
uint8_t Cali_Flag = 0;

// pid��д����
uint8_t Flash_Pid[38] = {0};


// ��ȡ���յ�����
u8* Get_RxBuffer(void)
{
  return (u8*)RxBuffer;
}

// ��ȡ�����
u8 Get_CMD_Length(void)
{
  return New_CMD_length;
}

// ��ȡ�����־
u8 Is_Recv_New_Cmd(void)
{
  return New_CMD_flag;
}

// ����������ݺ���ر�־
void Clear_CMD_Flag(void)
{
  #if ENABLE_CLEAR_RXBUF
  for (u8 i = 0; i < New_CMD_length; i++)
    RxBuffer[i] = 0;
  #endif
  New_CMD_length = 0;
  New_CMD_flag = 0;
}

// RxBuffer��0
void Clear_RxBuffer(void)
{
  for (u8 i = 0; i < PTO_MAX_BUF_LEN; i++)
    RxBuffer[i] = 0;
}

// ָ�������������յ�������ָ����䳤��
void Parse_Cmd_Data(u8 *data_buf, u8 num)
{
  #if ENABLE_CHECKSUM
  // ����У��
  int sum = 0;
  for (u8 i = 3; i < (num - 2); i++)
    sum += *(data_buf + i);
  sum = sum & 0xFF;

  u8 recvSum = *(data_buf + num - 2);
  if (!(sum == recvSum))
    return;
  #endif

  // �ж�֡ͷ
  if (!(*(data_buf) == 0x55))
    return;

  u8 func_id = *(data_buf + 1);
  switch (func_id) {
  // �жϹ����֣��ٶȿ���
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
  
	// ���Ʒ�ʽ���������ջ���
	case FUNC_CONMODE:
	{
		u8 openclose_ctrl_en = *(data_buf + 7); // ʹ�ܿ����ֶ�
    u8 openclose = *(data_buf + 8);         // ״̬�ֶ�
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
	
	// �˶�����
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
		
		// �ջ������Ƕȸ�ֵ
		if(openclose_rev == 1)
		{
			pidRoll.desired = roll_rev / 1.0;		// <>
			pidPitch.desired = pitch_rev / 1.0;	// WS
			pidYaw.desired = yaw_rev / 1.0;		// AD
			
			goback_rev = goback_rev / 1.0;		// <>
			rightleft_rev = rightleft_rev / 1.0;		// <>
		}
		// �������ٶ�ʹ��
		else
		{
			New_Speed_Flag = 1;
		}
		
		// ָʾ��
		if((yaw_rev != 0) || (pitch_rev != 0))
			Led1_Flag = 1;
		else
			Led1_Flag = 0;
		
		break;
	}
	
  // �жϹ����֣�LED��������״̬��imuУ׼
  case FUNC_BEEP_LED:
  {
    u8 led_ctrl_en = *(data_buf + 3); // ʹ�ܿ����ֶ�
    u8 led = *(data_buf + 4);         // ״̬�ֶ�
    if (led_ctrl_en) {
      if (led)
			{
        Led1_Flag = 1;
			}
      else
        Led1_Flag = 0;
    }

    u8 buzzer_ctrl_en = *(data_buf + 5); // ʹ�ܿ����ֶ�
    u8 buzzer = *(data_buf + 6);         // ״̬�ֶ�
    if (buzzer_ctrl_en) {
      if (buzzer)
        BUZZER_ON();
      else
        BUZZER_OFF();
    }

    u8 calibration_ctrl_en = *(data_buf + 7); // ʹ�ܿ����ֶ�
    u8 calibration = *(data_buf + 8);         // ״̬�ֶ�
    if (calibration_ctrl_en && calibration)
		{
			Led1_Flag = 1;					// ָʾ��
			//HAL_Delay(500);
			openclose_rev = 0;
			__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);		// �رմ���3����
			HAL_TIM_Base_Stop_IT(&htim1);										// �رն�ʱ��1�ж�
      jy901_calibration();														// IMUУ׼
			HAL_Delay(500);
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_UART_Receive_IT(&huart3, &Rx3_Temp, 1);
			pidRest(pPidObject, 6);
			roll_rev = pitch_rev = yaw_rev = goback_rev = rightleft_rev = 0;		// <>
			openclose_rev = 1;
			Led1_Flag = 0;
		}
		
		u8 ledf_ctrl_en = *(data_buf + 9); // ʹ�ܿ����ֶ�
    u8 ledf = *(data_buf + 10);         // ״̬�ֶ�
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
  
	// �жϹ����֣�pidflag--PID����
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
	
  // �жϹ����֣�pidRateX--PID����
  case FUNC_SET_PIDRATEX:
  {
		
    pidParamUpdate(&pidRateX, 1, data_buf);
    break;
  }
	
	// �жϹ����֣�pidRateY--PID����
  case FUNC_SET_PIDRATEY:
  {
		
    pidParamUpdate(&pidRateY, 2, data_buf);
    break;
  }
	
	// �жϹ����֣�pidRateZ--PID����
  case FUNC_SET_PIDRATEZ:
  {
		
    pidParamUpdate(&pidRateZ, 3, data_buf);
    break;
  }
	
	// �жϹ����֣�pidRoll--PID����
  case FUNC_SET_PIDROLL:
  {
		
    pidParamUpdate(&pidRoll, 4, data_buf);
    break;
  }
	
	// �жϹ����֣�pidPitch--PID����
  case FUNC_SET_PIDPITCH:
  {
		
    pidParamUpdate(&pidPitch, 5, data_buf);
    break;
  }
	
	// �жϹ����֣�pidYaw--PID����
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


// ��ӡ����λ������
void Out_Send_Data(uint16_t data1,uint16_t data2, uint16_t data3)
{
  #define OutLEN        7
  uint8_t data_buffer[OutLEN] = {0};
  uint8_t i, checknum = 0;
  
  // ���ֽ���ǰ�����ֽ��ں�
  // data1
  data_buffer[0] = data1&0xFF;
  data_buffer[1] = (data1>>8)&0xFF;
	// data2
  data_buffer[2] = data2&0xFF;
  data_buffer[3] = (data2>>8)&0xFF;
  // data3
  data_buffer[4] = data3&0xFF;
  data_buffer[5] = (data3>>8)&0xFF;

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < OutLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[OutLEN-1] = checknum & 0xFF;
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



// ���մ��ڵ��ֽ����ݽ��ղ�����
void Upper_Data_Receive(u8 Rx_Temp)
{
  switch (RxFlag) {
  // ֡ͷ
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

  // ��ʶλ
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

  // ����λ����
  case 2:
  {
    // New_CMD_lengthΪ����֡���ֽ��� = ֡ͷ+��ʶλ+����+У��λ+֡β(5 bytes)+����λ
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

  // ��ȡ��ʣ��������ֶ�
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

