#include "eeprom.h"




// 写指定个字节数据（阻塞式发送）
void AT24C256_WriteByte(uint16_t Addr, uint16_t Num, uint8_t *Data)
{
	HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR | WRITE_CMD, Addr, 2, Data, Num, 1000);
}


// 读指定个字节数据（阻塞式读取；在初始化读取，timeout设置为1000ms）
void AT24C256_ReadByte(uint16_t Addr, uint16_t Num, uint8_t *Data)
{
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR | READ_CMD, Addr, 2, Data, Num, 1000);
}