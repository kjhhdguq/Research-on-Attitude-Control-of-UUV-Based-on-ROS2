#ifndef __EEPROM_H_
#define __EEPROM_H_


#include "main.h"
#include "i2c.h"

#define READ_CMD				1
#define WRITE_CMD				0

#define DEV_ADDR				0xA0					//??????

#define PAGE_NUM			512						//??
#define PAGE_SIZE			64						//????(??)
#define CAPACITY_SIZE		(PAGE_NUM * PAGE_SIZE)	//???(??)
#define ADDR_BYTE_NUM		2						//??????

void AT24C256_WriteByte(uint16_t Addr, uint16_t Num, uint8_t *Data);
void AT24C256_ReadByte(uint16_t Addr, uint16_t Num, uint8_t *Data);

#endif
