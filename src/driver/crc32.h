/****************************************************
 * crc32.h
****************************************************/
#ifndef __CRC32_H
#define __CRC32_H

#include "stm32l0xx.h"

/* RCC Driver API */
void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);
void CRC_DeInit(void);
#endif
//eof
