/*
	crc.h file
	Header file for crc.c
*/
#ifndef __CRC_H
#define __CRC_H

#include "global.h"

#define   POLINOME_16        0xA001  //

uint16_t calcCrc16(const uint8_t *data, uint16_t len);
uint32_t calcCRC32(const uint32_t word, uint32_t crc);
uint32_t calcBufCRC32(const uint32_t *buf, uint32_t len);
#endif /* __CRC_H */
//eof
