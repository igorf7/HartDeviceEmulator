/**
 * @brief convert.h - header file for convert.c
 *        (c)2018 Sarov
 */
#ifndef __CONVERT_H
#define __CONVERT_H

#include "stm32l0xx.h"
#include "stdbool.h"

void FloatToByteArray(float f, uint8_t* array);
float ByteArrayToFloat(uint8_t* array, uint16_t pos);
void memcpyRevers(uint8_t* dst, uint8_t* src, uint32_t size);
#endif
//eof
