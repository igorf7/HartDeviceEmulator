/****************************************************
 * ports.h
****************************************************/
#ifndef __PORTS_H
#define __PORTS_H

#include "stm32l0xx.h"
#include "stdint.h"

/** @defgroup Bit_SET_and_Bit_RESET_enumeration
 */
typedef enum
{ Bit_RESET = 0,
    Bit_SET
}BitAction;

/* Ports Driver API */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void initPorts(void);

#endif
//eof
