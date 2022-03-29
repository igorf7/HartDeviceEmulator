/****************************************************
 * lptimer.h
 * LP Timer driver for STM32L05x
****************************************************/
#ifndef __LPTIMER_H
#define __LPTIMER_H

#include "stm32l0xx.h"
#include "stddef.h"

/* LP Timer Driver API */
void deinitLP_Timer(void);
void initLP_Timer(void(*callback)(void));
#endif
//eof
