#ifndef __DELAYS_H
#define __DELAYS_H

#include "stm32l0xx.h"
#include "rcc.h"

void delay_us(uint32_t uSec);

void delay_ms(uint64_t mSec);

#endif
