/****************************************************
 * rcc.h
****************************************************/
#ifndef __RCC_H
#define __RCC_H

#include "stm32l0xx.h"

#define RCC_FLAG_IWDGRST          ((uint8_t)0x5D)
#define FLAG_MASK                 ((uint8_t)0x1F)

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFA5E2) == 0x00) && ((PERIPH) != 0x00))

typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
}RCC_ClocksTypeDef;

/* RCC Driver API */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void SystemClockConfig(void);
#endif
//eof
