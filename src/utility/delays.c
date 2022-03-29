#include "delays.h"

static RCC_ClocksTypeDef RCC_ClocksStructure;

void delay_us(uint32_t uSec)
{
	__IO uint32_t us;
	
	RCC_GetClocksFreq(&RCC_ClocksStructure);
	us = RCC_ClocksStructure.HCLK_Frequency / RCC_ClocksStructure.HCLK_Frequency * uSec;
	while(--us > 0);
}

void delay_ms(uint64_t mSec)
{
	delay_us(mSec * 1000);
}
