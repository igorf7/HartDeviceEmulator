/**
 * @file lptimer.c
 * @brief LP Timer driver for STM32L05x
 *                (c)2018 Alcont, Sarov
 */
#include "lptimer.h"

void(*lpTimHandle)(void) = NULL;

/**
 * @brief  Deinitiolize the LP Timer.
 * @param  None
 * @retval None
 */
void deinitLP_Timer(void){
    RCC->APB1ENR &= ~RCC_APB1ENR_LPTIM1EN;
}
    
/**
 * @brief  Initiolize the LP Timer.
 * @param  Handle function for Carrier Detect event
 * @retval None
 */
void initLP_Timer(void(*callback)(void))
{
    lpTimHandle = callback;
    
    RCC->APB1ENR |= RCC_APB1ENR_LPTIM1EN;
    
    LPTIM1->CFGR |= (LPTIM_CFGR_PRESC_1|LPTIM_CFGR_PRESC_2);
    LPTIM1->IER |= LPTIM_IER_ARRMIE;
    LPTIM1->CR |= LPTIM_CR_ENABLE;
    LPTIM1->ARR = 65535;
    LPTIM1->CR |= LPTIM_CR_CNTSTRT; /* start the counter in continuous */
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 3);
}

/**
 * @brief  This function handles LP Timer interrupt request.
 * @param  None
 * @retval None
 */
void LPTIM1_IRQHandler(void)
{
    if ((LPTIM1->ISR & LPTIM_ISR_ARRM) != 0) // Check ARR match
    {
        LPTIM1->IER = 0;
        LPTIM1->CR = 0;        
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF; // Clear ARR match flag
        if( lpTimHandle != NULL ){
            lpTimHandle();  // notifies application about LPTIM event
        }
    }
}
//eof
