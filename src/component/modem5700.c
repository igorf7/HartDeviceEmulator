/**
 * @file modem5700.c
 * @brief This file provides firmware functions
 *        to communication with HART-modem AD5700
 *        (c)2018 I.Filippov
 */
#include "modem5700.h"

void(*carrierDetect)(void) = NULL;

/**
 * @brief Initialize MCU ports for comunication with HART-modem
 * @param Handle function for Carrier Detect event
 * @retval None
 */
void initHartModem(void(*callback)(void))
{
    carrierDetect = callback;

    /* Enable the peripheral clock of MODEM_PORT */
    RCC->IOPENR |= MODEM_PORT_CLK;
        
    /* Setup RESET & RTS pins */
    // PA11 out mode, PA12 out mode
    MODEM_PORT->MODER = (MODEM_PORT->MODER & ~(GPIO_MODER_MODE11))|(GPIO_MODER_MODE11_0);
    MODEM_PORT->MODER = (MODEM_PORT->MODER & ~(GPIO_MODER_MODE12))|(GPIO_MODER_MODE12_0);
    MODEM_PORT->OTYPER &= ~(GPIO_OTYPER_OT_11 | GPIO_OTYPER_OT_12); // push-pull
    // PA11 pull-up, PA12 pull-up
    MODEM_PORT->PUPDR = (MODEM_PORT->PUPDR & ~(GPIO_PUPDR_PUPD11))|(GPIO_PUPDR_PUPD11_0);
    MODEM_PORT->PUPDR = (MODEM_PORT->PUPDR & ~(GPIO_PUPDR_PUPD12))|(GPIO_PUPDR_PUPD12_0);
    // PA11, PA12 speed very high
    MODEM_PORT->OSPEEDR |= (GPIO_OSPEEDER_OSPEED11 | GPIO_OSPEEDER_OSPEED12);
    // RTS off
    MODEM_PORT->BSRR = MODEM_RTS_PIN;    
    
//#ifdef DEBUG_VERSION
//    /* Enable CD interrupts in debug configuration */
//    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
//    // Setup CD pin
//    MODEM_PORT->MODER = (MODEM_PORT->MODER & ~(GPIO_MODER_MODE8));
//    // CD pin as interrupt source input
//    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~SYSCFG_EXTICR3_EXTI8) | SYSCFG_EXTICR3_EXTI8_PA;
//    EXTI->IMR |= MODEM_CD_EXTI_LINE;
//    EXTI->RTSR |= MODEM_CD_EXTI_LINE;
//    EXTI->FTSR |= MODEM_CD_EXTI_LINE;
//    // Enable Interrupt on EXTI4_15
//    NVIC_EnableIRQ(EXTI4_15_IRQn);
//    NVIC_SetPriority(EXTI4_15_IRQn, 0);
//#endif
    resetHartModem();
}

/**
 * @brief Reset hart-modem via reset pin
 * @param None
 * @retval None
 */
void resetHartModem(void)
{
    MODEM_PORT->BRR = MODEM_RESET_PIN;
    MODEM_PORT->BSRR = MODEM_RESET_PIN;
}

/**
 * @brief Checks hart-modem state
 * @param None
 * @retval Modem state
 */
bool isHartModemSet(void)
{
    return (MODEM_PORT->ODR & MODEM_RESET_PIN);
}

/**
 * @brief Sets hart-modem RTS high (enable modulation)
 * @param None
 * @retval None
 */
void SetRTS_High(void)
{
    MODEM_PORT->BSRR = MODEM_RTS_PIN;
}

/**
 * @brief Sets hart-modem RTS low (enable demodulation)
 * @param None
 * @retval None
 */
void SetRTS_Low(void)
{
    MODEM_PORT->BRR = MODEM_RTS_PIN;
}

/**
 * @brief Handle interruption from hart-modem CD pin
 * @param None
 * @retval None
 */
void EXTI4_15_IRQHandler(void)
{
    if ((EXTI->PR & MODEM_CD_EXTI_LINE) && (EXTI->IMR & MODEM_CD_EXTI_LINE))
    {
        EXTI->PR = MODEM_CD_EXTI_LINE; 
                
        if (carrierDetect != NULL) {
            carrierDetect(); // notifies application about carrier detect event
        }
    }
}
//eof
