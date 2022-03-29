/**
 * @file ports.c
 * @brief PORTS driver for STM32L05x
 *        (c)2018 I.Filippov
 */
#include "ports.h"
#include "rcc.h"

/**
 * @brief Sets the selected data port bits.
 * @param GPIOx: where x can be (A..H) to select the GPIO peripheral.
 * @param GPIO_Pin: specifies the port bits to be written.
 *                This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 * @note This functions uses GPIOx_BSRR register to allow atomic read/modify 
 *             accesses. In this way, there is no risk of an IRQ occurring between
 *             the read and the modify access.
 * @retval None
 */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BSRR = GPIO_Pin;
}

/**
 * @brief Clears the selected data port bits.
 * @param GPIOx: where x can be (A..H) to select the GPIO peripheral.
 * @param GPIO_Pin: specifies the port bits to be written.
 *        This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 * @note  This functions uses GPIOx_BSRR register to allow atomic read/modify 
 *        accesses. In this way, there is no risk of an IRQ occurring between
 *        the read and the modify access.
 * @retval None
 */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BRR = GPIO_Pin;
}

/**
*/
void initPorts(void)
{
    GPIOA->AFR[0] = 0;
    GPIOA->AFR[1] = 0;
//    GPIO_InitTypeDef GPIO_InitStructure;

//    RCC->IOPENR |= (RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN | 
//                                    RCC_IOPENR_GPIODEN | RCC_IOPENR_GPIOHEN);
//    
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//#ifdef DEBUG_VERSION
//    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_13 | GPIO_Pin_14);
//#endif
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_3;
//    GPIO_Init(GPIOH, &GPIO_InitStructure);
}
//eof
