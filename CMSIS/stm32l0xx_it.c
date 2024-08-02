/**
 * @file stm32l0xx_it.c
 * @brief This file contains handlers for the system interrupt
*/
#include "stm32l0xx_it.h"

#ifdef USE_DEBUG_LEDS
#include "ledctl.h"
#endif

/**
 * @brief This function handles NMI exception.
 * @param None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief This function handles Hard Fault exception.
 * @param None
 * @retval None
 */
void HardFault_Handler(void)
{
#ifdef USE_DEBUG_LEDS
    ledIndicateErr();
#endif
  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
 * @brief This function handles SVCall exception.
 * @param None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief This function handles Debug Monitor exception.
 * @param None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief This function handles PendSVC exception.
 * @param None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief This function handles SysTick Handler.
 * @param None
 * @retval None
 */
void SysTick_Handler(void)
{
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
