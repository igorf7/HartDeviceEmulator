/**
 * @file iwdg.c
 * @brief IWDG driver for STM32L05x
 */
#include "iwdg.h"

/**
 * @brief Enables or disables write access to IWDG_PR and IWDG_RLR registers.
 * @param IWDG_WriteAccess: new state of write access to IWDG_PR and IWDG_RLR registers.
 *     This parameter can be one of the following values:
 *         @arg IWDG_WriteAccess_Enable: Enable write access to IWDG_PR and IWDG_RLR registers
 *         @arg IWDG_WriteAccess_Disable: Disable write access to IWDG_PR and IWDG_RLR registers
 * @retval None
 */
void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess)
{
    IWDG->KR = IWDG_WriteAccess;
}

/**
 * @brief Sets IWDG Prescaler value.
 * @param IWDG_Prescaler: specifies the IWDG Prescaler value.
 *     This parameter can be one of the following values:
 *         @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *         @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *         @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *         @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *         @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *         @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *         @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 * @retval None
 */
void IWDG_SetPrescaler(uint8_t IWDG_Prescaler)
{
    IWDG->PR = IWDG_Prescaler;
}

/**
 * @brief Sets IWDG Reload value.
 * @param Reload: specifies the IWDG Reload value.
 *     This parameter must be a number between 0 and 0x0FFF.
 * @retval None
 */
void IWDG_SetReload(uint16_t Reload)
{
    IWDG->RLR = Reload;
}

/**
 * @brief Enables IWDG (write access to IWDG_PR and IWDG_RLR registers disabled).
 * @param None
 * @retval None
 */
void IWDG_Enable(void)
{
    IWDG->KR = KR_KEY_ENABLE;
}

/**
 * @brief
 * @param None
 * @retval None
*/
void setupIwdg(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    IWDG_SetPrescaler(IWDG_Prescaler_32);
    /* Set counter reload value to obtain 250ms IWDG TimeOut.
         Counter Reload Value = 250ms/IWDG counter clock period
                                                    = 250ms / (LSI/32)
                                                    = 0.25s / (LsiFreq/32)
                                                    = LsiFreq/(32 * 4)
                                                    = LsiFreq/128
     */
    IWDG_SetReload(0xFFF);
    IWDG_ReloadCounter();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
}

/**
 * @brief Reloads IWDG counter with value defined in the reload register
 *     (write access to IWDG_PR and IWDG_RLR registers disabled).
 * @param None
 * @retval None
 */
void IWDG_ReloadCounter(void)
{
    IWDG->KR = KR_KEY_RELOAD;
}
//eof
