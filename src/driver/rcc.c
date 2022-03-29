/**
 * @file rcc.c
 * @brief RCC driver for STM32L05x
 */
#include "rcc.h"

#if !defined (HSE_VALUE) 
    #define HSE_VALUE        ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
#endif

#if !defined (MSI_VALUE)
    #define MSI_VALUE        ((uint32_t)2000000) /*!< Value of the Internal oscillator in Hz*/
#endif
     
#if !defined (HSI_VALUE)
    #define HSI_VALUE        ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif

/* Time-out values */
#define HSI_TIMEOUT_VALUE               ((uint32_t)100)     /* 100 ms */
#define PLL_TIMEOUT_VALUE               ((uint32_t)100)     /* 100 ms */
#define CLOCKSWITCH_TIMEOUT_VALUE       ((uint32_t)5000)    /* 5 s        */

static __IO uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __IO uint8_t PLLMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};

/**
 * @brief Checks whether the specified RCC flag is set or not.
 * @param RCC_FLAG: specifies the flag to check.
 * @retval The new state of RCC_FLAG (SET or RESET).
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
    uint32_t tmp = 0;
    uint32_t statusreg = 0;
    FlagStatus bitstatus = RESET;

    /* Get the RCC register index */
    tmp = RCC_FLAG >> 5;

    if (tmp == 1)               /* The flag to check is in CR register */
    {
        statusreg = RCC->CR;
    }
    else                        /* The flag to check is in CSR register (tmp == 2) */
    {
        statusreg = RCC->CSR;
    }

    /* Get the flag position */
    tmp = RCC_FLAG & FLAG_MASK;

    if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    /* Return the flag status */
    return bitstatus;
}

/**
 * @brief Clears the RCC reset flags.
 *        The reset flags are: RCC_FLAG_OBLRST, RCC_FLAG_PINRST, RCC_FLAG_PORRST, 
 *        RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST.
 * @param None
 * @retval None
 */
void RCC_ClearFlag(void)
{
    /* Set RMVF bit to clear the reset flags */
    RCC->CSR |= RCC_CSR_RMVF;
}

/**
 * @brief Returns the frequencies of the System, AHB and APB busses clocks.
 * @param RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold 
 *        the clocks frequencies. 
 *         
 * @note This function can be used by the user application to compute the 
 *       baudrate for the communication peripherals or configure other parameters.
 * @note Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
 *       must be called to update the structure's field. Otherwise, any
 *       configuration based on this function will be incorrect.
 * @retval None
 */
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
    uint32_t tmp = 0, pllmul = 0, plldiv = 0, pllsource = 0, presc = 0, msirange = 0;

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;
    
    switch (tmp)
    {
        case 0x00:  /* MSI used as system clock */
            msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
            RCC_Clocks->SYSCLK_Frequency = (32768 * (1 << (msirange + 1)));
            break;
        case 0x04:  /* HSI used as system clock */
            if (RCC->CR & RCC_CR_HSIDIVEN)
                RCC_Clocks->SYSCLK_Frequency = HSI_VALUE/4;
            else
                RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
            break;
        case 0x08:  /* HSE used as system clock */
            RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
            break;
        case 0x0C:  /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
            plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
            pllmul = PLLMulTable[(pllmul >> 18)];
            plldiv = (plldiv >> 22) + 1;
            
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

            if (pllsource == 0x00)
            {
                /* HSI oscillator clock selected as PLL clock source */
                if (RCC->CR & RCC_CR_HSIDIVEN)
                    RCC_Clocks->SYSCLK_Frequency = (((HSI_VALUE/4) * pllmul) / plldiv);
                else
                    RCC_Clocks->SYSCLK_Frequency = (((HSI_VALUE) * pllmul) / plldiv);
            }
            else
            {
                /* HSE selected as PLL clock source */
                RCC_Clocks->SYSCLK_Frequency = (((HSE_VALUE) * pllmul) / plldiv);
            }
            break;
        default: /* MSI used as system clock */
            msirange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13;
            RCC_Clocks->SYSCLK_Frequency = (32768 * (1 << (msirange + 1)));
            break;
    }
    /* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
    /* Get HCLK prescaler */
    tmp = RCC->CFGR & RCC_CFGR_HPRE;
    tmp = tmp >> 4;
    presc = AHBPrescTable[tmp]; 
    /* HCLK clock frequency */
    RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

    /* Get PCLK1 prescaler */
    tmp = RCC->CFGR & RCC_CFGR_PPRE1;
    tmp = (tmp >> 8) + 4;
    presc = AHBPrescTable[tmp];
    /* PCLK1 clock frequency */
    RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

    /* Get PCLK2 prescaler */
    tmp = RCC->CFGR & RCC_CFGR_PPRE2;
    tmp = (tmp >> 11) + 4;
    presc = AHBPrescTable[tmp];
    /* PCLK2 clock frequency */
    RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
}

/**
 * @brief This function configures the system clock as follows:
 *        SYSCLK_Frequency  = HSI 16 MHz
 *        HCLK_Frequency    = SYSCLK_Frequency / 2
 *        PCLK1_Frequency   = HCLK_Frequency / 4
 *        PCLK2_Frequency   = HCLK_Frequency / 4
 * @param None
 * @retval None
 */
void SystemClockConfig(void)
{
    /* Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    
    /* Select voltage scale 2 */
    PWR->CR = (PWR->CR & ~(PWR_CR_VOS)) | PWR_CR_VOS_1;
    
    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    
    /* Wait for HSI ready flag and HSIDIV flag */
    while ((RCC->CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY)
    {
        __NOP(); // The watcdog must reset MCU here, if an error occurs
    }
    /* Select HSI as system clock */
    RCC->CFGR |= (RCC_CFGR_HPRE_3 | RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_2 |
                    RCC_CFGR_PPRE2_0 | RCC_CFGR_PPRE2_2 | RCC_CFGR_SW_HSI);
    
    /* Wait for clock switched on HSI */
    while ((RCC->CFGR & RCC_CFGR_SWS_HSI) == 0)
    {
        __NOP(); // The watcdog must reset MCU here, if an error occurs
    }
}
//eof
