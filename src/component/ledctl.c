/**
 * @file ledctl.c
 * @brief LED driver for v318hart project
 *        (c)2018 I.Filippov
 */
#include "ledctl.h"
#include "rcc.h"

/**
 * @brief Initializes the LED port and pins.
 * @param None
 * @retval None
 */
void ledInit(void)
{
    /* Enable the peripheral clock of GPIOA and GPIOB */
    RCC->IOPENR |= RCC_IOPENR_LEDS;
    /* PB3 - Green Led */
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE3)) | GPIO_MODER_MODE3_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED3; // speed very high
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE4)) | GPIO_MODER_MODE4_0;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED4; // speed very high
}

/**
 * @brief Switches LED on.
 * @param Led color.
 * @retval None
 */
inline void ledOn(BoardLed_t led)
{
    LED_PORT->BSRR = led;
}

/**
 * @brief Switches LED off.
 * @param Led color.
 * @retval None
 */
inline void ledOff(BoardLed_t led)
{
    LED_PORT->BRR = led;
}

/**
 * @brief Toggles LED state.
 * @param Led color.
 * @retval None
 */
inline void ledToggle(BoardLed_t led)
{
    LED_PORT->ODR ^= led;
}

/**
*/
void ledIndicateErr(void)
{
    __IO uint32_t delay;

    while(1) {
        ledOn(LED_GREEN);
        ledOff(LED_RED);
        delay = 0x3FFFF;
        while(--delay > 0);
        ledOn(LED_RED);
        ledOff(LED_GREEN);
        delay = 0x3FFFF;
        while(--delay > 0);
    }
}
