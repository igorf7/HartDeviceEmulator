/**
 * @file spi.c
 * @brief SPI driver for v318hart project
 *        (c)2018 I.Filippov
 */
#include "spi.h"

/**
 * @brief Sets CS SPI high
 * @param None
 * @retval None
 */
void clearChipSelect(void)
{
    GPIOA->BSRR = CS_PIN;
}

/**
 * @brief Sets CS SPI low
 * @param None
 * @retval None
 */
void setChipSelect(void)
{
    GPIOA->BRR = CS_PIN;
}

/**
 * @brief Checks is SPI busy or not
 * @param None
 * @retval None
 */
bool isSpiBusy(SPI_TypeDef* SPIx)
{
    return (SPIx->SR & SPI_SR_BSY);
}

/**
 * @brief  Initialize SPI module.
 * @param  None 
 * @retval None 
 */
void initSpi1(void)
{
    /* Configure SPI pins */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    // Select AF mode on PA5, PA6, PA7
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7))
                          | (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
    
    // AF0 for SPI1 signals
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xF<<(4*5)) | (0xF<<(4*6)) | ((uint32_t)0xF<<(4*7))));
    
    // PA4 output mode
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4))|(GPIO_MODER_MODE4_0);
    // PA4 push-pull
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4;
    // PA4 pull-up
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD4))|(GPIO_PUPDR_PUPD4_0);
    // PA4 speed very high
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED4;
    clearChipSelect();
    
    /* Configure SPI */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 = SPI_CR1_CPHA | SPI_CR1_MSTR | SPI_CR1_DFF | SPI_CR1_SSM | 
                SPI_CR1_SSI | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE; // Master, BR = clk/2, 16 bit
    SPI1->CR1 |= SPI_CR1_SPE;   // Enable SPI1    
}

/**
 * @brief  Returns the most recent received data by the SPIx. 
 * @param  SPIx: To select the SPIx peripheral.
 * @retval The value of the received data.
 */
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx)
{
  /* Return the data in the DR register */
  return SPIx->DR;
}

/**
 * @brief  Transmits a Data through the SPIx.
 * @param  SPIx: To select the SPIx peripheral.
 * @param  Data: Data to be transmitted.
 * @retval None
 */
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{  
    // Test Tx empty
    while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE) {};
    // Write in the DR register the data to be sent
    SPIx->DR = Data;
}
//eof
