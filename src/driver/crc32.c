/**
 * @file crc32.c
 * @brief CRC32 driver for STM32L05x
 */
#include "crc32.h"

/**
 * @brief Deinitializes CRC peripheral registers to their default reset values.
 * @param None
 * @retval None
 */
void CRC_DeInit(void)
{
    /* Set DR register to reset value */
    CRC->DR = 0xFFFFFFFF;
    /* Reset IDR register */
    CRC->IDR = 0x00;
    /* Set INIT register to reset value */
    CRC->INIT = 0xFFFFFFFF;
    /* Reset the CRC calculation unit */
    CRC->CR = CRC_CR_RESET;
}

/**
 * @brief Resets the CRC Data register (DR).
 * @param None
 * @retval None
 */
void CRC_ResetDR(void)
{
    /* Reset CRC generator */
    CRC->CR = CRC_CR_RESET;
}

/**
 * @brief Computes the 32-bit CRC of a given data word(32-bit).
 * @param Data: data word(32-bit) to compute its CRC.
 * @retval 32-bit CRC
 */
uint32_t CRC_CalcCRC(uint32_t Data)
{
    CRC->DR = Data;
    
    return (CRC->DR);
}

/**
 * @brief Computes the 32-bit CRC of a given buffer of data word(32-bit).
 * @param pBuffer: pointer to the buffer containing the data to be computed.
 * @param BufferLength: length of the buffer to be computed					
 * @retval 32-bit CRC
 */
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength)
{
    uint32_t index = 0;
    
    for (index = 0; index < BufferLength; index++)
    {
        CRC->DR = pBuffer[index];
    }
    return (CRC->DR);
}

/**
 * @brief Returns the current CRC value.
 * @param None
 * @retval 32-bit CRC
 */
uint32_t CRC_GetCRC(void)
{
    return (CRC->DR);
}

/**
 * @brief Stores a 8-bit data in the Independent Data(ID) register.
 * @param IDValue: 8-bit value to be stored in the ID register 					
 * @retval None
 */
void CRC_SetIDRegister(uint8_t IDValue)
{
    CRC->IDR = IDValue;
}

/**
 * @brief Returns the 8-bit data stored in the Independent Data(ID) register.
 * @param None
 * @retval 8-bit value of the ID register 
 */
uint8_t CRC_GetIDRegister(void)
{
    return (CRC->IDR);
}
//eof

