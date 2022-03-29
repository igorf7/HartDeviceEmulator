/**
 * @file memory.c
 * @brief NVM (Non Volatile Memory) driver for STM32L05x
 */
#include "memory.h"

/************************************ Common section *************************************/
/*****************************************************************************************/

/**
 * @brief  Returns the FLASH Status.
 * @param  None
 * @retval FLASH Status: The returned value can be: 
 *         FLASH_BUSY, FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP or FLASH_COMPLETE.
 */
FLASH_Status FLASH_GetStatus(void)
{
    FLASH_Status FLASHstatus = FLASH_COMPLETE;
    
    if ((FLASH->SR & FLASH_SR_BSY) == FLASH_SR_BSY) {
        FLASHstatus = FLASH_BUSY;
    }
    else {    
        if ((FLASH->SR & (uint32_t)FLASH_SR_WRPERR)!= (uint32_t)0x00) { 
            FLASHstatus = FLASH_ERROR_WRP;
        }
        else {
            if ((FLASH->SR & (uint32_t)0x1E00) != (uint32_t)0x00) {
                FLASHstatus = FLASH_ERROR_PROGRAM; 
            }
            else {
                FLASHstatus = FLASH_COMPLETE;
            }
        }
    }
    /* Return the FLASH Status */
    return FLASHstatus;
}

/**
 * @brief  Waits for a FLASH operation to complete or a TIMEOUT to occur.
 * @param  Timeout: FLASH programming Timeout.
 * @retval FLASH Status: The returned value can be: FLASH_BUSY, 
 *                 FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout)
{ 
    __IO FLASH_Status status = FLASH_COMPLETE;
     
    /* Check for the FLASH Status */
    status = FLASH_GetStatus();
    
    /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
    while((status == FLASH_BUSY) && (Timeout != 0x00))
    {
        status = FLASH_GetStatus();
        Timeout--;
    }
    
    if (Timeout == 0x00) {
        status = FLASH_TIMEOUT;
    }
    /* Return the operation status */
    return status;
}

/********************************** Data EEPROM section **********************************/
/*****************************************************************************************/

/**
 * @brief  Unlocks the data memory and FLASH_PECR register access.
 * @param  None
 * @retval None
 */
void DATA_EEPROM_Unlock(void)
{
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != RESET) {    
        /* Unlocking the Data memory and FLASH_PECR register access*/
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
    }
}

/**
 * @brief  Locks the Data memory and FLASH_PECR register access.
 * @param  None
 * @retval None
 */
void DATA_EEPROM_Lock(void)
{
    /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
    FLASH->PECR |= FLASH_PECR_PELOCK;
}

/**
 * @brief  Erase a block in data memory.
 * @param  Address: specifies the start address.
 * @note   num must be a multiple of 4
 * @retval FLASH Status: The returned value can be: 
 *                 FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status EEPROM_EraseBlock(uint32_t Address, uint32_t num_bytes)
{
    uint32_t i;
    FLASH_Status state;
        
    DATA_EEPROM_Unlock();
    for (i = 0; i < num_bytes; i += 4) {
        state = DATA_EEPROM_EraseWord(Address+i);
        if (state != FLASH_COMPLETE) {
            DATA_EEPROM_Lock();
            return state;
        }
    }
    DATA_EEPROM_Lock();
    return state;
}

/**
 * @brief Erase a word in data memory.
 * @param Address: specifies the address to be erased.
 * @note  For STM32L1XX_MD, A data memory word is erased in the data memory only 
 *                 if the address to load is the start address of a word (multiple of a word).
 * @note  To correctly run this function, the DATA_EEPROM_Unlock() function
 *                 must be called before.
 *                 Call the DATA_EEPROM_Lock() to he data EEPROM access
 *                 and Flash program erase control register access(recommended to protect 
 *                 the DATA_EEPROM against possible unwanted operation).
 * @retval FLASH Status: The returned value can be: 
 *                 FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status DATA_EEPROM_EraseWord(uint32_t Address)
{
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    if (status == FLASH_COMPLETE) {
        /* Write "00000000h" to valid address in the data memory" */
        *(__IO uint32_t *) Address = 0x00000000;
    }
     
    /* Return the erase status */
    return status;
}

/**
 * @brief  Write data block to EEPROM memory
 * @param  addr - start address to write
 * @param  Data - address of data block
 * @param  num_bytes - number of bytes
 * @note   data must be aligned by 4 bytes
 * @retval status of operation
*/
FLASH_Status EEPROM_WriteBlock(uint32_t addr, uint8_t* data, uint32_t len)
{
	FLASH_Status state;
	uint32_t i;
    uint32_t shift = sizeof(uint32_t);
	
	DATA_EEPROM_Unlock();
    for (i = 0; i < len; i += shift) {
        state = DATA_EEPROM_EraseWord(addr);
        if (state != FLASH_COMPLETE) {
            break; // erase error
        }
        state = DATA_EEPROM_FastProgramWord(addr, *(uint32_t*)data);
        data += shift; // set next data address
        if (state != FLASH_COMPLETE) {
            break; // write error
        }
        addr += shift; // set next eeprom address
    }
	DATA_EEPROM_Lock();
	return state;
}

/**
 * @brief  Programs a word at a specified address in data memory.
 * @note   To correctly run this function, the DATA_EEPROM_Unlock() function must be called before.
 *         Call the DATA_EEPROM_Lock() to the data EEPROM access
 *         and Flash program erase control register access(recommended to protect
 *         the DATA_EEPROM against possible unwanted operation).
 * @param  Address: specifies the address to be written.
 * @param  Data: specifies the data to be written.
 * @note   This function assumes that the is data word is already erased.
 * @retval FLASH Status: The returned value can be:
 *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status DATA_EEPROM_FastProgramWord(uint32_t Address, uint32_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
    
    if (status == FLASH_COMPLETE) {
        /* Clear the FTDW bit */
        FLASH->PECR &= (uint32_t)(~((uint32_t)FLASH_PECR_FIX));
    
        /* If the previous operation is completed, proceed to program the new data */        
        *(__IO uint32_t *)Address = Data;
        
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);             
    }
    /* Return the Write Status */
    return status;
}

/************************************* Flash section *************************************/
/*****************************************************************************************/

/**
 * @brief  Unlocks the FLASH control register and program memory access.
 * @param  None
 * @retval None
 */
void FLASH_Unlock(void)
{
    if ((FLASH->PECR & FLASH_PECR_PRGLOCK) != RESET) {
        /* Unlocking the data memory and FLASH_PECR register access */
        DATA_EEPROM_Unlock();
  
        /* Unlocking the program memory access */
        FLASH->PRGKEYR = FLASH_PRGKEY1;
        FLASH->PRGKEYR = FLASH_PRGKEY2;  
    }
}

/**
 * @brief  Locks the Program memory access.
 * @param  None
 * @retval None
 */
void FLASH_Lock(void)
{
    /* Set the PRGLOCK Bit to lock the program memory access */
    FLASH->PECR |= FLASH_PECR_PRGLOCK;
}

/**
 * @brief  Erases a specified page in program memory.
 * @note   To correctly run this function, the FLASH_Unlock() function
 *         must be called before.
 *         Call the FLASH_Lock() to disable the flash memory access 
 *         (recommended to protect the FLASH memory against possible unwanted operation)
 * @param  Page_Address: The page address in program memory to be erased.
 * @note   A Page is erased in the Program memory only if the address to load 
 *         is the start address of a page (multiple of 128 bytes).
 * @retval FLASH Status: The returned value can be: 
 *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
 */
FLASH_Status FLASH_ErasePage(uint32_t Page_Address)
{
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
    if (status == FLASH_COMPLETE) {
        /* If the previous operation is completed, proceed to erase the page */

        /* Set the ERASE bit */
        FLASH->PECR |= FLASH_PECR_ERASE;

        /* Set PROG bit */
        FLASH->PECR |= FLASH_PECR_PROG;
  
        /* Write 00000000h to the first word of the program page to erase */
        *(__IO uint32_t *)Page_Address = 0x00000000;
 
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);

        /* If the erase operation is completed, disable the ERASE and PROG bits */
        FLASH->PECR &= (uint32_t)(~FLASH_PECR_PROG);
        FLASH->PECR &= (uint32_t)(~FLASH_PECR_ERASE);   
  }     
  /* Return the Erase Status */
  return status;
}

/**
 * @brief  Programs a word at a specified address in program memory.
 * @note   To correctly run this function, the FLASH_Unlock() function
 *         must be called before.
 *         Call the FLASH_Lock() to disable the flash memory access
 *         (recommended to protect the FLASH memory against possible unwanted operation).
 * @param  Address: specifies the address to be written.
 * @param  Data: specifies the data to be written.
 * @retval FLASH Status: The returned value can be:  
 *   FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
 */
FLASH_Status FLASH_FastProgramWord(uint32_t Address, uint32_t Data)
{
    FLASH_Status status = FLASH_COMPLETE;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);
  
    if (status == FLASH_COMPLETE) {
        /* If the previous operation is completed, proceed to program the new  word */  
        *(__IO uint32_t *)Address = Data;
        
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(FLASH_ER_PRG_TIMEOUT);       
    }
    /* Return the Write Status */
    return status;
}
//eof
