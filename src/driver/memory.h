/****************************************************
 * memory.h
 * Non volatile memory (NVM) driver
****************************************************/
#ifndef __MEMORY_H
#define __MEMORY_H

#include "stm32l0xx.h"

/* Data EEPROM Memory (STM32LO51) */
#define EEPROM_STARTADDRESS     (0x08080000)
#define EEPROM_PAGE_SIZE        (0x04) // bytes
#define CONFIG_DATA_STARTADDR   (EEPROM_STARTADDRESS + 4) // first 4 bytes is unique_id
#define CALIBR_DATA_STARTADDR   (0x08080100)
#define CONFIG_PAGE_SIZE        (CALIBR_DATA_STARTADDR - CONFIG_DATA_STARTADDR)
#define CALIBR_PAGE_SIZE        (0x80)
#define INFO_SIZE               (0x08)
#define CONFIG_INFO_STARTADDR   (CALIBR_DATA_STARTADDR - INFO_SIZE)
#define CALIBR_INFO_STARTADDR   (CALIBR_DATA_STARTADDR + CALIBR_PAGE_SIZE - INFO_SIZE)
#define APP_INFO_STARTADDR      (CALIBR_INFO_STARTADDR + INFO_SIZE)

/* Flash Programm Memory (STM32LO51) */
//#ifdef DEBUG_VERSION
#define SECTORS_IN_FLASH        8   // sector number is different for different device category
//#else
//#define SECTORS_IN_FLASH        16  // sector number is different for different device category
//#endif
#define PAGES_IN_SECTOR         32  // pages
#define FLASH_PAGE_SIZE         128 // bytes
#define FLASH_PAGES_NUMBER      (PAGES_IN_SECTOR * SECTORS_IN_FLASH)
#define FLASH_SECTOR_SIZE       (FLASH_PAGE_SIZE * PAGES_IN_SECTOR)
#define FLASH_TOTAL_SIZE        (FLASH_SECTOR_SIZE * SECTORS_IN_FLASH)
#define FLASH_START_ADDRESS     (0x08000000)
#define FLASH_END_ADDRESS       (FLASH_START_ADDRESS + FLASH_TOTAL_SIZE - 1)

#define SECTORS_FOR_LOADER      3
#define FLASH_LOADER_SIZE       (FLASH_SECTOR_SIZE * SECTORS_FOR_LOADER)    // 12 KiB for loader
#define FLASH_APP_SIZE          (FLASH_TOTAL_SIZE - FLASH_LOADER_SIZE)      // flash size for application
#define LOADER_PAGES_NUMBER     (PAGES_IN_SECTOR * SECTORS_FOR_LOADER)
#define APP_START_ADDRESS       (FLASH_START_ADDRESS + FLASH_LOADER_SIZE)
#define APP_PAGES_NUMBER        (FLASH_PAGES_NUMBER - LOADER_PAGES_NUMBER)
#define APP_END_ADDRESS         FLASH_END_ADDRESS

/* Flash program erase key1 and key2 */
#define FLASH_PEKEY1            ((uint32_t)0x89ABCDEF) /*!< Flash program erase key1 */
#define FLASH_PEKEY2            ((uint32_t)0x02030405) /*!< Flash program erase key: used with FLASH_PEKEY2
                                                            to unlock the write access to the FLASH_PECR register and
                                                            data EEPROM */
/* Flash program memory key1 and key2 */
#define FLASH_PRGKEY1           ((uint32_t)0x8C9DAEBF) /*!< Flash program memory key1 */
#define FLASH_PRGKEY2           ((uint32_t)0x13141516) /*!< Flash program memory key2: used with FLASH_PRGKEY2
                                                               to unlock the program memory */
                                                               
#define FLASH_ER_PRG_TIMEOUT    ((uint32_t)0x8000)

/** 
 * @brief  FLASH Status  
 */ 
typedef enum
{ 
    FLASH_BUSY = 1,
    FLASH_ERROR_WRP,
    FLASH_ERROR_PROGRAM,
    FLASH_COMPLETE,
    FLASH_TIMEOUT
}FLASH_Status;

FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

/* NVM Driver API */
void DATA_EEPROM_Unlock(void);
void DATA_EEPROM_Lock(void);
FLASH_Status EEPROM_EraseBlock(uint32_t Address, uint32_t num_bytes);
FLASH_Status DATA_EEPROM_EraseWord(uint32_t Address);
FLASH_Status EEPROM_WriteBlock(uint32_t addr, uint8_t* data, uint32_t len);
FLASH_Status DATA_EEPROM_FastProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_GetStatus(void);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_FastProgramWord(uint32_t Address, uint32_t Data);
#endif
//eof
