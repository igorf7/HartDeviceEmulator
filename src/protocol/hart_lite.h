/**
 * @brief HART protocol implementation for loader
 *        (c)2018 Alcont, Sarov
 */
#ifndef __HART_LITE_HLITE_H
#define __HART_LITE_HLITE_H

#include "stm32l0xx.h"
#include "convert.h"
#include "uart.h"
#include "modem5700.h"

/* HART Delimiter */
#define POLING_ADDR_FLAG            (uint8_t)0x00       // 0 Poling (1 Byte) Address
#define UNIQUE_ADDR_FLAG            (uint8_t)0x80       // 1 Unique (5 Byte) Address
#define FRAME_TYPE_ACK              (uint8_t)6          // ACK (Field Device to Master)

#define H_PREAMBLE_SIZE             (uint8_t)5          //
#define LONGFRAME_HEADER_SIZE       (uint8_t)9          // Header + checksum size in long frame
#define H_DATA_SIZE                 (uint16_t)255       // Max data size + checksum
#define HART_MAX_SIZE               (uint16_t)264       // HART packet max lenght without preamble

/* Flash and EEPROM errors */
typedef enum{
    no_error = 0,
    flash_erase_err,
    flash_write_err,
    eeprom_write_err
}LoadError_t;

/* Data transmission errors  (Status, byte[0]. TRANSMIT_ERR is set) */
typedef enum{
    // 0x01 - not defined
    BUFF_OVER_ERR = 0x02,
    // 0x04 - reserved
    CHECKSUM_ERR = 0x08,
    FRAMING_ERR = 0x10,
    OVERRUN_ERR = 0x20,
    PARITY_ERR = 0x40,
    TRANSMIT_ERR = 0x80
}TxError_t;

/* Command response codes (Status, byte[0]. TRANSMIT_ERR is not set) */
typedef enum{
    NO_CMD_ERR = 0,
    //NOT_DEFINED,                    // 1 - not defined
    INVALID_SELECTION = 2,
    PRM_TOO_LARGE,
    PRM_TOO_SMALL,
    TOO_FEW_BYTES,
    SPECIFIC_CMD_ERR,
    WRITE_PROTECT_MODE,
    UPDATE_FAILURE,
    PROC_PRM_TOO_LARGE,
    PROC_PRM_TOO_SMALL,
    WRONG_PRM_CODE,
    WRONG_UNIT_CODE,
    RANGE_OUT_OF_LIMITS,
    UPPER_RANGE_ERR,
    LOWER_RANGE_ERR,
    ACCESS_LIMITED,
    DEVICE_IS_BUSY = 32,
    UNSUPPORTED_CMD = 64,
}CmdError_t;

/* HART Protocol Commands */
typedef enum{

    /* HART common commands */
    COMMAND_42 = 42,                     // 42 - Performs Device Reset
    
    /* Non-Public commands */
    ERASE_APP_FLASH = 122,          // 122 - Non-Public command Erase flash memory of the application
    WRITE_FLASH_FIRST,              // 123 - Non-Public command for start writing to the device flash memory
    WRITE_FLASH_NEXT,               // 124 - Non-Public command Wrire next data chunk to the device flash memory
    START_APPLICATION,              // 125 - Non-Public command Start device application from loader
    SET_UNIQUE_ID,                  // 126 - Non-Public command Set device unique identificator
    
    /* HART specific commands */
    CHECK_HASH_SUM = 230,           // 230 - Check hash sum of the writen app memory
    CHECK_MEMORY,                   // 231 - Check device memory
    CLEAR_CONFIG                    // 232 - Clear configuration data in the EEPROM (Return to the factory settings)
}HartCommand_t;

/* Staus field */
typedef union{
    uint16_t both;
    uint8_t byte[2];    
}HartStatus_t;

/* Protocol State Structure */
typedef struct{
    HartStatus_t Status;
    bool isConfClearReq;
    bool isConfUpdateReq;
    bool isResetRequired;
}ProtocolState_t;

/* Structure of RX/TX buffer */
typedef struct{
    int16_t index;
    int16_t lenth;
    uint8_t data[HART_MAX_SIZE];
}DataBuffer_t;

/* Structure of address for HART long frame */
#pragma pack(push, 1)
typedef struct{
    uint16_t dev_type;          // Expanded Device Type code
    uint8_t dev_id_0;           // Slave device id most
    uint8_t dev_id_1;           // Slave device id secondary
    uint8_t dev_id_2;           // Slave device id least
}LongAddress_t;
#pragma pack(pop)

/* Structure of long HART-frame */
#pragma pack(push, 1)
typedef struct{
    uint8_t delimeter;          // Start symbol in long frame
    LongAddress_t address;      // Address in long frame
    uint8_t command;            // Command in long frame
    uint8_t data_len;           // Lenght of data field in long frame
    uint8_t data[H_DATA_SIZE];  // Data field in long frame
}HartLongFrame_t;
#pragma pack(pop)

/**
 * @brief execution of the command depending on its code
 * @param cmd - opcode (command)
 * @param rx_data - data field of the received packet
 * @param tx_data - data field to filling and trasmit
 * @retval size of the tx_data
 */
static uint8_t ExecuteCommand(uint16_t cmd, uint8_t* rxData, uint8_t* txData);


/*******************************************************************/
/************************* HART driver API *************************/

/**
 * @brief Initialize of protocol variables
 * @param None
 * @retval None
 */
void initHartProtocol(void);

/**
 * @brief Returns a preamble size
 * @param None
 * @retval preamble size
 */
uint8_t getPreambleSize(void);

/**
 * @brief Parsing of received data packet
 * @param packet - received data packet
 * @retval pointer to the status word
 */
ProtocolState_t* packetParsing(uint8_t* packet);

/**
 * @brief Sets the buffer address for the return of the
 *        data packet from the protocol to the application
 * @param Buffer address
 * @retval None
 */
void setBufferAddress(DataBuffer_t* dataBuffer);

/**
 * @brief Update status field in the HART packet
 * @param New status
 * @retval None
 */
void setTransferError(HartStatus_t* newstate);

/**
 * @brief sends a data packet using the hart protocol
 * @param data packet
 * @param lenght of the data packet
 */
void HartSendPacket(uint8_t* packet, uint16_t len);
#endif
//eof
