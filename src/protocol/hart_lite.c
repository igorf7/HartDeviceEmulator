/**
 * @file hart_lite.c
 * @brief HART protocol implementation - lite version
 *        (c)2018 Alcont, Sarov
 */
#include "hart_lite.h"
#include "memory.h"
#include "crc32.h"
#include "lptimer.h"
#include "main.h"

#define DEVICE_UNIQUE_ID    (uint32_t)0xBC614E

static HartLongFrame_t* rxLongFrame;
static HartLongFrame_t* txLongFrame;

static DataBuffer_t* txPacket;

static ProtocolState_t hartState;

static FLASH_Status FlashState;

static uint64_t rcvLongAddress;

static uint32_t flashAppSize,
                fwHashSum = 0,
                realAppSize = 0;

static uint16_t rxDataLen,
                burstModeLong = 0;;

static uint8_t
       startSymbol;

static bool isJumpRequested = false;

static uint8_t txBuffer[HART_MAX_SIZE];


/**
 * @brief Calculates a checksum
 * @param packet - data packet
 * @param len - packet lenght
 */
static uint8_t calcChecksum(uint8_t *packet, uint16_t len)
{
    uint8_t checksum = packet[0];
    
    for(int i = 1; i < len; i++){
        checksum ^= packet[i];
    }
    return checksum;
}

/**
 * @brief 
 * @param 
 * @param 
 */
static uint32_t checkMemoryRegion(uint32_t addr, uint32_t size)
{
    RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable CRC32
    CRC_ResetDR();
    uint32_t check_sum = CRC_CalcBlockCRC((uint32_t*)addr, size);
    RCC->AHBENR &= ~RCC_AHBENR_CRCEN; // disable CRC32
    
    return check_sum;
}

/**
 * @brief Update status field in the HART packet
 * @param New status
 * @retval None
 */
void setTransferError(HartStatus_t* newstate)
{
    hartState.Status.byte[0] = newstate->byte[0];
}

/**
 * @brief Initialize of protocol variables
 * @param Buffer address
 * @retval None
 */
void initHartProtocol(void)
{
    startSymbol = FRAME_TYPE_ACK;
    burstModeLong = 0;
    hartState.Status.both = 0;
    hartState.isConfClearReq = false;
    hartState.isConfUpdateReq = false;
    hartState.isResetRequired = false;
    hartState.Status.both = 0;
    flashAppSize = (uint32_t)FLASH_APP_SIZE;
}

/**
 * @brief Sets the buffer address for the return of the
 *        data packet from the protocol to the application
 * @param Buffer address
 * @retval None
 */
void setBufferAddress(DataBuffer_t* buff_addr)
{
    txPacket = buff_addr;
}

/**
 * @brief sends a data packet using the HART protocol
 * @param data packet
 * @param lenght of the data packet
 */
void HartSendPacket(uint8_t* packet, uint16_t len)
{
    uint32_t i;
    
    for(i = 0; i < H_PREAMBLE_SIZE; i++){
        txPacket->data[i] = 0xFF; // fill preamble field
    }
    memcpy(txPacket->data+i, packet, len); // fill PDU field
    txPacket->index = 0;
    txPacket->lenth = len + H_PREAMBLE_SIZE;
    
    SetRTS_Low();   // set hart-modem in TX mode
    UartSendPacket(USART1, txPacket->data, txPacket->lenth); 
    SetRTS_High();  // set hart-modem in RX mode
}

/**
 * @brief Parsing of received data packet
 * @param packet - received data packet
 * @retval long or normal frame flag
 */
ProtocolState_t* packetParsing(uint8_t* packet)
{
    uint16_t packet_lenght;

    /* Check start symbol (normal or long frame) */
    switch( packet[0] & UNIQUE_ADDR_FLAG )
    {
        case POLING_ADDR_FLAG: // Polling Address
            // The Hart Loader is not supports short frames.
            __NOP();
            break;
        case UNIQUE_ADDR_FLAG: // Unique Address
            rxLongFrame = (HartLongFrame_t*)packet;
            packet_lenght = rxLongFrame->data_len+LONGFRAME_HEADER_SIZE;
            if( rxLongFrame->data[rxLongFrame->data_len] != calcChecksum(packet, packet_lenght-1) ){
                hartState.Status.byte[0] |= (CHECKSUM_ERR | TRANSMIT_ERR); // set error flag in the status field
            }
            /* Check broadcast or unique address */
            memcpyRevers((uint8_t*)&rcvLongAddress, (uint8_t*)&rxLongFrame->address, sizeof(rxLongFrame->address));
            rcvLongAddress &= 0x3FFFFFFFFF;
            if( rcvLongAddress == 0 ){
                txLongFrame = (HartLongFrame_t*)txBuffer;
                txLongFrame->data_len = ExecuteCommand(rxLongFrame->command, rxLongFrame->data, txLongFrame->data);
                if( txLongFrame->data_len > 0 ){
                    txLongFrame->delimeter = 0;
                    txLongFrame->delimeter |= (UNIQUE_ADDR_FLAG | startSymbol);
                    txLongFrame->address = rxLongFrame->address;
                    txLongFrame->address.dev_type |= burstModeLong;
                    txLongFrame->command = rxLongFrame->command;
                    txLongFrame->data[0] = hartState.Status.byte[0];
                    txLongFrame->data[1] = hartState.Status.byte[1];
                    packet_lenght = txLongFrame->data_len+LONGFRAME_HEADER_SIZE;
                    txLongFrame->data[txLongFrame->data_len] = calcChecksum((uint8_t*)txLongFrame, packet_lenght-1);
                    HartSendPacket(txBuffer, packet_lenght); // response to the command
                }
            }
            break;
        default:
            __NOP();
            break;
    } // switch
    if( isJumpRequested ) jumpToAppication();
    return &hartState;
}

/**
 * @brief execution of the command depending on its code
 * @param cmd - opcode (command)
 * @retval size of the txData
 */
static uint8_t ExecuteCommand(uint16_t cmd, uint8_t* rxData, uint8_t* txData)
{
    static uint32_t flash_address = 0, i = 0;
    
    uint16_t txSize = 2; // tx data lenght (2 bytes reserved for status field)
    uint32_t* pData = (uint32_t*)rxData;
    
    rxDataLen = rxLongFrame->data_len;
    
    deinitLP_Timer();

    switch( cmd )   /* HART Universal Commands */
    {
        case COMMAND_42: // Perform Device Reset
            hartState.isResetRequired = true;
            break;
        
        case SET_UNIQUE_ID:
            flash_address = rxData[0]; flash_address <<= 8;
            flash_address |= rxData[1]; flash_address <<= 8;
            flash_address |= rxData[2];
            /* Save uinique id number to eeprom */
            FlashState = EEPROM_WriteBlock(EEPROM_STARTADDRESS, (uint8_t*)&flash_address, sizeof(flash_address));
            if( FlashState != FLASH_COMPLETE ){
                hartState.Status.byte[1] = eeprom_write_err; // eeprom write error
            }
            txData[txSize++] = rxData[0];
            txData[txSize++] = rxData[1];
            txData[txSize++] = rxData[2];
            break;
        
        case ERASE_APP_FLASH: // Erase Application memory
            FLASH_Unlock();
            flash_address = APP_START_ADDRESS;
            for(i = 0; i < APP_PAGES_NUMBER; i++){                
                FlashState = FLASH_ErasePage(flash_address);
                if( FlashState != FLASH_COMPLETE ){
                    hartState.Status.byte[1] = flash_erase_err;  // flash erase error!
                    FLASH_Lock();
                    goto exit;
                }
                flash_address += FLASH_PAGE_SIZE;
            }
            FLASH_Lock();
            // The response contains an application memory size
            txData[txSize++] = (uint8_t)(flashAppSize >> 24);
            txData[txSize++] = (uint8_t)(flashAppSize >> 16);
            txData[txSize++] = (uint8_t)(flashAppSize >> 8);
            txData[txSize++] = (uint8_t)(flashAppSize >> 0);
            break;
        
        case WRITE_FLASH_FIRST:
            flash_address = APP_START_ADDRESS;
            realAppSize = 0;
        case WRITE_FLASH_NEXT:
            FLASH_Unlock();
            realAppSize += rxDataLen;           // compute firmware size
            for(i = 0; i < rxDataLen; i += 4){
                FlashState = FLASH_FastProgramWord(flash_address, *pData++);
                if( FlashState != FLASH_COMPLETE ){
                    hartState.Status.byte[1] = flash_write_err;  // flash write error!
                    FLASH_Lock();
                    goto exit;
                }
                flash_address += 4;
            }
            FLASH_Lock();
            break;

        case CHECK_HASH_SUM:            
            RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable CRC32
            CRC_ResetDR();
            fwHashSum = CRC_CalcBlockCRC((uint32_t*)APP_START_ADDRESS, realAppSize/4);
            RCC->AHBENR &= ~RCC_AHBENR_CRCEN; // disable CRC32
            txData[txSize++] = (uint8_t)(fwHashSum >> 24);
            txData[txSize++] = (uint8_t)(fwHashSum >> 16);
            txData[txSize++] = (uint8_t)(fwHashSum >> 8);
            txData[txSize++] = (uint8_t)(fwHashSum >> 0);
            // Save application size and hash info to eeprom
            FlashState = EEPROM_WriteBlock(APP_INFO_STARTADDR, (uint8_t*)&realAppSize, sizeof(realAppSize));
            if( FlashState != FLASH_COMPLETE ){
                hartState.Status.byte[1] = eeprom_write_err; // eeprom write error
            }
            FlashState = EEPROM_WriteBlock(APP_INFO_STARTADDR+4, (uint8_t*)&fwHashSum, sizeof(fwHashSum));
            if( FlashState != FLASH_COMPLETE ){
                hartState.Status.byte[1] = eeprom_write_err; // eeprom write error
            }
            break;
            
        case CHECK_MEMORY:
            txData[txSize] = 0;
            // Check configuration memory region
            if( *(uint32_t*)(CONFIG_INFO_STARTADDR+4) == 
                checkMemoryRegion(CONFIG_DATA_STARTADDR, *(uint32_t*)CONFIG_INFO_STARTADDR/4) )
            {
                txData[txSize] |= 0x01; // config memory writen flag
            }
            // Check calibration memory region
            if( *(uint32_t*)(CALIBR_INFO_STARTADDR+4) == 
                checkMemoryRegion(CALIBR_DATA_STARTADDR, *(uint32_t*)CALIBR_INFO_STARTADDR/4) )
            {
                txData[txSize] |= 0x02; // calib memory writen flag
            }
            // Check application memory region
            if( *(uint32_t*)(APP_INFO_STARTADDR+4) == 
                checkMemoryRegion(APP_START_ADDRESS, *(uint32_t*)APP_INFO_STARTADDR/4) )
            {
                txData[txSize] |= 0x04; // app memory writen flag
            }
            txSize++;
            break;
        
        case START_APPLICATION:
            isJumpRequested = true;
            break;
        
        default:
            hartState.Status.byte[0] = UNSUPPORTED_CMD;
            break;
    }
exit:
    return txSize;
}
//eof
