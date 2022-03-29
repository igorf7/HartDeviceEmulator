/**
 * @file hart.c
 * @brief HART protocol implementation
 *        (c)2018 I.Filippov
 */
#include "hart.h"
#include "memory.h"
#include "crc32.h"
#include "iwdg.h"
#include <stdlib.h>

static HartShortFrame_t* rxShortFrame;
static HartShortFrame_t* txShortFrame;
static HartLongFrame_t* rxLongFrame;
static HartLongFrame_t* txLongFrame;

static DataBuffer_t* txPacket;

static ProtocolState_t hartState;

static HartConfig_t Config;

static FLASH_Status EEPROM_State;

static uint64_t rcvLongAddress = 0,
                myLongAddress = 0;

static uint32_t
       hartTimeStamp = 123,
       uniqueID = 0;

static uint16_t
       extendedStatus = 0,
       burstModeLong = 0;

static uint8_t
       burstModeShort = 0,
       extDevStatus,
       addressType,
       startSymbol;

static hartVariable_t variable[DINAMIC_VAR_NUM];

static struct{
    float current,
          percent;
}prmPV;

static uint8_t txBuffer[HART_MAX_SIZE];

/**
 *
 */
static void enableBurstMode(void)
{
    startSymbol = FRAME_TYPE_BACK;
    burstModeShort = 0x40;
    burstModeLong = 0x4000;
    
    // TODO:
    // Setup and start monopoly mode timer
}

/**
 *
 */
static void disableBurstMode(void)
{
    startSymbol = FRAME_TYPE_ACK;
    burstModeShort = 0;
    burstModeLong = 0;
    
    // TODO:
    // Stop monopoly mode timer
}

/**
 * @brief Calculates a checksum
 * @param packet - data packet
 * @param len - packet lenght
 */
static uint8_t calcChecksum(uint8_t *packet, uint16_t len)
{
    uint8_t checksum = packet[0];
    
    for (int i = 1; i < len; i++) {
        checksum ^= packet[i];
    }
    return checksum;
}

/**
 * @brief Initialize of protocol variables
 * @param Buffer address
 * @retval None
 */
void initHartProtocol(void)
{
    uint32_t checksum;
    uint32_t nan = 0x7FA00000;
    float* fp_nan = (float*)&nan;   // value for unused variables
    
    uint32_t size = sizeof(Config);
    
    disableBurstMode();

    RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable CRC32
    CRC_ResetDR();
    checksum = CRC_CalcBlockCRC((uint32_t*)CONFIG_DATA_STARTADDR, size/4);
    RCC->AHBENR &= ~RCC_AHBENR_CRCEN; // disable CRC32
    
    //////////////////////////////////////////////////////////////////////
    ///srand(*(uint32_t*)(EEPROM_STARTADDRESS));
    //////////////////////////////////////////////////////////////////////
    
    uniqueID = *(uint32_t*)(EEPROM_STARTADDRESS);
    hartState.Status.both = 0;
    
    if (checksum == *(uint32_t*)(CONFIG_INFO_STARTADDR+4)) {
        /* Loading saved configuration from EEPROM */
        memcpy((uint8_t*)&Config, (uint8_t*)CONFIG_DATA_STARTADDR, sizeof(Config));
    }
    else {
        /* Loading default configuration */
        Config.pollingID   = DEVICE_POLING_ID;
        Config.univCmdRev  = UNIV_CMD_REVISION;
        Config.specCmdRev  = SPEC_CMD_REVISION;
        Config.softwareRev = SOFTWARE_REVISION;
        Config.hardwareRev = HARDWARE_REVISION << 3;
        Config.pvUnits     = PV_DEFAULT_UNITS;
        Config.svUnits     = SV_DEFAULT_UNITS;
        Config.tvUnits     = TV_DEFAULT_UNITS;
        Config.qvUnits     = QV_DEFAULT_UNITS;
        Config.pvAlarmCode = alNone;
        Config.deviceType  = DEVICE_TYPE_CODE;
        Config.assemblyNum = ASSEMBLY_NUMBER;
        Config.pvLowLim    = PV_LOW_LIMIT;
        Config.pvHighLim   = PV_HIGH_LIMIT;
        Config.pvDamping   = PV_DAMPING_VALUE;
        Config.pvMinSpan   = PV_MINIMAL_SPAN;
        Config.Date.day    = DATE_DAY;
        Config.Date.month  = DATE_MONTH;
        Config.Date.year   = DATE_YEAR;
        Config.changeCounter = 0;
    }
    
//    if (Config.pollingID == 0) {          // is it nessuary?
//        Config.isLoopCurrMode = true;
//    }
//    else {
//        Config.isLoopCurrMode = false;
//    }
    
    /* Device unique address */
    myLongAddress = (Config.deviceType & 0x3FFF);
    myLongAddress <<= 24;
    myLongAddress |= uniqueID;
    
    /*  */
    hartState.isResetRequired = false;
    hartState.isConfClearReq = false;

    /* Initialization of supported variables */
    prmPV.current = 4.08f;
    prmPV.percent = 0.0f;
    variable[PV].classification = PV_CLASSIFICATION;    
    variable[PV].code = PV_CODE;
    variable[PV].unit = Config.pvUnits;
    variable[PV].value = 0.123;
    variable[PV].status = 0;    
    variable[SV].classification = SV_CLASSIFICATION;
    variable[SV].code = SV_CODE;
    variable[SV].unit = Config.svUnits;
    variable[SV].value = 0.005;
    variable[SV].status = 0;        
    
    /* Initialization of unsupported variables */
    variable[TV].classification = 0; // not classified
    variable[TV].code = 250; // not used
    variable[TV].unit = 250; // not used
    variable[TV].value = *fp_nan;
    variable[TV].status = 0x30;
    variable[QV].classification = 0; // not classified
    variable[QV].code = 250; // not used
    variable[QV].unit = 250; // not used
    variable[QV].value = *fp_nan;
    variable[QV].status = 0x30;
    
    extDevStatus = 0;
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
 * @brief Returns a preamble size
 * @param None
 * @retval preamble size
 */
uint8_t getPreambleSize(void)
{
    return H_PREAMBLE_SIZE;
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
 * @brief sends a data packet using the HART protocol
 * @param data packet
 * @param lenght of the data packet
 */
void HartSendPacket(uint8_t* packet, uint16_t len)
{
    uint32_t i;
    
    for (i = 0; i < H_PREAMBLE_SIZE; i++) {
        txPacket->data[i] = 0xFF; // fill preamble field
    }
    memcpy(txPacket->data+i, packet, len); // fill PDU field
    txPacket->index = 0;
    txPacket->lenth = len + H_PREAMBLE_SIZE;
    
    SetRTS_Low(); // set hart-modem in TX mode
    SendPacket((uint32_t)txPacket->data, txPacket->lenth); // send data packet to hart-modem
}

/**
 * @brief Parsing of received data packet
 * @param packet - received data packet
 * @retval long or normal frame flag
 */
ProtocolState_t* packetParsing(uint8_t* packet)
{
    uint16_t packet_lenght;
    
    IWDG_ReloadCounter();                   // reload watchdog timer
//    if (Config.changeCounter != 0) {
//        hartState.Status.byte[1] |= CONFIG_CHANGED;
//    }
    /* Check start symbol (normal or long frame) */
    switch(packet[0] & UNIQUE_ADDR_FLAG)
    {
        case POLING_ADDR_FLAG: // Polling Address
            addressType = POLING_ADDR_FLAG;
            rxShortFrame = (HartShortFrame_t*)packet;
            packet_lenght = rxShortFrame->data_len+FRAME_HEADER_SIZE;
            if (rxShortFrame->data[rxShortFrame->data_len] != calcChecksum(packet, packet_lenght-1)) {
                hartState.Status.byte[0] |= (CHECKSUM_ERR | TRANSMIT_ERR); // set error flag in the status field
            }
            if (((rxShortFrame->address & 0x3F) == Config.pollingID) && (rxShortFrame->command == COMMAND_0)) {
                txShortFrame = (HartShortFrame_t*)txBuffer;
                txShortFrame->data_len = ExecuteCommand(rxShortFrame->command, rxShortFrame->data, txShortFrame->data);
                if (txShortFrame->data_len > 0) {
                    txShortFrame->delimeter = 0;
                    txShortFrame->delimeter |= startSymbol;
                    txShortFrame->address = rxShortFrame->address;
                    txShortFrame->address |= burstModeShort;
                    txShortFrame->command = rxShortFrame->command;
                    txShortFrame->data[0] = hartState.Status.byte[0];
                    txShortFrame->data[1] = hartState.Status.byte[1];
                    packet_lenght = txShortFrame->data_len+FRAME_HEADER_SIZE;
                    txShortFrame->data[txShortFrame->data_len] = calcChecksum((uint8_t*)txShortFrame, packet_lenght-1);
                    HartSendPacket(txBuffer, packet_lenght); // response to the command
                }
            }
            break;
        case UNIQUE_ADDR_FLAG: // Unique Address
            addressType = UNIQUE_ADDR_FLAG;
            rxLongFrame = (HartLongFrame_t*)packet;
            packet_lenght = rxLongFrame->data_len+LONGFRAME_HEADER_SIZE;
            if (rxLongFrame->data[rxLongFrame->data_len] != calcChecksum(packet, packet_lenght-1)) {
                hartState.Status.byte[0] |= (CHECKSUM_ERR | TRANSMIT_ERR); // set error flag in the status field
            }
            /* Check broadcast or unique address */
            memcpyRevers((uint8_t*)&rcvLongAddress, (uint8_t*)&rxLongFrame->address, sizeof(rxLongFrame->address));
            rcvLongAddress &= 0x3FFFFFFFFF;
            if ((rcvLongAddress == 0) || (rcvLongAddress == myLongAddress)) {
                txLongFrame = (HartLongFrame_t*)txBuffer;
                txLongFrame->data_len = ExecuteCommand(rxLongFrame->command, rxLongFrame->data, txLongFrame->data);
                if (txLongFrame->data_len > 0) {
                    txLongFrame->delimeter = 0;
                    txLongFrame->delimeter |= (UNIQUE_ADDR_FLAG | startSymbol);
                    txLongFrame->address = rxLongFrame->address;
                    txLongFrame->address.dev_type |= burstModeLong;
                    txLongFrame->address = rxLongFrame->address;
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
    
    if (hartState.isConfClearReq == true) {
        hartState.isConfClearReq = false;
        EEPROM_State = EEPROM_EraseBlock(CONFIG_DATA_STARTADDR, sizeof(Config));
        if (EEPROM_State != FLASH_COMPLETE) {
            __NOP();// error erase EEPROM
        }
        hartState.isResetRequired = true;
    }
    if (hartState.isConfUpdateReq == true) {
        hartState.isConfUpdateReq = false;
        uint32_t size = sizeof(Config);        
        Config.changeCounter++;
        RCC->AHBENR |= RCC_AHBENR_CRCEN; // enable CRC32
        CRC_ResetDR();
        uint32_t crc32 = CRC_CalcBlockCRC((uint32_t*)&Config, size/4);
        RCC->AHBENR &= ~RCC_AHBENR_CRCEN; // disable CRC32
        // Write config to the EEPROM
        EEPROM_State = EEPROM_WriteBlock(CONFIG_DATA_STARTADDR, (uint8_t*)&Config, size);
        if (EEPROM_State != FLASH_COMPLETE) {
            __NOP(); // eeprom write error
        }
        // Write info about config_size to the EEPROM
        EEPROM_State = EEPROM_WriteBlock(CONFIG_INFO_STARTADDR, (uint8_t*)&size, 4);
        if (EEPROM_State != FLASH_COMPLETE) {
            __NOP(); // eeprom write error
        }
        // Write info about config_hash to the EEPROM
        EEPROM_State = EEPROM_WriteBlock(CONFIG_INFO_STARTADDR+4, (uint8_t*)&crc32, sizeof(crc32));
        if (EEPROM_State != FLASH_COMPLETE) {
            __NOP(); // eeprom write error
        }
        //hartState.isResetRequired = true;
        initHartProtocol();
    }
//    if (Config.isLoopCurrMode != true)
//        hartState.Status.byte[1] |= (CURRENT_FIXED | 0x80);   // is it nessuary?

    return &hartState;
}

/**
 * @brief execution of the command depending on its code
 * @param cmd - opcode (command)
 * @retval size of the txData
 */
static uint8_t ExecuteCommand(uint16_t cmd, uint8_t* rxData, uint8_t* txData)
{
    uint16_t txSize = 2; // tx data lenght (2 bytes reserv for status field)
    uint16_t rxDataLen = rxLongFrame->data_len;
    uint16_t i;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///variable[PV].value += 0.1;
    ///variable[SV].value += 0.0001;
    ///prmPV.current += 0.0;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    variable[PV].value = 0.0001 + (rand() % ((80 + 1) - 1) + 1);
    variable[PV].value /= 1000;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    if (cmd < 33) {
        switch(cmd)   /* HART Universal Commands */
        {
            case COMMAND_0:
cmd_0:          txData[txSize++] = 254; // extention
                txData[txSize++] = Config.deviceType >> 8;
                txData[txSize++] = Config.deviceType;
                txData[txSize++] = H_PREAMBLE_SIZE;
                txData[txSize++] = Config.univCmdRev;
                txData[txSize++] = Config.specCmdRev;
                txData[txSize++] = Config.softwareRev;
                txData[txSize++] = Config.hardwareRev;
                txData[txSize++] = 0;   // flags
                txData[txSize++] = (uint8_t)(uniqueID >> 16);
                txData[txSize++] = (uint8_t)(uniqueID >> 8);
                txData[txSize++] = (uint8_t)(uniqueID >> 0);
            
                txData[txSize++] = 5; // Minimum number of preambles to be sent with the response message
                txData[txSize++] = DEVICE_VAR_NUM; // Maximum Number of Device Variables
                txData[txSize++] = Config.changeCounter >> 8;
                txData[txSize++] = Config.changeCounter;
                txData[txSize++] = extendedStatus; //
                txData[txSize++] = (uint8_t)(DEVICE_VENDOR_CODE >> 8);
                txData[txSize++] = (uint8_t)DEVICE_VENDOR_CODE;
                txData[txSize++] = (uint8_t)(DEVICE_VENDOR_CODE >> 8);
                txData[txSize++] = (uint8_t)DEVICE_VENDOR_CODE;
                txData[txSize++] = DEVICE_PROFILE;
                
                ///
                ///Config.pollingID++;
                ///uniqueID++;
                ///
                
                break;
            case COMMAND_1:
                txData[txSize++] = Config.pvUnits; // PV measure units
                FloatToByteArray(variable[PV].value, &txData[txSize]); // PV
                txSize += sizeof(variable[PV].value);
                break;
            case COMMAND_2:
                FloatToByteArray(prmPV.current, &txData[txSize]); // current
                txSize += sizeof(prmPV.current);
                prmPV.percent = (prmPV.current-4.0f)/16.0f*100.0f;
                FloatToByteArray(prmPV.percent, &txData[txSize]); // percent of the range
                txSize += sizeof(prmPV.percent);
                break;
            case COMMAND_3:
                FloatToByteArray(prmPV.current, &txData[txSize]); // current
                txSize += sizeof(prmPV.current);
                for (i = 0; i < DINAMIC_VAR_NUM; i++) {
                    if (variable[i].code != 250) { // if this variable is supported
                        txData[txSize++] = variable[i].unit; // variable measure units
                        FloatToByteArray(variable[i].value, &txData[txSize]);
                        txSize += sizeof(variable[i].value);
                    }
                }
                break;
            case COMMAND_6:
                if ((rxData[0] > 63) || (rxDataLen == 0)) {
                    hartState.Status.byte[0] = INVALID_SELECTION; // Invalid Poll Address Selection
                }
                else {
                    txData[txSize++] = Config.pollingID = rxData[0];
                    if ((addressType == UNIQUE_ADDR_FLAG) && rxDataLen > 1) {
                        txData[txSize++] = rxData[1];
                        Config.isLoopCurrMode = (bool)rxData[1]; // set/reset loop current mode
                    }
                    hartState.isConfUpdateReq = true;
                }
                break;
            case COMMAND_7:
                txData[txSize++] = Config.pollingID;
                txData[txSize++] = (uint8_t)Config.isLoopCurrMode;
                break;
            case COMMAND_8:
                txData[txSize+PV] = 0; // That's right
                txData[txSize+SV] = 0;
                txData[txSize+TV] = 0;
                txData[txSize+QV] = 0;
                for (i = 0; i < DINAMIC_VAR_NUM; i++) {
                    txData[txSize+i] = variable[i].classification;
                }
                txSize += 4;  // That's right
                break;
            case COMMAND_9:
                txData[txSize++] = extDevStatus;     // Extended device status
                for (i = 0; i < rxDataLen; i++) {
                    switch (rxData[i])
                    {
                    	case 0:
                        case PV_CODE:
                            txData[txSize++] = variable[PV].code;
                            txData[txSize++] = variable[PV].classification;
                            txData[txSize++] = variable[PV].unit;
                            FloatToByteArray(variable[PV].value, &txData[txSize]);
                            txSize += sizeof(variable[PV].value);
                            txData[txSize++] = variable[PV].status;
                    		break;
                    	case 1:
                        case SV_CODE:
                            txData[txSize++] = variable[SV].code;
                            txData[txSize++] = variable[SV].classification;
                            txData[txSize++] = variable[SV].unit;
                            FloatToByteArray(variable[SV].value, &txData[txSize]);
                            txSize += sizeof(variable[SV].value);
                            txData[txSize++] = variable[SV].status;
                    		break;
                        case 2:
                        case TV_CODE:
                            txData[txSize++] = variable[TV].code;
                            txData[txSize++] = variable[TV].classification;
                            txData[txSize++] = variable[TV].unit;
                            FloatToByteArray(variable[TV].value, &txData[txSize]);
                            txSize += sizeof(variable[TV].value);
                            txData[txSize++] = variable[TV].status;
                    		break;
                    	case 3:
                        case QV_CODE:
                            txData[txSize++] = variable[QV].code;
                            txData[txSize++] = variable[QV].classification;
                            txData[txSize++] = variable[QV].unit;
                            FloatToByteArray(variable[QV].value, &txData[txSize]);
                            txSize += sizeof(variable[QV].value);
                            txData[txSize++] = variable[QV].status;
                    		break;
                        case PERC_CODE:
                            txData[txSize++] = PERC_CODE;
                            txData[txSize++] = 111;
                            txData[txSize++] = 57;
                            prmPV.percent = (prmPV.current-4.0f)/16.0f*100.0f;
                            FloatToByteArray(prmPV.percent, &txData[txSize]);
                            txSize += sizeof(prmPV.percent);
                            txData[txSize++] = 0;
                            break;
                        case CURR_CODE:
                            txData[txSize++] = CURR_CODE;
                            txData[txSize++] = 84;
                            txData[txSize++] = 39;
                            FloatToByteArray(prmPV.current, &txData[txSize]);
                            txSize += sizeof(prmPV.current);
                            txData[txSize++] = 0;
                            break;
                    	default:
                            hartState.Status.byte[0] = INVALID_SELECTION;
                    		break;                        
                    }
                } // for
                hartState.Status.byte[0] = 14; // Dynamic Variables Returned for Device Variables
                if (rxDataLen > DINAMIC_VAR_NUM) {
                    hartState.Status.byte[0] = 30; // Command Response Truncated
                }
                txData[txSize++] = hartTimeStamp >> 24;
                txData[txSize++] = hartTimeStamp >> 16;
                txData[txSize++] = hartTimeStamp >> 8;
                txData[txSize++] = hartTimeStamp;
                break;
            case COMMAND_11:
                if (strncmp((char*)rxData, (char*)Config.shortTag, H_SHORTTAG_SIZE) != 0) {
                    __NOP(); // wrong tag
                }
                else {
                    goto cmd_0; // the answer is the same as in command Read_Unique_ID
                }
                break;
            case COMMAND_12:
cmd_12:         memcpy(&txData[txSize], Config.message, H_MESSAGE_SIZE); // place message to packet
                txSize += H_MESSAGE_SIZE;
                break;
            case COMMAND_13:
cmd_13:         memcpy(&txData[txSize], Config.shortTag, H_SHORTTAG_SIZE);
                txSize += H_SHORTTAG_SIZE;
                memcpy(&txData[txSize], Config.descriptor, H_DESCRIP_SIZE);
                txSize += H_DESCRIP_SIZE;
                //memcpy(&txData[txSize], (char*)&Config.Date, sizeof(Config.Date));
                //txSize += sizeof(Config.Date);
                txData[txSize++] = Config.Date.day;
                txData[txSize++] = Config.Date.month;
                txData[txSize++] = Config.Date.year;
                break;
            case COMMAND_14:
                txData[txSize++] = (uint8_t)(uniqueID >> 16);
                txData[txSize++] = (uint8_t)(uniqueID >> 8);
                txData[txSize++] = (uint8_t)(uniqueID >> 0);
                txData[txSize++] = Config.pvUnits;
                FloatToByteArray(Config.pvHighLim, &txData[txSize]);    // high lim
                txSize += sizeof(Config.pvHighLim);
                FloatToByteArray(Config.pvLowLim, &txData[txSize]);     // low lim
                txSize += sizeof(Config.pvLowLim);
                FloatToByteArray(Config.pvMinSpan, &txData[txSize]);    // ?????????
                txSize += sizeof(Config.pvMinSpan);
                break;
            case COMMAND_15:
                txData[txSize++] = Config.pvAlarmCode;
                txData[txSize++] = 0; // Transfer Function Code (0 - Linear)
                txData[txSize++] = variable[PV].unit;
                FloatToByteArray(Config.pvHighLim, &txData[txSize]);    // high lim
                txSize += sizeof(Config.pvHighLim);
                FloatToByteArray(Config.pvLowLim, &txData[txSize]);     // low lim
                txSize += sizeof(Config.pvLowLim);
                FloatToByteArray(Config.pvDamping, &txData[txSize]);    // Damping value
                txSize += sizeof(Config.pvDamping);
                txData[txSize++] = 250; // Write protect not used
                txData[txSize++] = 250; // Reserved. Must be set to "250", Not Used
                txData[txSize++] = 0;   // Analog Channel is an analog output
                break;
            case COMMAND_16:
cmd_16:         txData[txSize++] = (uint8_t)(Config.assemblyNum >> 16);
                txData[txSize++] = (uint8_t)(Config.assemblyNum >> 8);
                txData[txSize++] = (uint8_t)(Config.assemblyNum >> 0);
                break;
            case COMMAND_17:
                memcpy(Config.message, rxData, H_MESSAGE_SIZE);
                hartState.isConfUpdateReq = true;
                goto cmd_12;
            case COMMAND_18:
                memcpy(Config.shortTag, rxData, H_SHORTTAG_SIZE);
                rxData += H_SHORTTAG_SIZE;
                memcpy(Config.descriptor, rxData, H_DESCRIP_SIZE);
                rxData += H_DESCRIP_SIZE;
                //memcpy((char*)&Config.Date, rxData, sizeof(Config.Date));
                Config.Date.day = *rxData;
                Config.Date.month = *(rxData+1);
                Config.Date.year = *(rxData+2);
                hartState.isConfUpdateReq = true;
                goto cmd_13; // the answer is the same as in command Read_Tag_Desc_Date
            case COMMAND_19:
                Config.assemblyNum = (0x000000FF & *rxData); Config.assemblyNum <<= 8;
                Config.assemblyNum |= *(rxData+1); Config.assemblyNum <<= 8;
                Config.assemblyNum |= *(rxData+2);
                hartState.isConfUpdateReq = true;
                goto cmd_16; // the answer is the same as in command Read_Assembly_Num
            case COMMAND_20:
cmd_20:         memcpy(&txData[txSize], Config.longTag, H_LONGTAG_SIZE);
                txSize += H_LONGTAG_SIZE;
                break;
            case COMMAND_21:
                if (strncmp((char*)rxData, (char*)Config.longTag, H_LONGTAG_SIZE) != 0) {
                    __NOP(); // wrong tag
                }
                else {
                    goto cmd_0; // the answer is the same as in command Read_Unique_ID
                }
                break;
            case COMMAND_22:
                memcpy(Config.longTag, rxData, H_LONGTAG_SIZE);
                hartState.isConfUpdateReq = true;
                goto cmd_20;
            default:
                hartState.Status.byte[0] = UNSUPPORTED_CMD;
                break;
        }
    }
    else if (cmd < 128) {
        switch(cmd)   /* HART Common Practice Commands */
        {
        	case COMMAND_34: // Write Primary Variable Damping Value
                FloatToByteArray(Config.pvDamping, &txData[txSize]);
                txSize += sizeof(Config.pvDamping);
        		break;
            case COMMAND_35: // Write Primary Variable Range Values
                txData[txSize++] = rxData[0];
                Config.pvHighLim = ByteArrayToFloat(rxData, 1);
                FloatToByteArray(Config.pvHighLim, &txData[txSize]);
                txSize += sizeof(Config.pvHighLim);
                Config.pvLowLim = ByteArrayToFloat(rxData, 5);
                FloatToByteArray(Config.pvLowLim, &txData[txSize]);
                txSize += sizeof(Config.pvLowLim);
                hartState.isConfUpdateReq = true;
        		break;
            case COMMAND_38: // Reset Configuration Changed Flag
                txData[txSize++] = Config.changeCounter >> 8;
                txData[txSize++] = (uint8_t)Config.changeCounter;
                if (rxDataLen == 0) { // if HART 6 (or earlier) Version
                    hartState.Status.byte[1] &= ~CONFIG_CHANGED;
                    Config.changeCounter = 0;
                }
                else {   // if HART 7 Version
                    if (rxData[0] == (Config.changeCounter >> 8) && (rxData[1] == Config.changeCounter)) {
                        hartState.Status.byte[1] &= ~CONFIG_CHANGED;
                        Config.changeCounter = 0;
                    }
                    else {
                        hartState.Status.byte[0] = 9; // error: Configuration Change Counter Mismatch
                    }
                }
        		break;
            case COMMAND_42: // Perform Device Reset
                hartState.isResetRequired = true;
                break;
            case COMMAND_44: // Write Primary Variable Units
                if (Config.pvUnits != rxData[0]) {
                    Config.pvUnits = rxData[0];
                    hartState.isConfUpdateReq = true;
                }
                txData[txSize++] = Config.pvUnits;
                break;
            case COMMAND_48: // Read Additional Device Status
                txData[txSize++] = 0; txData[txSize++] = 0;     // Device-Specific Status [0..5]
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = extDevStatus;                // Extended Device Status
                txData[txSize++] = 0;                           // Device Operating Mode
                txData[txSize++] = 0; txData[txSize++] = 0;     // Standardized Status 0 and 1
                txData[txSize++] = 0;                           // Analog Channel Saturated
                txData[txSize++] = 0; txData[txSize++] = 0;     // Standardized Status 2 and 3
                txData[txSize++] = 0;                           // Analog Channel Fixed
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = 0; txData[txSize++] = 0;     // Device-Specific Status [14..24]
                txData[txSize++] = 0; txData[txSize++] = 0;
                txData[txSize++] = 0; //txData[txSize++] = 0;
                break;
            case COMMAND_109:
                txData[txSize++] = rxData[0];
                if (rxData[0] != 0)
                    enableBurstMode();
                else
                    disableBurstMode();
                break;
        	default:
                hartState.Status.byte[0] = UNSUPPORTED_CMD;
        		break;
        }
    }
    else {
        switch(cmd)   /* HART Specific Commands */
        {
            case CLEAR_CONFIG:
                hartState.isConfClearReq = true;
                break;
        	default:
                hartState.Status.byte[0] = UNSUPPORTED_CMD;
        		break;
        }
    }
    return txSize;
}
//eof
