/**
 * @brief HART protocol implementation
 *        (c)2018 I.Filippov
 */
#ifndef __HART_H
#define __HART_H

#include "stm32l0xx.h"
#include "convert.h"
#include "uart.h"
#include "modem5700.h"

#define DEVICE_VAR_NUM              (uint8_t)0
#define DINAMIC_VAR_NUM             (uint8_t)4          // Do not change this!
#define DEVICE_VENDOR_CODE          (uint16_t)(0x6116)  // ALCONT Ltd
#define DEVICE_DISTR_CODE           (uint16_t)(0x6116)  // ALCONT Ltd
#define DEVICE_TYPE_CODE            (uint16_t)(0xE42D)  // V-318-i
#define DEVICE_PROFILE              (uint16_t)1
#define UNIV_CMD_REVISION           (uint8_t)7
#define SPEC_CMD_REVISION           (uint8_t)7
#define SOFTWARE_REVISION           (uint8_t)5
#define HARDWARE_REVISION           (uint8_t)4
#define PV_CLASSIFICATION           (uint8_t)67         // PV - Velocity
#define SV_CLASSIFICATION           (uint8_t)172        // SV - Acceleration
#define TV_CLASSIFICATION           (uint8_t)96         // TV - Acceleration
#define QV_CLASSIFICATION           (uint8_t)96         // QV - Acceleration
#define PV_DEFAULT_UNITS            (uint8_t)21         // m/s
#define SV_DEFAULT_UNITS            (uint8_t)172        // m/s2
#define TV_DEFAULT_UNITS            (uint8_t)21         // m/s2
#define QV_DEFAULT_UNITS            (uint8_t)21         // m/s2
#define ASSEMBLY_NUMBER             (uint32_t)1
#define PV_LOW_LIMIT                (float)0.0
#define PV_HIGH_LIMIT               (float)0.1
#define PV_MINIMAL_SPAN             (float)0.01          //
#define PV_DAMPING_VALUE            (float)0.03
#define DATE_DAY                    (uint8_t)3
#define DATE_MONTH                  (uint8_t)7
#define YEAR_NOW                    1972
#define DATE_YEAR                   (YEAR_NOW-1900)     // Do not change this!

/* Polling address */
#define DEVICE_POLING_ID            (uint8_t)0

/* HART Delimiter */
#define POLING_ADDR_FLAG            (uint8_t)0x00       // 0 Poling (1 Byte) Address
#define UNIQUE_ADDR_FLAG            (uint8_t)0x80       // 1 Unique (5 Byte) Address
#define EXP_BYTES_MASK              (uint8_t)0x60       // Mask for Number of Expansion Bytes
#define PHY_LAYER_MASK              (uint8_t)0x18       // Mask for Physical Layer Type
#define FRAME_TYPE_MASK             (uint8_t)0x07       // Mask for Frame Type
#define FRAME_TYPE_ACK              (uint8_t)6          // ACK (Field Device to Master)
#define FRAME_TYPE_STX              (uint8_t)2          // STX (Master to Field Device)
#define FRAME_TYPE_BACK             (uint8_t)1          // BACK (Burst Frame)

#define H_PREAMBLE_SIZE             (uint8_t)5          //
#define FRAME_HEADER_SIZE           (uint8_t)5          // Header + checksum size in norm frame
#define LONGFRAME_HEADER_SIZE       (uint8_t)9          // Header + checksum size in long frame
#define EXPANSION_SIZE              (uint8_t)4          // Expansion data
#define H_DATA_SIZE                 (uint16_t)255       // Max data size + checksum
#define HART_MAX_SIZE               (uint16_t)264       // HART packet max lenght without preamble

#define H_SHORTTAG_SIZE             (uint32_t)6
#define H_LONGTAG_SIZE              (uint32_t)32
#define H_DESCRIP_SIZE              (uint32_t)12
#define H_MESSAGE_SIZE              (uint32_t)24

/* Hart Variables Order */
typedef enum{
    PV = 0,
    SV,
    TV,
    QV
}VarOrder_t;

/* Hart Variable Codes */
typedef enum{
    PERC_CODE = 244,
    CURR_CODE,
    PV_CODE,
    SV_CODE,
    TV_CODE,
    QV_CODE
}VarCodes_t;

/* Alarm Selection */
typedef enum{
    alHigh = 0,
    alLow,
    alHoldLastOutputValue = 239,
    //240-249 Enumeration May Be Used For Manufacturer Specific Definitions
    //250 Not Used
    alNone = 251,
    alUnknown,
    alSpecial
}AlarmCodes_t;

/* Extended Device Status Flags */
typedef enum{
    MAINTEINANCE_REQ = 0x01,
    DEV_VAR_ALLERT = 0x02,
    CRITICAL_POWER_FAIL = 0x04,
    FAILURE = 0x08,
    OUT_OF_SPECIFICATION = 0x10,
    FUNCTION_CHECK = 0x20
}ExtStatusFlags_t;

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

/* Status flags (Status, byte[1]) */
typedef enum{
    PV_OUT_OF_LIMITS = 0x01,        // primary variable out of limits
    NON_PV_OUT_OF_LIMITS = 0x02,    // variable (not primary) out of limits
    CURRENT_SATURATED = 0x04,       // output current saturated
    CURRENT_FIXED = 0x08,           // output current fixed
    MORE_STATUS = 0x10,             // extended status available
    COLD_START  = 0x20,             // cold start
    CONFIG_CHANGED = 0x40,          // configuration has been changed
    DEVICE_MALFUNCTION = 0x80       // device malfunction
}StatusFlags_t;

/* HART Protocol Commands */
typedef enum{
    /* HART universal commands */
    COMMAND_0 = 0,                  // 0 - Reads the sensor unique ID
    COMMAND_1,                      // 1 - Reads value of the primary variable
    COMMAND_2,                      // 2 - Reads current and percent of range
    COMMAND_3,                      // 3 - Reads value of the current of the PV, and up to four predefined Dynamic Variables
    COMMAND_6 = 6,                  // 6 - Writes the address to the field device
    COMMAND_7,                      // 7 - Read Loop Configuration
    COMMAND_8,                      // 8 - Read Dynamic Variable Classifications
    COMMAND_9,                      // 9 - Read Device Variables with Status
    COMMAND_11 = 11,                // 11 - Reads Unique Identifier Associated with Tag
    COMMAND_12,                     // 12 - Reads a message in the device
    COMMAND_13,                     // 13 - Reads the Tag, Descriptor, and Date contained in the device
    COMMAND_14,                     // 14 - Reads the sensor information by primary variable
    COMMAND_15,                     // 15 - Reads the signal information on primary variable
    COMMAND_16,                     // 16 - Reads the Assembly Number belonging to this device
    COMMAND_17,                     // 17 - Writes a message to the device
    COMMAND_18,                     // 18 - Writes the Tag, Descriptor, and Date in the device
    COMMAND_19,                     // 19 - Writes Assembly Number to Device
    COMMAND_20,                     // 20 - Reads long Tag
    COMMAND_21,                     // 21 - Reads Unique Identifier Associated with Tag
    COMMAND_22,                     // 22 - Writes long Tag
    /* HART common commands */
    COMMAND_33 = 33,                // 33 - Reads the sensor variables
    COMMAND_34,                     // 34 - Writes damping value for the primary variable
    COMMAND_35,                     // 35 - Writes range value for the primary variable
    COMMAND_36,                     // 36 - Upper range value of the primary variable
    COMMAND_37,                     // 37 - Lower range value of the primary variable
    COMMAND_38,                     // 38 - Resets (sets to 'zero') Configuration Change Flag
    COMMAND_39,                     // 39 - Recording from shadow RAM to non-volatile memory or vice versa
    COMMAND_40,                     // 40 - Set device in the fixed current mode of the PV
    COMMAND_41,                     // 41 - Initializes the device self-test function
    COMMAND_42,                     // 42 - Performs Device Reset
    COMMAND_43,                     // 43 - Adjusts Primary variable to zero of the scale
    COMMAND_44,                     // 44 - Writes units of measure of the primary variable
    COMMAND_45,                     // 45 - DAC zero adjustment
    COMMAND_46,                     // 46 - DAC Gain adjustment
    COMMAND_47,                     // 47 - Writes function for the analog output of the device by the Primary Variable
    COMMAND_48,                     // 48 - Read Additional Device Status
//    READ_PRIM_SERIAL,               // 49 - Returns serial number of sensor primary variable
//    READ_VARS_CODES,                // 50 - Returns the numbers of the variables for the 1-st, 2-nd, 3-d, 4-th variables
//    WRITE_VARS_CODES,               // 51 - Assigns device variables to the 1-st, 2-nd, 3-d, 4-th variables
//    SET_ZERO_VARIABLE,              // 52 - Sets the sensor variable to zero
//    WRITE_UNIT_VARIABLE,            // 53 - Sets the sensor variable units
//    READ_VARIABLE_INFO,             // 54 - Reads information about the sensor variable
//    WRITE_DAMPING_VAR,              // 55 - Writes damping value for the sensor variable
//    WRITE_VAR_SERIAL,               // 56 - Writes serial number of sensor variable (by variable's code)
//    READAll_TAG_DESC_DATE,          // 57 - Reads the Tag, Descriptor, Date of the whole device
//    WRITEAll_TAG_DESC_DATE,         // 58 - Writes the Tag, Descriptor, Date for the whole device
//    WRITE_PREAMBLE_NUM,             // 59 - Quantity of the preamble of answer
//    READ_AOUT_RANGE_PERC,           // 60 - Reads analogue output and percentage from the range
//    READ_DIN_VARS_AOUT_PRIM,        // 61 - Reads dynamic variables and analogue output of primary variable
//    READ_ANALOGUE_OUTPUTS,          // 62 - Reads the selected analog output levels
//    READ_AOUT_INFO,                 // 63 - Reads information about analogue output
//    WRITE_ADDDAMP_VALUE,            // 64 - Writes an additional damping value for the selected analog output
//    WRITE_AOUT_RANGE,               // 65 - Writes range values for the selected analog output
//    FIXED_AOUTPUT_MODE,             // 66 - Enter/exit fixed analogue output mode
//    AOUTPUT_ZERO_ADJAST,            // 67 - Zero adjustment of the selected analog output
//    AOUTPUT_HIGH_ADJUST,            // 68 - Adjustment of the upper limit of the selected analog output
//    WRITE_AOUT_CONV_FUNC,           // 69 - Writes the conversion function for the selected analog output of the device
    WRITE_VARS_MONOPOLY = 107,      // 107 - Writes sensor variables for monopoly mode
    WRITE_COMMANDD_MONO,            // 108 - Selects the sensor response will transmit during operation in monopoly mode
    COMMAND_109,                    // 109 - This command is used to enter and exit the monopoly mode
//    READ_All_DYNAMIC_VARS,          // 110 - Reads up to four predefined dynamic variables
//    TSERVICE_MANAGEMENT,            // 111 - The command from the category of  management the data link layer
    /* Non-Public commands */
    ERASE_APP_FLASH = 122,          // 122 - Non-Public command Erase flash memory of the application
    WRITE_FLASH_FIRST,              // 123 - Non-Public command for start writing to the device flash memory
    WRITE_FLASH_NEXT,               // 124 - Non-Public command Wrire next data chunk to the device flash memory
    START_APPLICATION,              // 125 - Non-Public command Start device application from loader
    SET_UNIQUE_ID,                  // 126 - Non-Public command Set device unique identificator
    /* HART specific commands */
    // 128 ... 253 //
    SPEC_CMD_1 = 128,
    SPEC_CMD_2,    
    CHECK_HASH_SUM = 230,           // 230 - Check hash sum of the writen app memory
    CHECK_MEMORY,                   // 231 - Check device memory
    CLEAR_CONFIG                    // 232 - Clear configuration data in the EEPROM (Return to the factory settings)
}HartCommand_t;

/* Staus field */
typedef union{
    uint16_t both;
    uint8_t byte[2];    
}HartStatus_t;

/* Hart Variable Structure */
typedef struct{
    float value;
    uint8_t code;
    uint8_t unit;
    uint8_t classification;
    uint8_t status;
}hartVariable_t;

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

/* Structure of date in HART-format */
#pragma pack(push, 1)
typedef struct{
    uint8_t day;
    uint8_t month;
    uint8_t year; // (year - 1900)
}HartDate_t;
#pragma pack(pop)

/* Structure of address for HART long frame */
#pragma pack(push, 1)
typedef struct{
    uint16_t dev_type;          // Expanded Device Type code
    uint8_t dev_id_0;           // Slave device id most
    uint8_t dev_id_1;           // Slave device id secondary
    uint8_t dev_id_2;           // Slave device id least
}LongAddress_t;
#pragma pack(pop)

/* Structure of short HART-frame */
#pragma pack(push, 1)
typedef struct{
    uint8_t delimeter;          // Start symbol in frame
    uint8_t address;            // Address in frame
    uint8_t command;            // Command in frame
    uint8_t data_len;           // Lenght of data field in frame
    uint8_t data[64];           // Data field in frame
}HartShortFrame_t;
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

/* Structure of HART device configuration */
#pragma pack(push, 4)
typedef struct{
    float pvLowLim;
    float pvHighLim;
    float pvMinSpan;
    float pvDamping;
//    uint32_t uniqueID;
    uint32_t assemblyNum;
    uint16_t deviceType;
    uint16_t changeCounter;
    uint8_t pollingID;
    uint8_t pvUnits;
    uint8_t svUnits;
    uint8_t tvUnits;
    uint8_t qvUnits;
    uint8_t pvAlarmCode;
    uint8_t univCmdRev;
    uint8_t specCmdRev;
    uint8_t softwareRev;
    uint8_t hardwareRev;
    uint8_t shortTag[H_SHORTTAG_SIZE];
    uint8_t longTag[H_LONGTAG_SIZE];
    uint8_t descriptor[H_DESCRIP_SIZE];
    uint8_t message[H_MESSAGE_SIZE];
    HartDate_t Date;
    bool isLoopCurrMode;
    //uint8_t dummy1;                       // it is nessuary to allign the structure
    //uint8_t dummy2;
    
}HartConfig_t;
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
