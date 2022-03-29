/**
 * @file main.c
 * @brief This file contain main application functions
 *        (c)2018 Alcont, Sarov
*/
#include "main.h"
#include "task.h"
#include "ports.h"
#include "rcc.h"
#include "uart.h"
#include "hart_lite.h"
#include "modem5700.h"
#include "memory.h"
#include "lptimer.h"

static bool isStartDetected = false;

static UartEvents_t uartEvents;

static DataBuffer_t txDataBuf;
static DataBuffer_t rxDataBuf;

extern QueueItem_t TaskQueue[];
extern QueueItem_t* WriteQPtr;
extern QueueItem_t* ReadQPtr;

/**
 * @brief  Main function of the v318hart project.
 * @param  None
 * @retval None
 */
int main(void)
{
    SystemClockConfig();     // clock system setup
    SystemCoreClockUpdate(); // update clock variables

    initPorts();             // ports setup
    
    /* Check if the system has resumed from IWDG reset */
    if( RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET ){
        RCC_ClearFlag();    // clear reset flag
    }

    /* Uart callback-functions initialize */
    uartEvents.rxByteDone = onUartRxByteDone;
    uartEvents.rxBlockDone = onUartRxBlockDone;
    uartEvents.txReady = NULL;
    //uartEvents.txComplete = onUartTxComplete;
    uartEvents.txComplete = NULL;
    uartEvents.error = NULL;

    /* Uart hardware initialization */
    initUart1(&uartEvents, 1200);

    /* HART-Modem initialization */
    initHartModem(NULL);
    
    /* Protocol initialization */
    initHartProtocol();
    setBufferAddress(&txDataBuf); // pass buffer address to the protocol

    /* Initialize the queue of task handlers */
    InitTaskQueue();
    
    /* Timeout for start application */
    initLP_Timer(&onLP_Timer);
    
    /* Mainloop */
    while( 1 )
    {
        if( WriteQPtr == ReadQPtr ){            // check is queue empty or not
            __NOP();                            // background task run here
		}
		else{
			ReadQPtr->FuncPtr(ReadQPtr->Param); // pass control to the handler
			ReadQPtr = ReadQPtr->Next;          // move pointer to next item
		}
    }
} // end of main func

/**
 * @brief  This function pass control to the application.
 * @param  None
 * @retval None
 */
void jumpToAppication(void)
{
    uint32_t jumpAddress;
    void (*jumpToApp)(void);
    
    // Deinit periphery
    NVIC_DisableIRQ(LPTIM1_IRQn);
    NVIC_DisableIRQ(USART1_IRQn);
    RCC->AHBENR &= ~RCC_AHBENR_CRCEN;
    RCC->IOPENR &= ~RCC_IOPENR_GPIOAEN;
    RCC->IOPENR &= ~RCC_IOPENR_GPIOBEN;
    RCC->IOPENR &= ~RCC_IOPENR_GPIOCEN;
    RCC->IOPENR &= ~RCC_IOPENR_GPIODEN;
    RCC->IOPENR &= ~RCC_IOPENR_GPIOHEN;
    RCC->APB1ENR &= ~RCC_APB1ENR_LPTIM1EN;
    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
    RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
    // Jump to Application
    __disable_irq();
    jumpAddress = *(__IO uint32_t*)(APP_START_ADDRESS + 4);
    jumpToApp = (void (*)(void))(jumpAddress);
    SCB->VTOR = APP_START_ADDRESS;
    __set_MSP(*(__IO uint32_t*)APP_START_ADDRESS);
    jumpToApp();
}

/*************************************************************************************/
/*** Callback functions, they are called from the corresponding interrupt handlers ***/
/*************************************************************************************/

/**
 * @brief  On Uart Rx done callback function.
 *         It is called from the Uart Rx interrupt handler.
 * @param  data - received byte via UART
 * @retval None
 */
void onUartRxByteDone(uint8_t data)
{
    switch( data )
    {
        case 0xFF:
            if( !isStartDetected ) {
                return; // skip a preamble
            }
            break;
        default:
            isStartDetected = true;
            break;
    }
    rxDataBuf.data[rxDataBuf.index++] = data;
    if( rxDataBuf.index > (HART_MAX_SIZE) ){
        rxDataBuf.index = 0;
        isStartDetected = false;
    }
}

/**
 * @brief  On Uart Rx Block done callback function (packet received).
 *         It is called from the Uart Rx Timeout interrupt handler.
 * @param  prm - a pointer to void (can be used to accept parameter for this handler)
 * @retval None
 */
void onUartRxBlockDone(void* prm)
{
    ProtocolState_t* state;
    
    isStartDetected = false;
    rxDataBuf.lenth = rxDataBuf.index;
    rxDataBuf.index = 0;
    
    /* If the packet is from the master, pass it to the Hart protocol */
    if( (rxDataBuf.data[0] & 0x02) == 0x02 ){
        state = packetParsing(rxDataBuf.data);
    }
  
    /* Check and clear status field */
    if( state->Status.both != 0 ){
        state->Status.both = 0;
    }
    
    /* Check and perform reboot if required */
    if( state->isResetRequired == true ){
        NVIC_SystemReset(); // reset MCU
    }
}

/**
 * @brief  On Uart Error callback function.
 *         It is called from the Uart Error interrupt handler.
 * @param  None
 * @retval None
 */
void onUartError(uint8_t err_code)
{
    static HartStatus_t err_state;
 
    // Handle UART errors
    if( err_code & UART_PARITY_ERROR ){
        err_state.byte[0] |= (PARITY_ERR | TRANSMIT_ERR);
    }
    if( err_code & UART_FRAME_ERROR ){
        err_state.byte[0] |= (FRAMING_ERR | TRANSMIT_ERR);
    }
    if( err_code & UART_OVERRUN_ERROR ){
        err_state.byte[0] |= (OVERRUN_ERR | TRANSMIT_ERR);
    }
    if( err_state.byte[0] != 0 ){
        setTransferError(&err_state); // set transfer error flags in status field
    }
    err_state.both = 0;
}

/**
 * @brief  On LP Timer callback function.
 *         It is called from the LP Timer interrupt handler.
 * @param  None
 * @retval None
 */
void onLP_Timer(void)
{
    deinitLP_Timer();
    jumpToAppication();
}
//eof
