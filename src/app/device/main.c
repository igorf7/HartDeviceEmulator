/**
 * @file main.c
 * @brief This file contain main application functions
 *        (c)2018 I.Filippov
*/
#include "main.h"
#include "task.h"
#include "iwdg.h"
#include "rcc.h"
#include "uart.h"
#include "hart.h"
#include "ports.h"
#include "memory.h"
#include "modem5700.h"

#ifdef USE_DEBUG_LEDS
#include "ledctl.h"
#endif // USE_DEBUG_LEDS

static bool isStartDetected = false;
static bool isRebootRequired = false;

static UartEvents_t uartEvents;
static DmaCh2Events_t dmaCh2Events;
    
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
#ifndef DEBUG_VERSION
    SCB->VTOR = APP_START_ADDRESS;
#endif
    
    SystemClockConfig();     // clock system setup
    SystemCoreClockUpdate(); // update clock variables

    initPorts();             // ports setup
    
    /* Configure LED3 and LED4 on STM32L0-Discovery */
#ifdef USE_DEBUG_LEDS
    ledInit();
#endif

    /* Check if the system has resumed from IWDG reset */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        RCC_ClearFlag(); // clear reset flag
    }
    
    /* Uart callback-functions initialize */
    uartEvents.rxByteDone = onUartRxByteDone;   // byte received
    uartEvents.rxBlockDone = onUartRxBlockDone; // packet received
    uartEvents.txReady = NULL;                  // not used
    uartEvents.txComplete = onUartTxComplete;   // transmit complete
    uartEvents.error = onUartError;             // error occurred
    
    /* DMA callback-functions initialize */
    dmaCh2Events.ch2TxComplete = onDmaCh2TxComplete; // RAM to USART_TX transfer done
    dmaCh2Events.ch2TxError = NULL;                  // not used yet
    
    /* Uart hardware initialization */
    initUart1(&uartEvents, 1200);
    initDmaMem2Uart(&dmaCh2Events); // DMA for RAM to USART_TX transfer
    
    /* HART-Modem initialization */
    initHartModem(&onCarrierDetect);
    
    /* Protocol initialization */
    initHartProtocol();
    setBufferAddress(&txDataBuf); // pass buffer address to the protocol
    
    /* IWDG hardware initialization */
    setupIwdg();

    /* Initialize the queue of task handlers */
    InitTaskQueue();
    
    __enable_irq();
    
    /* Mainloop */
    while (1)
    {
        IWDG_ReloadCounter();                   // reload watchdog timer
        
        if (WriteQPtr == ReadQPtr) {            // check the queue is empty or not
			__nop();//dataProcessing();                   // background task run here
		}
		else {
			ReadQPtr->FuncPtr(ReadQPtr->Param); // pass control to the handler
			ReadQPtr = ReadQPtr->Next;          // move the pointer to the next item
		}
    }
} // end of main func


/*************************************************************************************/
/*** Callback functions, they are called from the corresponding interrupt handlers ***/
/*************************************************************************************/

/**
 * @brief  On Uart Rx done callback function.
 *         It is called from the Uart Rx interrupt handler.
 * @param  data - received byte via UART.
 * @retval None
 */
void onUartRxByteDone(uint8_t data)
{
    switch (data)
    {
        case 0xFF:
            if (!isStartDetected) {
                return; // skip a preamble
            }
            break;
        default:
            isStartDetected = true;
            break;
    }
    rxDataBuf.data[rxDataBuf.index++] = data;
    if (rxDataBuf.index > (HART_MAX_SIZE)) {
        rxDataBuf.index = 0;
        isStartDetected = false;
    }
}

/**
 * @brief  On rxBlockDone Task Handler.
 *         It is called from the Uart Rx Timeout interrupt handler.
 * @param  None
 * @retval None
 */
void onUartRxBlockDone(void* prm)
{
    ProtocolState_t* state;
    
    isStartDetected = false;
    rxDataBuf.lenth = rxDataBuf.index;
    rxDataBuf.index = 0;
    
    /* If the packet is from the master, pass it to the Hart protocol */
    if ((rxDataBuf.data[0] & 0x02) == 0x02) {
        state = packetParsing(rxDataBuf.data);
    }
    
    isRebootRequired = state->isResetRequired;
    
    /* Check and clear status field */
    if (state->Status.both != 0) {
        state->Status.both = 0;
    }
}

/**
 * @brief  On Uart Tx Complete callback function.
 *         It is called from the Uart Tx Complete interrupt handler.
 * @param  None
 * @retval None
 */
void onUartTxComplete(void)
{
    /* The end of UART transmission */
    UartTcDisable(USART1);  // disable interrupt Tx Complete
    SetRTS_High();          // set hart-modem in RX mode
    //
    if (isRebootRequired == true) {
        NVIC_SystemReset(); // reset MCU
    }
}

/**
 * @brief  On DMA Channel_2 Tx Complete callback function.
 *         It is called from the DMA Tx Complete interrupt handler.
 *         Channel_2 is used to transfer data from RAM to USART.
 * @param  None
 * @retval None
 */
void onDmaCh2TxComplete(void)
{
    stopDmaMem2Uart();      // stop DMA
    UartTcEnable(USART1);   // enable UART Tx Complete interrupt
}

/**
 * @brief  On modem Carrier Detect callback function.
 *         It is called from the CD Pin interrupt handler.
 * @param  None
 * @retval None
 */
void onCarrierDetect(void)
{
    // Used in debug mode only
    __NOP();
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
    if (err_code & UART_PARITY_ERROR) {
        err_state.byte[0] |= (PARITY_ERR | TRANSMIT_ERR);
    }
    if (err_code & UART_FRAME_ERROR) {
        err_state.byte[0] |= (FRAMING_ERR | TRANSMIT_ERR);
    }
    if (err_code & UART_OVERRUN_ERROR) {
        err_state.byte[0] |= (OVERRUN_ERR | TRANSMIT_ERR);
    }
    if (err_state.byte[0] != 0) {
        setTransferError(&err_state); // set transfer error flags in status field
    }
    err_state.both = 0;
}
//eof
