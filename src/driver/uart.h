/**
 * @brief UART driver
 *        (c)2018 I.Filippov
 */
#ifndef __UART_H
#define __UART_H

#include "stm32l0xx.h"
#include "string.h"
#include "stdbool.h"

/**
 * @brief UART error codes
 */
typedef enum{
    UART_OVERRUN_ERROR = USART_ISR_ORE,
    UART_NOISE_ERROR = USART_ISR_NE,
    UART_FRAME_ERROR = USART_ISR_FE,
    UART_PARITY_ERROR = USART_ISR_PE,
}UartError_t;

/**
 * @brief UART events callback functions
 */
typedef struct{
    void(*rxByteDone)(uint8_t);
    void(*rxBlockDone)(void* prm);
    void(*txReady)(void);
    void(*txComplete)(void);
    void(*error)(uint8_t);
}UartEvents_t;

/**
 * @brief DMA events callback functions
 */
typedef struct{
    void(*ch2TxComplete)(void);
    void(*ch2TxError)(void);
}DmaCh2Events_t;

/* UART Driver API */
void initUart1(UartEvents_t *events, uint32_t br);
void UartTxEnable(USART_TypeDef* USARTx);
void UartTxDisable(USART_TypeDef* USARTx);
void UartSendByte(USART_TypeDef* USARTx, uint8_t data);
void UartSendPacket(USART_TypeDef* USARTx, uint8_t* pack, uint16_t len);
void UartTcEnable(USART_TypeDef* USARTx);
void UartTcDisable(USART_TypeDef* USARTx);
bool isUartTxDone(void);
void initDmaMem2Uart(DmaCh2Events_t* events);
void SendPacket(uint32_t src_addr, uint32_t size);
static void startDmaMem2Uart(uint32_t src_addr, uint32_t size);
void stopDmaMem2Uart(void);
#endif
//eof
