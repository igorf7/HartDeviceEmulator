/**
 * @brief Main header file for HartDevice project
 *        (c)2018 I.Filippov
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l0xx.h"

/* v318hart API */
void onUartRxByteDone(uint8_t data);
void onUartRxBlockDone(void* prm);
void onUartTxReady(void);
void onUartTxComplete(void);
void onUartError(uint8_t err_code);
void onCarrierDetect(void);
void onDmaCh2TxComplete(void);

#endif /* __MAIN_H */
