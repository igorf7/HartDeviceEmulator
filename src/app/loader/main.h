/**
 * @brief Main header file for HartDevice project
 *        (c)2018 Alcont, Sarov
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l0xx.h"

void jumpToAppication(void);

/* hartLoader API */
void onUartRxByteDone(uint8_t data);
void onUartRxBlockDone(void *prm);
void onUartTxComplete(void);
void onDmaCh2TxComplete(void);
void onLP_Timer(void);
#endif /* __MAIN_H */
//eof
