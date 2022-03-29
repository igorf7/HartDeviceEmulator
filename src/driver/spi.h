/**
 * @brief SPI driver
 *        (c)2018 I.Filippov
 */
#ifndef __SPI_H
#define __SPI_H

#include "stm32l0xx.h"
#include "stdbool.h"

#define CS_PIN      (1 << 4)

/* SPI Driver API */
void initSpi1(void);
uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx);
void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data);
void clearChipSelect(void);
bool isSpiBusy(SPI_TypeDef* SPIx);
void setChipSelect(void);
#endif
//eof
