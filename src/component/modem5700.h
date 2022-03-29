/**
 * @file modem5700.h
 * @brief Header file for modem.c
 *        (c)2018 I.Filippov
 */
#ifndef __MODEM5700_H
#define __MODEM5700_H

#include "stm32l0xx.h"
#include "stddef.h"
#include "stdbool.h"

#define MODEM_PORT_CLK        RCC_IOPENR_GPIOAEN
#define MODEM_PORT            GPIOA
#define MODEM_CD_PIN          (uint16_t)(1 << 8)    // pin 8
#define MODEM_RESET_PIN       (uint16_t)(1 << 11)   // pin 11
#define MODEM_RTS_PIN         (uint16_t)(1 << 12)   // pin 12

#define MODEM_CD_EXTI_LINE    (uint16_t)(1 << 8)    // exti line 8
#define MODEM_CD_EXTI_PORT    EXTI_PortSourceGPIOA
#define MODEM_CD_EXTI_PIN     EXTI_PinSource8

/* Modem Driver API */
void initHartModem(void(*callback)(void));
void resetHartModem(void);
bool isHartModemSet(void);
void SetRTS_High(void);
void SetRTS_Low(void);
    
#endif
//eof
