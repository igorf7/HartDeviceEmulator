#ifndef __LEDCTL_H
#define __LEDCTL_H

#include "stm32l0xx.h"
#include "ports.h"

#define RCC_IOPENR_LEDS         RCC_IOPENR_GPIOBEN
#define LED_PORT                GPIOB
#define GREEN_LED_PIN           (1 << 3)
#define RED_LED_PIN             (1 << 4)
#define LED_PINS                (GREEN_LED_PIN|RED_LED_PIN)

typedef enum {
  LED_GREEN = GREEN_LED_PIN,
  LED_RED   = RED_LED_PIN,
  LED_ALL   = LED_PINS
} BoardLed_t;

/* Leds Driver API */
void ledInit(void);
void ledOn(BoardLed_t led);
void ledOff(BoardLed_t led);
void ledToggle(BoardLed_t led);
void ledIndicateErr(void);
#endif // __LEDCTL_H
