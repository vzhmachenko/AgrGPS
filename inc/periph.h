#ifndef _Periph_H_
#define _Periph_H_
#include "stm32f4xx.h"

void gpio_ini(void);

void RCC_Init(void);
//GPS
void USART2_init(void);
void dma1ini(void);

//BlueTooth
void USART6_init(void);
void dma2ini(void);

#endif
