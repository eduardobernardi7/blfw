#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f2xx.h"

void SYSTICK_delay_ms(__IO uint32_t nTime);
void SYSTICK_tick(void);
void SYSTICK_Init(void);


#endif /* __SYSTICK_H */