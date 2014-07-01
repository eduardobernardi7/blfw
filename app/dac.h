#ifndef __DAC_H
#define __DAC_H

#include "stm32f2xx.h"

void DAC_HwInit(void);
void DAC_SetVctr(uint16_t vctr2, uint16_t vctr1);

#endif /* __DAC_H */
