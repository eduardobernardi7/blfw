#ifndef __DAC_H
#define __DAC_H

#include "stm32f2xx.h"
#define  DAC_MAX_VALUE      (4095)


// Return values
enum DAC_RETURN_VAL
{
    DAC_NOERROR     = 0,
    DAC_VALUE_OUTSIDE_BOUNDARIES = -1,
};

void DAC_HwInit(void);
int DAC_SetDAC1ValInMilivolts(uint16_t val);
int DAC_SetDAC2ValInMilivolts(uint16_t val);
int DAC_SetBothDacsInMilivolts(uint16_t dac1_val, uint16_t dac2_val);

#endif /* __DAC_H */
