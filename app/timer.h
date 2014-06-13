#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f2xx.h"

#define MAX_IR_PULSES   50

typedef struct tim_capture
{
  uint32_t capture[2];
  uint8_t cap_flag;
  uint32_t pulse_len;
  uint32_t freq;
}tim_capture;

typedef struct tim_ir_gen
{
  uint32_t pulses[MAX_IR_PULSES];
  uint32_t iterator;
  uint32_t n_pulses;
}tim_ir_gen;

void TIMER_OutputCompareCh3Init(void);
void TIMER_InputCaptureCh2Init(void);
tim_capture * TIMER_GetCaptureCh2Struct(void);
void TIMER_DemoIrGenStructInit(void);
tim_ir_gen * TIMER_GetIrGenStruct(void);

#endif /* __TIMER_H */
