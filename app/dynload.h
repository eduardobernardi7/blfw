// dynload.h
// Header file for Dynamic Load module

#include <stdint.h>

#ifndef __DYNLOAD_H_
#define __DYNLOAD_H_

#define DL_CURRENT_RESISTOR         (0.1f)
#define DL_PID_KP                   (0.35f)
#define DL_PID_KI                   (0.1f)
#define DL_PID_KD                   (0.05f)
#define DL_ADC_ISENSE_LB1           (ADC12_CH2)

void DL_Init(void);
void DL_DetCompleteCallbBack(uint8_t (*completeCallBack)(uint16_t));
void DL_TickFromISR();

void DL_StartPID(uint16_t desired_current_ma);
void DL_StopPID(void);


#endif
