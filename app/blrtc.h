#ifndef __BLRTC_H
#define __BLRTC_H

#include "stm32f2xx.h"

void BLRTC_Init(void);
int BLRTC_SetTime(RTC_TimeTypeDef time);
RTC_TimeTypeDef BLRTC_GetTime(void);


RTC_DateTypeDef BLRTC_GetDate(void);

void BLRTC_ConfigAlarm(RTC_AlarmTypeDef alarm);

#endif