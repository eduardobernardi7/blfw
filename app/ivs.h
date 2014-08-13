#ifndef __IVS_H
#define __IVS_H

//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void IVS_Init(void) ;
SemaphoreHandle_t * IVS_GetTimerMutex(void);
SemaphoreHandle_t * IVS_GetTimerTickSem(void);
void IVS_TickFromISR(void);
void IVS_Perform_Curve(unsigned int periodHours, unsigned int periodIntervalMins);
void IVS_StopCurve(void);


#endif /* __IVS_H */