#ifndef __IHM_H
#define __IHM_H

//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

typedef enum PB_TAG
{
  USER_B1,
  USER_B2
}PB_T;

typedef enum LED_TAG
{
  USER_LED0,
  USER_LED1,
  USER_LED2,
  USER_LED3
}LED_T;

typedef enum LED_VALUE_TAG
{
  HIGH,
  LOW,
  TOG
}LED_VALUE_T;

void IHM_Init(void);
xQueueHandle * IHM_GetPBQueuePointer(void);
xTaskHandle * IHM_GetTaskHandlePointer(void);
xSemaphoreHandle * IHM_GetPBSemPointer(void);
void IHM_SetLed(LED_T led, LED_VALUE_T value);

#endif /* __IHM_H */