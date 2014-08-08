#ifndef __USBP_H
#define __USBP_H

//FreeRtos
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"



void USBP_Init(void);
int USBP_ProcessDataFromISR(uint8_t data);


#endif /* __USBP_H */