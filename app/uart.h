#ifndef __UART_H
#define __UART_H

#include "stm32f2xx.h"

void UART_Init(void);

int UART_Post_RxDataFromISR(uint16_t data);

int UART_Get_RxData(uint16_t * data);

int UART_PostTxData(uint16_t * data);

int UART_Get_TxDataFromISR(uint16_t * data);

#endif /* __UART_H */