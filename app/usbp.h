#ifndef __USBP_H
#define __USBP_H

void USBP_Init(void);
int USBP_ProcessDataFromISR(uint8_t data);


#endif /* __USBP_H */