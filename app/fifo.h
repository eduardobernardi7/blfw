//FIFO.H
// Header file for GENERIC FIFO routines
// MAH - 06/2014 - Initial Version

#ifndef __FIFO_H_
#define __FIFO_H_

#include <string.h>
#include <stdint.h>

// FIFO type
typedef struct FIFO_TAG
{
    uint8_t    *base;
    uint8_t    *head;
    uint8_t    *tail;
    uint32_t    elements;
    size_t      element_size;
} FIFO_T;

// Return values
enum FIFO_RETURN_VAL
{
    FIFO_NOERROR    = 0,
    FIFO_OVERFLOW   = -1,
    FIFO_UNDERFLOW  = -2
};

// Prototypes
void       FIFO_Init(FIFO_T *me, void *buffer, uint32_t elements, size_t element_size);
int        FIFO_Post(FIFO_T *me, void *element);
int        FIFO_Get(FIFO_T *me, void *element);
uint32_t   FIFO_AvailableElements(FIFO_T *me);

#endif // __FIFO_H_