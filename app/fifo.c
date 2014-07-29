// FIFO.C
// Source file for GENERIC FIFO routines
// MAH - 06/2014 - Initial Version

#include <stdint.h>
#include <string.h>
#include <intrinsics.h>
#include "fifo.h"

// FIFO constructor 
void FIFO_Init(FIFO_T *me, void *buffer, uint32_t elements, size_t element_size)
{
    me->elements = elements;
    me->element_size = element_size;
    me->base = (uint8_t *) buffer;
    me->head = (uint8_t *) buffer;
    me->tail = (uint8_t *) buffer;    
}

// Puts new element on FIFO
int FIFO_Post(FIFO_T *me, void *element)
{ 
    // Critical region, interrupts disabled
    __disable_interrupt();
    
    // New head
    uint8_t *new_head = me->head + me->element_size;
    
    // Head wrapping around
    if (new_head > me->base + ((me->elements-1)*me->element_size))
        new_head = me->base;
    
    // Overflow, return error code
    if (new_head == me->tail)
    {
        // End of critical region, interrupts enabled again
        __enable_interrupt();
        return FIFO_OVERFLOW;
    }
    // Adds a member to the FIFO and advance head
    else
    {
        memcpy(new_head,element,(size_t) me->element_size);  
        me->head = new_head;
        
        // End of critical region, interrupts enabled again
        __enable_interrupt();
         return FIFO_NOERROR;  
    }
}

// Gets an element from the FIFO
int FIFO_Get(FIFO_T *me, void *element)
{     
    // Tail caught up with head, return underflow
    if (me->tail == me->head)
        return FIFO_UNDERFLOW;
    // Gets the member from the FIFO and advance tail
    else
    {
        // Critical region, interrupts disabled
        __disable_interrupt();
        // New tail
        uint8_t *new_tail = me->tail + me->element_size;
    
        // Tail wrapping around
        if (new_tail > me->base + ((me->elements-1)*me->element_size))
            new_tail = me->base;
        
        memcpy(element,new_tail,(size_t) me->element_size);  
        me->tail = new_tail;
        // End of critical region, interrupts enabled again
        __enable_interrupt();
         return FIFO_NOERROR;  
    }  
}

// Gets the available space on the FIFO
// The minus one comes from the fact that Head == Tail is only acceptable as 
// a FIFO empty condition. So, for all practical purposes, you got one less
// space available to fill the Queue. 
uint32_t FIFO_AvailableElements(FIFO_T *me)
{
    // Head == Tail -> Queue is free!
    if(me->head == me->tail)
        return (me->elements - 1);
    // Head is geometrically ahead of tail
    else if(me->head > me->tail)
        return (me->elements - 1 - (me->head - me->tail)/me->element_size);
    // Head is geometrically behind of tail
    else 
        return ((me->tail - me->head)/me->element_size - 1);
}

