#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f2xx.h"
   
int16_t circindex(int16_t index, int16_t incr, int16_t nitems);
void sort(uint32_t * buf, uint16_t size);
void gauss(float ** a, float * b, float * x, uint16_t n);
void reg_lin(float v[2], float  r[2], float * coef_ang, float * coef_lin);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H */
