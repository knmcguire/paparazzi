#ifndef ARRAYFUNCTIONS_H
#define ARRAYFUNCTIONS_H

#include "string.h" //memcpy

extern void array_shiftleft(float *array, int size, int shift);
extern void array_shiftright(float *array, int size, int shift);
extern int array_getminidx(int length, float *x);
extern int array_getmaxidx(int length, float *x);
extern void array_arraymin(int length, float *x1, float *x2);

#endif