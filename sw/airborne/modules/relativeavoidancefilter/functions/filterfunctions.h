#ifndef _FILTERFUNCTIONS_H_
#define _FILTERFUNCTIONS_H_

#define UNUSEDVAR(x) (void)(x)

#include "math.h"

extern void zero(float *matrix, int row, int col);
extern void identity(float *matrix, int n);
extern void linear_filter(float *u, float* X, float* dt, float *dX, float* A);
extern void linear_measure(float*X, float* Y, float *H);

#endif
