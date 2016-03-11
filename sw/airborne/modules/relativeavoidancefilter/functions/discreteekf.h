#ifndef DISCRETEEKF_H
#define DISCRETEEKF_H

#include "fmatrix.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define N 9
#define M 8

typedef struct ekf_filter {

  /* state                           */
  float X[N];
  /* state prediction                */
  float Xp[N];
  /* measurement prediction          */
  float Zp[M];
  /* state covariance matrix         */
  float P[N*N];
  /* process covariance noise        */
  float Q[N*N];
  /* measurement covariance noise    */
  float R[M*M];
  /* jacobian of the measure wrt X   */
  float H[N*M];

  /* Temp matrices */
  float tmp1[N*N];
  float tmp2[N*N];
  float tmp3[N*N];

} ekf_filter;

/*
 * Basic functions describing evolution and measure
 */

extern void linear_filter(float* X, float* dt, float *dX, float* A);
extern void linear_measure(float*X, float* Y, float *H);

extern void ekf_filter_setup(
					ekf_filter *filter, 
					float* Q,
					float* R);
extern void ekf_filter_reset(ekf_filter *filter, float *x0, float *P0);

extern void ekf_filter_predict(ekf_filter *filter);
extern void ekf_filter_update(ekf_filter *filter, float *y);

extern void ekf_filter_get_state(ekf_filter* filter, float *X, float* P);

#endif /* EKF_H */