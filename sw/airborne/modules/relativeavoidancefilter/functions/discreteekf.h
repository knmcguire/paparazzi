#ifndef DISCRETEEKF_H
#define DISCRETEEKF_H

#include "fmatrix.h"
#include "stdlib.h"
#include "string.h"
#include "string.h"
#include "math.h"

#define NSTATES 9
#define NMEASUREMENTS 8
#define UNUSEDVAR(x) (void)(x)

typedef struct ekf_filter {

  // unsigned state_dim;
  // unsigned measure_dim;

  /* state                           */
  float X[NSTATES];
  /* state prediction                */
  float Xp[NSTATES];
  /* measurement prediction          */
  float Z[NMEASUREMENTS];
  /* measurement prediction          */
  float Zp[NMEASUREMENTS];
  /* state covariance matrix         */
  float P[NSTATES*NSTATES];
  /* process covariance noise        */
  float Q[NSTATES*NSTATES];
  /* measurement covariance noise    */
  float R[NMEASUREMENTS*NMEASUREMENTS];
  /* jacobian of Xdot wrt X          */
  float A[NSTATES*NSTATES];
  /* jacobian of the measure wrt X   */
  float H[NSTATES*NMEASUREMENTS];
  /* error matrix                    */
  float E[NMEASUREMENTS*NMEASUREMENTS];
  /* inverse error matrix            */
  float invE[NMEASUREMENTS*NMEASUREMENTS];
  /* kalman gain                     */
  float K[NSTATES*NMEASUREMENTS];
  /* Error vector                    */
  float err[NMEASUREMENTS];

  /* temps */
  float dX[NSTATES];
  float Pdot[NSTATES*NSTATES];
  float tmp1[NSTATES*NSTATES];
  float tmp2[NSTATES*NSTATES];
  float tmp3[NSTATES*NSTATES];

} ekf_filter;

/*
 * Basic functions describing evolution and measure
 */

extern void zero(float *matrix, int row, int col);
extern void identity(float *matrix, int n);
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