#include "discreteekf.h"


#include "fmatrix.h"

#include "stdlib.h"
#include "string.h"


struct ekf_filter {
  unsigned state_dim;
  unsigned measure_dim;

  /* state                           */
  float* X;
  /* state prediction                */
  float* Xp;
  /* measurement prediction          */
  float* Z;
  /* measurement prediction          */
  float* Zp;
  /* state covariance matrix         */
  float* P;
  /* process covariance noise        */
  float* Q;
  /* measurement covariance noise    */
  float* R;
  /* jacobian of Xdot wrt X          */
  float* A;
  /* jacobian of the measure wrt X   */
  float* H;
  /* error matrix                    */
  float* E;
  /* inverse error matrix            */
  float* invE;
  /* kalman gain                     */
  float* K;
  /* Error vector                    */
  float* err;

  filter_function ffun;
  measure_function mfun;

  /* temps */
  float* dX;
  float* Pdot;
  float* tmp1;
  float* tmp2;
  float* tmp3;

};


struct ekf_filter* ekf_filter_new(
          unsigned state_dim,
				  unsigned measure_dim,
				  float* Q,
				  float* R,
				  filter_function ffun,
				  measure_function mfun) {

  struct ekf_filter* ekf = malloc(sizeof(struct ekf_filter));
  ekf->state_dim = state_dim;
  ekf->measure_dim = measure_dim;

  int n = ekf->state_dim;
  ekf->X = malloc( n * sizeof(float));
  ekf->dX = malloc( n * sizeof(float));
  ekf->Xp = malloc( n * sizeof(float));

  n = ekf->measure_dim;
  ekf->Z = malloc( n * sizeof(float));
  ekf->Zp = malloc( n * sizeof(float));
  ekf->err = malloc( n * sizeof(float));

  n = ekf->state_dim * ekf->state_dim;
  ekf->P = malloc( n * sizeof(float));
  ekf->Pdot = malloc( n * sizeof(float));
  ekf->tmp1 = malloc( n * sizeof(float));
  ekf->tmp2 = malloc( n * sizeof(float));
  ekf->tmp3 = malloc( n * sizeof(float));

  ekf->Q = malloc( n * sizeof(float));
  memcpy(ekf->Q, Q, n * sizeof(float));

  n = ekf->measure_dim * ekf->measure_dim;
  ekf->R = malloc( n * sizeof(float));
  memcpy(ekf->R, R, n * sizeof(float));

  n = ekf->state_dim * ekf->state_dim;
  ekf->A = malloc( n * sizeof(float));

  n = ekf->measure_dim * ekf->state_dim;
  ekf->H = malloc( n * sizeof(float));

  n = ekf->measure_dim * ekf->measure_dim;
  ekf->E = malloc( n * sizeof(float));
  ekf->invE = malloc( n * sizeof(float));

  n = ekf->state_dim * ekf->measure_dim;
  ekf->K = malloc( n * sizeof(float));

  ekf->ffun = ffun;
  ekf->mfun = mfun;

  return ekf;
}


void ekf_filter_reset(struct ekf_filter *filter, float *x0, float *P0) {
  memcpy(filter->X, x0, filter->state_dim * sizeof(float));
  memcpy(filter->P, P0, filter->state_dim * filter->state_dim * sizeof(float));
}

void ekf_filter_get_state(struct ekf_filter* filter, float *X, float* P){
  memcpy(X, filter->X, filter->state_dim * sizeof(float));
  memcpy(P, filter->P, filter->state_dim * filter->state_dim * sizeof(float));
}

void ekf_filter_predict(struct ekf_filter* filter, float *u) {

  /*
    PREDICT:
    Predict state
      x_p = f(x);
      A = Jacobian of f(x)
    
    Predict P
      P = A * P * A' + Q;
    
    Predict measure
      z_p = h(x_p)
      H = Jacobian of h(x)
  */  

  float dt;
  int n = filter->state_dim; // Number of states

  // Fetch dt, dX and A given the current state X and input u
  filter->ffun(u, filter->X, &dt, filter->dX, filter->A);

  // Get state prediction Xp = X + dX
  fmat_add(n,1, filter->Xp, filter->X, filter->dX); 

  // Get measurement prediction Zp based on Xp and get Jacobian H
  filter->mfun(filter->Xp, filter->Zp, filter->H);

  /*
      discrete update
      P = A * P * A' + Q
  */
  fmat_mult(n, n, n, filter->tmp1, filter->A, filter->P); // A*P
  fmat_transpose(n, n, filter->tmp2, filter->A); // A'
  fmat_mult(n, n, n, filter->tmp3, filter->tmp1, filter->tmp2); // A*P*A'
  fmat_add(n, n, filter->P, filter->tmp3, filter->Q); // A*P*A' + Q

}

void ekf_filter_update(struct ekf_filter* filter, float *y) {

  /*
    UPDATE:
      Get Kalman Gain
        P12 = P * H';
        K = P12/(H * P12 + R);
      
      Update x
        x = x_p + K * (z - z_p);
      
      Update P
        P = (eye(numel(x)) - K * H) * P;
  */

  int n = filter->state_dim;
  int m = filter->measure_dim;
  // The problem is that H is not updating!
  /*  E = H * P * H' + R */
  fmat_transpose(m, n, filter->tmp2, filter->H); // H'
  fmat_mult(n, n, m, filter->tmp1, filter->P, filter->tmp2); // P*H'
  fmat_mult(m, n, m, filter->tmp3, filter->H, filter->tmp1); // H*P*H'
  fmat_add(m, m, filter->E, filter->tmp3, filter->R); // H*P*H' + R

  /* Get Kalman gain K = P * H' * inv(E) */
  fmat_inverse(m, filter->invE, filter->E); // inv(E)
  fmat_transpose(m, n, filter->tmp1, filter->H); // H'
  fmat_mult(n, n, m, filter->tmp2, filter->P, filter->tmp1);  // P*H'
  fmat_mult(n, m, m, filter->K, filter->tmp2, filter->invE); // K = P*H'*inv(E)

  /* P = P - K * H * P */
  fmat_mult(n, m, n, filter->tmp1, filter->K, filter->H); // K*H
  fmat_mult(n, n, n, filter->tmp2, filter->tmp1, filter->P); // K*H*P
  fmat_sub(n, n, filter->P, filter->P, filter->tmp2);
  
  /*  X = X + err * K */
  memcpy(filter->Z, y, m * sizeof(float));
  fmat_sub(m, 1, filter->err, filter->Z, filter->Zp);

  fmat_mult( n, m, 1, filter->tmp1, filter->K,filter->err);

  fmat_add(n, 1, filter->X, filter->Xp, filter->tmp1);

}
