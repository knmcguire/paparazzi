#include "discreteekf.h"


void ekf_filter_setup(
  ekf_filter *filter,
				  float* Q,
				  float* R)
{
  memcpy(filter->Q, Q, NSTATES*NSTATES * sizeof(float));
  memcpy(filter->R, R, NMEASUREMENTS*NMEASUREMENTS * sizeof(float));
}


void ekf_filter_reset(ekf_filter *filter, float *x0, float *P0)
{
  memcpy(filter->X, x0, NSTATES * sizeof(float));
  memcpy(filter->P, P0, NSTATES * NSTATES * sizeof(float));
}

void ekf_filter_get_state(ekf_filter* filter, float *X, float* P){
  memcpy(X, filter->X, NSTATES * sizeof(float));
  memcpy(P, filter->P, NSTATES * NSTATES * sizeof(float));
}

void ekf_filter_predict(ekf_filter* filter) {

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
  int n = NSTATES; // Number of states

  // Fetch dt, dX and A given the current state X and input u
  linear_filter(filter->X, &dt, filter->dX, filter->A);

  // Get state prediction Xp = X + dX
  fmat_add(n, 1, filter->Xp, filter->X, filter->dX); 

  // Get measurement prediction Zp based on Xp and get Jacobian H
  linear_measure(filter->Xp, filter->Zp, filter->H);

  /*
      discrete update
      P = A * P * A' + Q
  */
  fmat_mult(n, n, n, filter->tmp1, filter->A, filter->P); // A*P
  fmat_transpose(n, n, filter->tmp2, filter->A); // A'
  fmat_mult(n, n, n, filter->tmp3, filter->tmp1, filter->tmp2); // A*P*A'
  fmat_add(n, n, filter->P, filter->tmp3, filter->Q); // A*P*A' + Q

}

void ekf_filter_update(ekf_filter* filter, float *y) {

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

  int n = NSTATES;
  int m = NMEASUREMENTS;
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

void linear_filter(float* X, float* dt, float *dX, float* A)
{

  *dt = 0.2;

  /* dX */
  // Make a zero vector
  fmat_make_zeroes(dX,9,1);
  dX[0] = -(X[2] - X[4])*(*dt);
  dX[1] = -(X[3] - X[5])*(*dt);
  
  /* F'(x) */
  // make an identity matrix
  fmat_make_identity(A,9);
  A[0*9+2] = -*dt;
  A[0*9+4] =  *dt;

  A[1*9+3] = -*dt;
  A[1*9+5] =  *dt;
};

void linear_measure(float*X, float* Y, float *H)
{
  float Pn = -65.0;
  float gamma = 2.5;

  // RSSI measurement
  Y[0] = Pn - (10.0 * gamma * log10(sqrt(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0))));

  // x velocity of i wrt i body frame
  Y[1] = X[2];

  // y velocity of i wrt i body frame
  Y[2] = X[3];

  // Orientation of i wrt north
  Y[3] = X[6];

  // x velocity of j wrt j body frame
  Y[4] = cos( X[6] - X[7] ) * X[4] - sin( X[6] - X[7] ) * X[5];

  // y velocity of j wrt  j body frame
  Y[5] = sin( X[6] - X[7] ) * X[4] + cos( X[6] - X[7] ) * X[5];

  // Orientation of j wrt north
  Y[6] = X[7];

  // Height difference
  Y[7] = X[8];

  int n = 9;
  int m = 8;  
  int row, col;

  // Generate the Jacobian Matrix
  for (row = 0 ; row < m ; row++ )
  {
    for (col = 0 ; col < n ; col++ )
    {
      if ((row == 0) && (col == 0 || col == 1 || col == 8 ))
        H[ row*n+col ] = (-gamma*10/log(10))*(X[col]/(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0)));
      
      else if (((row == 1) && (col == 2)) ||
        ((row == 2) && (col == 3)) ||
        ((row == 3) && (col == 6)) ||
        ((row == 6) && (col == 7)) ||
        ((row == 7) && (col == 8)))
      { 
        H[ row*n+col ] = 1.0;
      }

      else if ((row == 4) && (col == 4))
        H[ row*n+col ] = cos(X[6]-X[7]);
      else if ((row == 4) && (col == 5))
        H[ row*n+col ] = -sin(X[6]-X[7]);
      else if ((row == 4) && (col == 6))
        H[ row*n+col ] = X[4]*sin(X[7]-X[6]) - X[5] * cos(X[7] - X[6]);
      else if ((row == 4) && (col == 7))
        H[ row*n+col ] = X[4]*sin(X[6]-X[7]) + X[5] * cos(X[6] - X[7]);
      

      else if ((row == 5) && (col == 4))
        H[ row*n+col ] = sin(X[6]-X[7]);
      else if ((row == 5) && (col == 5))
        H[ row*n+col ] = cos(X[6]-X[7]);
      else if ((row == 5) && (col == 6))
        H[ row*n+col ] = X[4]*cos(X[7]-X[6]) + X[5] * sin(X[7] - X[6]);
      else if ((row == 5) && (col == 7))
        H[ row*n+col ] = -X[4]*cos(X[6]-X[7]) + X[5] * sin(X[6] - X[7]);

      else 
        H[ row*n+col ] = 0.0;
    }
  }

};
