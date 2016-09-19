/*
 * kalman_filter_vision.c
 *
 *  Created on: Sep 12, 2016
 *      Author: knmcguire
 */
#include "kalman_filter_vision.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_simple_matrix.h"

//TODO: implement kalman filter for larger states
//TODO: implement extended kalman filter



/**
 * A simple linear 2D kalman filter, computed using floats and matrices. To be used for vision task like optical flow, tracking markers etc.
 * @param[in] *model  The process model for the prediction of the next state (array size of [1 x 4])
 * @param[in] *measurements  Values of the measurements (array size of [1 x 2])
 * @param[in] *covariance  Covariance matrix of previous iteration (array size of [1 x 4])
 * @param[out] *covariance  Updated Covariance matrix (array size of [1 x 4])
 * @param[in] *state  Previous state vector with estimates (array size of [1 x 2])
 * @param[out] *state  Updated state vector with estimates (array size of [1 x 2])
 * @param[in] *measurement_noise  Expected variance of the noise of the measurements (array size of [1 x 2])
 * @param[in] *process_noise  Expected variance of the noise of the process model (array size of [1 x 2])
 */
void kalman_filter_linear_2D_float(float *model, float *measurements, float *covariance, float *state
                                   , float *process_noise, float *measurement_noise)
{
  //......................Preparation kalman filter ..................... //

  // process model (linear)
  float _G[2][2];
  MAKE_MATRIX_PTR(G, _G, 2);
  G[0][0] = model[0];
  G[0][1] = model[1];
  G[1][0] = model[2];
  G[1][1] = model[3];

  // transpose of G
  float _Gtrans[2][2];
  MAKE_MATRIX_PTR(Gtrans, _Gtrans, 2);
  float_mat_copy(Gtrans, G, 2, 2);
  float_mat_transpose(Gtrans, 2);

  // Observation model (linear)
  // note: right now both velocity and acceleration are observed
  float _H[2][2];
  MAKE_MATRIX_PTR(H, _H, 2);
  H[0][0] = 1.0f;
  H[0][1] = 0.0f;
  H[1][0] = 0.0f;
  H[1][1] = 1.0f;

  //transpose of H
  float _Htrans[2][2];
  MAKE_MATRIX_PTR(Htrans, _Htrans, 2);
  float_mat_copy(Htrans, H, 2, 2);
  float_mat_transpose(Htrans, 2);

  //Previous state
  float _Xprev[1][2];
  MAKE_MATRIX_PTR(Xprev, _Xprev, 2);
  Xprev[0][0] = state[0];
  Xprev[1][0] = state[1]; //state[1];

  //Previous covariance
  float _Pprevious[2][2];
  MAKE_MATRIX_PTR(Pprevious, _Pprevious, 2);
  Pprevious[0][0] = covariance[0];
  Pprevious[0][1] = covariance[1];
  Pprevious[1][0] = covariance[2];
  Pprevious[1][1] = covariance[3];

  //measurements;
  float _Z[1][2];
  MAKE_MATRIX_PTR(Z, _Z, 2);
  Z[0][0] = measurements[0];
  Z[1][0] = measurements[1];

  //Process noise model
  float _Q[2][2];
  MAKE_MATRIX_PTR(Q, _Q, 2);
  Q[0][0] = process_noise[0];
  Q[0][1] = 0.0f;
  Q[1][0] = 0.0f;
  Q[1][1] =  process_noise[1];

  //measurement nosie model
  float _R[2][2];
  MAKE_MATRIX_PTR(R, _R, 2);
  R[0][0] = measurement_noise[0];
  R[0][1] = 0.0f;
  R[1][0] = 0.0f;
  R[1][1] = measurement_noise[1];

  //Variables during kalman computation:
  float _Xpredict[1][2];
  MAKE_MATRIX_PTR(Xpredict, _Xpredict, 2);
  float _Xnext[1][2];
  MAKE_MATRIX_PTR(Xnext, _Xnext, 2);

  float _Ppredict[2][2];
  MAKE_MATRIX_PTR(Ppredict, _Ppredict, 2);
  float _Pnext[2][2];
  MAKE_MATRIX_PTR(Pnext, _Pnext, 2);

  float _K[2][2];
  MAKE_MATRIX_PTR(K, _K, 2);

  float _eye[2][2];
  MAKE_MATRIX_PTR(eye, _eye, 2);
  eye[0][0] = 1.0f;
  eye[0][1] = 0.0f;
  eye[1][0] = 0.0f;
  eye[1][1] = 1.0f;

  float _temp_mat[2][2];
  MAKE_MATRIX_PTR(temp_mat, _temp_mat, 2);
  float _temp_mat2[2][2];
  MAKE_MATRIX_PTR(temp_mat2, _temp_mat2, 2)
  float _temp_mat3[2][2];
  MAKE_MATRIX_PTR(temp_mat3, _temp_mat3, 2)

  float _temp_vec[1][2];
  MAKE_MATRIX_PTR(temp_vec, _temp_vec, 2);
  float _temp_vec2[1][2];
  MAKE_MATRIX_PTR(temp_vec2, _temp_vec2, 2);


  //......................KALMAN FILTER ..................... //


  // 1. calculate state predict

  //Xpredict = G* Xprev;
  float_mat_mul(Xpredict, G, Xprev, 2, 2, 1);
  //......................KALMAN FILTER ..................... //

  // 2. calculate covariance predict

  // Ppredict = G*Pprevious*Gtrans + Q
  //...Pprevious*Gtrans...
  float_mat_mul(temp_mat, Pprevious, Gtrans, 2, 2, 2);
  //G*Pprevious*Gtrans...
  float_mat_mul(temp_mat2, G, temp_mat, 2, 2, 2);
  //G*Pprevious*Gtrans+Q
  float_mat_sum(Ppredict, temp_mat2, Q, 2, 2);

  // 3. Calculate Kalman gain

  // K = Ppredict * Htrans /( H * Ppredict * Htrans + R)
  // ... Ppredict * Htrans ...
  float_mat_mul(temp_mat, Ppredict, Htrans, 2, 2, 2);
  //... H * Predict * Htrans
  float_mat_mul(temp_mat2, H, temp_mat, 2, 2, 2);
  //..( H * Ppredict * Htrans + R)
  float_mat_sum(temp_mat3, temp_mat2, R, 2, 2);
  //...inv( H * Ppredict * Htrans + R)
  //TODO: Make a matrix inverse function for more than 2x2 matrix!

  float det_temp2 = 1 / (temp_mat3[0][0] * temp_mat3[1][1] - temp_mat3[0][1] * temp_mat3[1][0]);
  temp_mat2[0][0] =  det_temp2 * (temp_mat3[1][1]);
  temp_mat2[0][1] =  det_temp2 * (-1 * temp_mat3[1][0]);
  temp_mat2[1][0] =  det_temp2 * (-1 * temp_mat3[0][1]);
  temp_mat2[1][1] =  det_temp2 * (temp_mat3[0][0]);
  // K = Ppredict * Htrans / *inv( H * Ppredict * Htrans + R)
  float_mat_mul(K, temp_mat, temp_mat2, 2, 2, 2);

  // 4. Update state estimate

  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  // ... Htrans * Xpredict)
  float_mat_mul(temp_vec, Htrans, Xpredict, 2, 2, 1);

  //... (Z - Htrans * Xpredict)
  float_mat_diff(temp_vec2, Z, temp_vec, 2, 1);

  // ... K *(Z - Htrans * Xpredict)
  float_mat_mul(temp_vec, K, temp_vec2, 2, 2, 1);


  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  float_mat_sum(Xnext, Xpredict, temp_vec, 2, 1);

  // 5. Update covariance matrix

  // Pnext = (eye(2) - K*H)*P_predict
  // ...K*H...
  float_mat_mul(temp_mat, K, H, 2, 2, 2);
  //(eye(2) - K*H)
  float_mat_diff(temp_mat2, eye, temp_mat, 2, 2);
  // Pnext = (eye(2) - K*H)*P_predict
  float_mat_mul(Pnext, temp_mat2, Ppredict, 2, 2, 2);


  //save values for next state
  covariance[0] = Pnext[0][0];
  covariance[1] = Pnext[0][1];;
  covariance[2] = Pnext[1][0];;
  covariance[3] = Pnext[1][1];;


  state[0] = Xnext[0][0];
  state[1] = Xnext[1][0];
}

/**
 * A simple linear3DD kalman filter, computed using floats and matrices. To be used for vision task like optical flow, tracking markers etc.
 * @param[in] *model  The process model for the prediction of the next state (array size of [1 x 9])
 * @param[in] *measurements  Values of the measurements (array size of [1 x 3])
 * @param[in] *covariance  Covariance matrix of previous iteration (array size of [1 x 9])
 * @param[out] *covariance  Updated Covariance matrix (array size of [1 x 9])
 * @param[in] *state  Previous state vector with estimates (array size of [1 x 3])
 * @param[out] *state  Updated state vector with estimates (array size of [1 x 3])
 * @param[in] *measurement_noise  Expected variance of the noise of the measurements (array size of [1 x 3])
 * @param[in] *process_noise  Expected variance of the noise of the process model (array size of [1 x 3])
 */
void kalman_filter_linear_3D_float(float *model, float *measurements, float *covariance, float *state
                                   , float *process_noise, float *measurement_noise)
{
  //......................Preparation kalman filter ..................... //

  // todo make initialization tools for the matrixes
  // process model (linear)

  int x, y, ind;

  float _G[2][2];
  MAKE_MATRIX_PTR(G, _G, 3);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { ind = x + y;  G[x][y] = model[ind]; }}

  // transpose of G
  float _Gtrans[2][2];
  MAKE_MATRIX_PTR(Gtrans, _Gtrans, 2);
  float_mat_copy(Gtrans, G, 2, 2);
  float_mat_transpose(Gtrans, 2);

  // Observation model (linear)
  // note: right now both velocity and acceleration are observed
  float _H[3][3];
  MAKE_MATRIX_PTR(H, _H, 3);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { if (x == y) { H[x][y] = 1.0f; } else { H[x][y] = 0.0f; } }}

  //transpose of H
  float _Htrans[3][3];
  MAKE_MATRIX_PTR(Htrans, _Htrans, 3);
  float_mat_copy(Htrans, H, 3, 3);
  float_mat_transpose(Htrans, 3);

  //Previous state
  float _Xprev[1][3];
  MAKE_MATRIX_PTR(Xprev, _Xprev, 3);
  for (x = 0; x < 3; x++) {  Xprev[x][0] = state[x]; }

  //Previous covariance
  float _Pprevious[3][3];
  MAKE_MATRIX_PTR(Pprevious, _Pprevious, 3);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { ind = x + y;  Pprevious[x][y] = covariance[ind]; }}

  //measurements;
  float _Z[1][3];
  MAKE_MATRIX_PTR(Z, _Z, 3);
  for (x = 0; x < 3; x++) {  Z[x][0] = measurements[x]; }


  //Process noise model
  float _Q[3][3];
  MAKE_MATRIX_PTR(Q, _Q, 3);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { if (x == y) { Q[x][y] = process_noise[x]; } else { Q[x][y] = 0.0f; } }}

  //measurement nosie model
  float _R[3][3];
  MAKE_MATRIX_PTR(R, _R, 2);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { if (x == y) { R[x][y] = measurement_noise[x]; } else { R[x][y] = 0.0f; } }}


  //Variables during kalman computation:
  float _Xpredict[1][3];
  MAKE_MATRIX_PTR(Xpredict, _Xpredict, 3);
  float _Xnext[1][3];
  MAKE_MATRIX_PTR(Xnext, _Xnext, 3);

  float _Ppredict[3][3];
  MAKE_MATRIX_PTR(Ppredict, _Ppredict, 3);
  float _Pnext[3][3];
  MAKE_MATRIX_PTR(Pnext, _Pnext, 3);

  float _K[3][3];
  MAKE_MATRIX_PTR(K, _K, 3);

  float _eye[3][3];
  MAKE_MATRIX_PTR(eye, _eye, 3);
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { if (x == y) { eye[x][y] = 1.0f; } else { eye[x][y] = 0.0f; } }}


  float _temp_mat[3][3];
  MAKE_MATRIX_PTR(temp_mat, _temp_mat, 3);
  float _temp_mat2[3][3];
  MAKE_MATRIX_PTR(temp_mat2, _temp_mat2, 3)
  float _temp_mat3[3][3];
  MAKE_MATRIX_PTR(temp_mat3, _temp_mat3, 3)

  float _temp_vec[1][3];
  MAKE_MATRIX_PTR(temp_vec, _temp_vec, 3);
  float _temp_vec2[1][3];
  MAKE_MATRIX_PTR(temp_vec2, _temp_vec2, 3);


  //......................KALMAN FILTER ..................... //


  // 1. calculate state predict

  //Xpredict = G* Xprev;
  float_mat_mul(Xpredict, G, Xprev, 3, 3, 1);
  //......................KALMAN FILTER ..................... //

  // 2. calculate covariance predict

  // Ppredict = G*Pprevious*Gtrans + Q
  //...Pprevious*Gtrans...
  float_mat_mul(temp_mat, Pprevious, Gtrans, 3, 3, 3);
  //G*Pprevious*Gtrans...
  float_mat_mul(temp_mat2, G, temp_mat, 3, 3, 3);
  //G*Pprevious*Gtrans+Q
  float_mat_sum(Ppredict, temp_mat2, Q, 3, 3);

  // 3. Calculate Kalman gain

  // K = Ppredict * Htrans /( H * Ppredict * Htrans + R)
  // ... Ppredict * Htrans ...
  float_mat_mul(temp_mat, Ppredict, Htrans, 3, 3, 3);
  //... H * Predict * Htrans
  float_mat_mul(temp_mat2, H, temp_mat, 3, 3, 3);
  //..( H * Ppredict * Htrans + R)
  float_mat_sum(temp_mat3, temp_mat2, R, 3, 2);
  //...inv( H * Ppredict * Htrans + R)
  //TODO: Make a matrix inverse function for more than 2x2 matrix!

  MAT_INV33(temp_mat2, temp_mat3);

//  float det_temp2 = 1 / (temp_mat3[0][0] * temp_mat3[1][1] - temp_mat3[0][1] * temp_mat3[1][0]);
//  temp_mat2[0][0] =  det_temp2 * (temp_mat3[1][1]);
//  temp_mat2[0][1] =  det_temp2 * (-1 * temp_mat3[1][0]);
//  temp_mat2[1][0] =  det_temp2 * (-1 * temp_mat3[0][1]);
//  temp_mat2[1][1] =  det_temp2 * (temp_mat3[0][0]);
  // K = Ppredict * Htrans / *inv( H * Ppredict * Htrans + R)
  float_mat_mul(K, temp_mat, temp_mat2, 3, 3, 3);

  // 4. Update state estimate

  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  // ... Htrans * Xpredict)
  float_mat_mul(temp_vec, Htrans, Xpredict, 3, 3, 1);

  //... (Z - Htrans * Xpredict)
  float_mat_diff(temp_vec2, Z, temp_vec, 3, 1);

  // ... K *(Z - Htrans * Xpredict)
  float_mat_mul(temp_vec, K, temp_vec2, 3, 3, 1);


  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  float_mat_sum(Xnext, Xpredict, temp_vec, 3, 1);

  // 5. Update covariance matrix

  // Pnext = (eye(2) - K*H)*P_predict
  // ...K*H...
  float_mat_mul(temp_mat, K, H, 3, 3, 3);
  //(eye(2) - K*H)
  float_mat_diff(temp_mat2, eye, temp_mat, 3, 3);
  // Pnext = (eye(2) - K*H)*P_predict
  float_mat_mul(Pnext, temp_mat2, Ppredict, 3, 3, 3);


  //save values for next state
  for (x = 0; x < 3; x++) { for (y = 0; y < 3; y++) { ind = x + y;  covariance[ind] = Pnext[x][y]; }}
  for (x = 0; x < 3; x++) {   state[x] = Xnext[x][0]; }


}

