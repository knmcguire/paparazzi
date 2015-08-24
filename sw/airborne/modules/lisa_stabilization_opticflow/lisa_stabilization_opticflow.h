/*
 * Copyright (C) K N McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/lisa_stabilization_opticflow/lisa_stabilization_opticflow.h"
 * @author K N McGuire
 * Guidance stabilization based on optical flow detected with a downfacing stereocam on a Lisa s nanoquad
 */

#ifndef LISA_STABILIZATION_OPTICFLOW_H
#define LISA_STABILIZATION_OPTICFLOW_H


#include "std.h"
#include "math/pprz_algebra_int.h"


struct opticflow_result_t {
  float fps;              ///< Frames per second of the optical flow calculation
  uint16_t corner_cnt;    ///< The amount of coners found by FAST9
  uint16_t tracked_cnt;   ///< The amount of tracked corners

  int16_t flow_x;         ///< Flow in x direction from the camera (in subpixels)
  int16_t flow_y;         ///< Flow in y direction from the camera (in subpixels)
  int16_t flow_der_x;     ///< The derotated flow calculation in the x direction (in subpixels)
  int16_t flow_der_y;     ///< The derotated flow calculation in the y direction (in subpixels)

  float vel_x;            ///< The velocity in the x direction
  float vel_y;            ///< The velocity in the y direction

  float div_size;         ///< Divergence as determined with the size_divergence script
};


extern int phi_gain_p;
extern int phi_gain_i;
extern int theta_gain_p;
extern int theta_gain_i;


//extern struct opticflow_result_t opticflow_result;

extern void lisa_stab_of_init(void);
extern void lisa_stab_of_start(void);
extern void lisa_stab_of_periodic(void);
extern void send_edge_flow_velocity(void);
void velocity_calculate(struct opticflow_result_t *opticflow_result);
float simpleKalmanFilter(float* cov,float previous_est, float current_meas,float Q,float R);


// Implement own Horizontal loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);


#endif

