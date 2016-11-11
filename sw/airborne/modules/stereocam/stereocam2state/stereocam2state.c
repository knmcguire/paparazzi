/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.c"
 * @author Kimberly McGuire
 * This module sends the data retreived from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#include "modules/stereocam/stereocam2state/stereocam2state.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"

#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#ifndef STEREOCAM2STATE_RECEIVED_DATA_TYPE
#define STEREOCAM2STATE_RECEIVED_DATA_TYPE 0
#endif

#include "subsystems/datalink/telemetry.h"

void stereocam_to_state(void);

void stereo_to_state_init(void)
{

}

void stereo_to_state_periodic(void)
{
  if (stereocam_data.fresh) {
    stereocam_to_state();
    stereocam_data.fresh = 0;
  }
}

void stereocam_to_state(void)
{
  int16_t RES = 100;

  // Get info from stereocam data
  // 0 = stereoboard's #define SEND_EDGEFLOW
#if STEREOCAM2STATE_RECEIVED_DATA_TYPE == 0
  // opticflow and divergence (unscaled with depth
  int16_t div_x = (int16_t)stereocam_data.data[0] << 8;
  div_x |= (int16_t)stereocam_data.data[1];
  int16_t flow_x = (int16_t)stereocam_data.data[2] << 8;
  flow_x |= (int16_t)stereocam_data.data[3];
  int16_t div_y = (int16_t)stereocam_data.data[4] << 8;
  div_y |= (int16_t)stereocam_data.data[5];
  int16_t flow_y = (int16_t)stereocam_data.data[6] << 8;
  flow_y |= (int16_t)stereocam_data.data[7];

  float fps = (float)stereocam_data.data[9];
  int8_t agl = stereocam_data.data[8]; // in cm

  // velocity global
  int16_t vel_x_global_int = (int16_t)stereocam_data.data[10] << 8;
  vel_x_global_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_y_global_int = (int16_t)stereocam_data.data[12] << 8;
  vel_y_global_int |= (int16_t)stereocam_data.data[13];
  int16_t vel_z_global_int = (int16_t)stereocam_data.data[14] << 8;
  vel_z_global_int |= (int16_t)stereocam_data.data[15];

  // Velocity Pixelwise
  int16_t vel_x_pixelwise_int = (int16_t)stereocam_data.data[16] << 8;
  vel_x_pixelwise_int |= (int16_t)stereocam_data.data[17];
  int16_t vel_z_pixelwise_int = (int16_t)stereocam_data.data[18] << 8;
  vel_z_pixelwise_int |= (int16_t)stereocam_data.data[19];

  // Velocity (global pixelwise) in camera coordinates
  struct Int16Vect3 vel_pixelwise, vel_global;
  vel_pixelwise.x = vel_x_pixelwise_int;
  vel_pixelwise.z = vel_z_pixelwise_int;

  vel_global.x = vel_x_global_int;
  vel_global.y = vel_y_global_int;
  vel_global.z = vel_z_global_int;

  // Derotate velocity and transform from frame to body coordinates
  // TODO: send resolution directly from stereocam
  float vel_body_x = 0;
  float vel_body_y = 0;

#if STEREOCAM2STATE_CAM_FORWARD == 1
  vel_body_x = - (float)vel_pixelwise.z / RES;
  vel_body_y = (float)vel_pixelwise.x / RES;
#else
  vel_body_x = - (float)vel_global.x / RES;
  vel_body_y = (float)vel_global.y / RES;
#endif

  //TODO: give velocity body in z direction?

  // KALMAN filter
  float vel_body_x_filter = 0;
  float vel_body_y_filter = 0;
  kalman_edgeflow_stereocam(vel_body_x, vel_body_y, &vel_body_x_filter, &vel_body_y_filter);

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  if (!(abs(vel_body_x) > 0.5 || abs(vel_body_x) > 0.5)) {
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                vel_body_x_filter,
                                vel_body_y_filter,
                                0.0f,
                                0.3f
                               );
  }

  // EDGEFLOW_STEREOCAM telemetry messages, with some values replaced by 0
  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  //TODO add body rotated optitrackc measurements here
  DOWNLINK_SEND_EDGEFLOW_STEREOCAM(DefaultChannel, DefaultDevice, fps, &vel_x_global_int, &vel_y_global_int,
                                   &vel_z_global_int,
                                   &vel_x_pixelwise_int, &vel_y_pixelwise_int, &vel_body_x, &vel_body_y, &vel_body_x_filter, &vel_body_y_filter,
                                   &dummy_float, &dummy_float)

#endif

}

void kalman_edgeflow_stereocam(float vel_body_x, float vel_body_y, float *vel_body_x_filter, float *vel_body_y_filter)
{
  //------------------------- KALMAN filter
  // Parameters
  float measurement_noise_edgeflow = 0.5;
  static uint8_t wait_filter_counter = 0;
  static float previous_state_x[2] = {0.0f, 0.0f};
  static float covariance_x[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float measurements_x[2];

  static float previous_state_y[2] = {0.0f, 0.0f};
  static float covariance_y[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float measurements_y[2];

  float process_noise[2] = {0.001f, 0.001f};
  float measurement_noise[2] = {measurement_noise_edgeflow, 1.0f};

  // get accelerometer values
  struct Int32Vect3 acc_meas_body;
  struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
  int32_rmat_transp_vmult(&acc_meas_body, body_to_imu_rmat, &imu.accel);

  // Wait for a certain amount of time (so that edgeflow has some time to run)
  if (wait_filter_counter > 100) {

    measurements_x[0] = vel_body_x;
    measurements_x[1] = ACCEL_FLOAT_OF_BFP(acc_meas_body.x);
    measurements_y[0] = vel_body_y;
    measurements_y[1] = ACCEL_FLOAT_OF_BFP(acc_meas_body.y);

    // Bound measurments if above a certain value.
    if (!(fabs(vel_body_x) > 1.0 || fabs(vel_body_y) > 1.0)) {

      kalman_filter(measurements_x, covariance_x,
                    previous_state_x, process_noise, measurement_noise, fps);
      kalman_filter(measurements_y, covariance_y,
                    previous_state_y, process_noise, measurement_noise, fps);
    }

    *vel_body_x_filter = previous_state_x[0];
    *vel_body_y_filter =  previous_state_y[0];

  } else {
    wait_filter_counter++;
  }

}
