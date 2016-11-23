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

#ifndef STEREOCAM2STATE_CAM_FORWARD
#define STEREOCAM2STATE_CAM_FORWARD 1
#endif

#include "subsystems/datalink/telemetry.h"

#include "modules/computer_vision/lib/filters/kalman_filter_vision.h"

static abi_event gps_ev;
struct NedCoor_f opti_vel;
struct FloatVect3 velocity_rot_gps;

#include "filters/median_filter.h"
#include "filters/low_pass_filter.h"

struct MedianFilterF medianfilter_x, medianfilter_y;
static Butterworth2LowPass butterfilter_x, butterfilter_y;

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  opti_vel.x = (float)(gps_s->ecef_vel.x) / 100;
  opti_vel.y = (float)(gps_s->ecef_vel.y) / 100;
  opti_vel.z = (float)(gps_s->ecef_vel.z) / 100;
}


void stereocam_to_state(void);

void stereo_to_state_init(void)
{
  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

  init_median_filter_f(&medianfilter_x);
  init_median_filter_f(&medianfilter_y);
  init_butterworth_2_low_pass(&butterfilter_x, 14, 1. / 23, 0.0f);
  init_butterworth_2_low_pass(&butterfilter_y, 14, 1. / 23, 0.0f);


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

//optitrack data
  struct FloatVect3 velocity_rot_gps;
  struct Int32Vect3 velocity_rot_gps_int;
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_vel);

  //TODO: give velocity body in z direction?

  // Median filter and 2nd order butterworth filter
  float vel_body_x_median_filter = update_median_filter_f(&medianfilter_x, vel_body_x);
  float vel_body_y_median_filter = update_median_filter_f(&medianfilter_y, vel_body_y);

  float vel_body_x_butter_filter = update_butterworth_2_low_pass(&medianfilter_x, vel_body_x_median_filter);
  float vel_body_y_butter_filter = update_butterworth_2_low_pass(&medianfilter_y, vel_body_y_median_filter);


  // KALMAN filter
  static float vel_body_x_filter = 0;
  static float vel_body_y_filter = 0;

  bool kalman_filter_on = true;
  float kalman_filter_process_noise = 0.001f;


  // KALMAN filter on velocity
  float measurement_noise[2] = {0.5f, 1.0f};
  static bool reinitialize_kalman = true;

  static uint8_t wait_filter_counter = 0;

  if (kalman_filter_on == true) {
    if (wait_filter_counter > 50) {

      // Get accelerometer values rotated to body axis
      struct FloatVect3 accel_meas_body;
      float psi = stateGetNedToBodyEulers_f()->psi;
      accel_meas_body.x =  cosf(-psi) * stateGetAccelNed_f()->x - sinf(-psi) * stateGetAccelNed_f()->y;
      accel_meas_body.y = sinf(-psi) * stateGetAccelNed_f()->x + cosf(-psi) * stateGetAccelNed_f()->y;


      float acceleration_measurement[2];
      acceleration_measurement[0] = accel_meas_body.x;
      acceleration_measurement[1] = accel_meas_body.y;


      if (!(fabs(vel_body_x) > 1.0 || fabs(vel_body_x) > 1.0)) {

        vel_body_x_filter = vel_body_x;
        vel_body_y_filter = vel_body_y;

        kalman_edgeflow_stereocam(&vel_body_x_filter, &vel_body_y_filter, acceleration_measurement, 26.0f,
                                  measurement_noise, kalman_filter_process_noise, reinitialize_kalman);

        if (reinitialize_kalman) {
          reinitialize_kalman = false;
        }
      }

    } else {
      wait_filter_counter++;
    }
  } else {
    reinitialize_kalman = true;
  }


  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                              vel_body_x_filter,
                              vel_body_y_filter,
                              0.0f,
                              0.3f
                             );


  // EDGEFLOW_STEREOCAM telemetry messages, with some values replaced by 0
  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  //TODO add body rotated optitrackc measurements here
  DOWNLINK_SEND_EDGEFLOW_STEREOCAM(DefaultChannel, DefaultDevice, &now_ts, &vel_x_global_int, &vel_y_global_int,
                                   &vel_z_global_int,
                                   &vel_x_pixelwise_int, &vel_z_pixelwise_int, &vel_body_x, &vel_body_y, &vel_body_x_filter, &vel_body_y_filter,
                                   &velocity_rot_gps.x, &velocity_rot_gps.y);

#endif

}

/**
 * Filter the velocity with a simple linear kalman filter, together with the accelerometers
 * @param[in] *velocity_x  Velocity in x direction of body fixed coordinates
 * @param[in] *velocity_y  Belocity in y direction of body fixed coordinates
 * @param[in] *acceleration_measurement  Measurements of the accelerometers
 * @param[in] fps  Frames per second
 * @param[in] *measurement_noise  Expected variance of the noise of the measurements
 * @param[in] *measurement_noise  Expected variance of the noise of the model prediction
 * @param[in] reinitialize_kalman  Boolean to reinitialize the kalman filter
 */
void kalman_edgeflow_stereocam(float *velocity_x, float *velocity_y, float *acceleration_measurement, float fps,
                               float *measurement_noise, float kalman_process_noise, bool reinitialize_kalman)
{
  // Initialize variables
  static float covariance_x[4], covariance_y[4], state_estimate_x[2], state_estimate_y[2];
  float measurements_x[2], measurements_y[2];

  if (reinitialize_kalman) {
    state_estimate_x[0] = 0.0f;
    state_estimate_x[1] = 0.0f;
    covariance_x[0] = 1.0f;
    covariance_x[1] = 1.0f;
    covariance_x[2] = 1.0f;
    covariance_x[3] = 1.0f;

    state_estimate_y[0] = 0.0f;
    state_estimate_y[1] = 0.0f;
    covariance_y[0] = 1.0f;
    covariance_y[1] = 1.0f;
    covariance_y[2] = 1.0f;
    covariance_y[3] = 1.0f;
  }

  /*Model for velocity estimation
   * state = [ velocity; acceleration]
   * Velocity_prediction = last_velocity_estimate + acceleration * dt
   * Acceleration_prediction = last_acceleration
   * model = Jacobian([vel_prediction; accel_prediction],state)
   *       = [1 dt ; 0 1];
   * */
  float model[4] =  {1.0f, 1.0f / fps , 0.0f , 1.0f};
  float process_noise[2] = {kalman_process_noise, kalman_process_noise};

  // Measurements from velocity_x of optical flow and acceleration directly from scaled accelerometers
  measurements_x[0] = *velocity_x;
  measurements_x[1] = acceleration_measurement[0];

  measurements_y[0] = *velocity_y;
  measurements_y[1] = acceleration_measurement[1];

  // 2D linear kalman filter
  kalman_filter_linear_2D_float(model, measurements_x, covariance_x, state_estimate_x, process_noise, measurement_noise);
  kalman_filter_linear_2D_float(model, measurements_y, covariance_y, state_estimate_y, process_noise, measurement_noise);

  *velocity_x = state_estimate_x[0];
  *velocity_y = state_estimate_y[0];
}

/*void kalman_edgeflow_stereocam(float fps, float vel_body_x, float vel_body_y, float *vel_body_x_filter,
                               float *vel_body_y_filter)
{
  //------------------------- KALMAN filter
  // Parameters
  float measurement_noise_edgeflow = 0.5;
  static uint8_t wait_filter_counter = 0;

  static float state_estimate_x[2] = {0.0f, 0.0f};
  static float covariance_x[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float measurements_x[2];

  static float state_estimate_y[2] = {0.0f, 0.0f};
  static float covariance_y[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  static bool reinitialize_kalman = true;
  bool kalman_filter_on = true;

  float kalman_process_noise = 0.000f;


  // get accelerometer values from state and rotate to heading of drone

  float psi = stateGetNedToBodyEulers_f()->psi;
  float accelx =  cosf(-psi) * stateGetAccelNed_f()->x - sinf(-psi) * stateGetAccelNed_f()->y;
  float accely = sinf(-psi) * stateGetAccelNed_f()->x + cosf(-psi) * stateGetAccelNed_f()->y;


  // Wait for a certain amount of time (so that edgeflow has some time to run)
  if (kalman_filter_on == true) {
    if (wait_filter_counter > 100) {



      // Bound measurments if above a certain value.
      if (!(fabs(vel_body_x) > 1.0 || fabs(vel_body_y) > 1.0)) {

//////////////////////
        static float covariance_x[4], covariance_y[4], state_estimate_x[2], state_estimate_y[2];
        float measurements_x[2], measurements_y[2];

        float measurement_noise[2] = {measurement_noise_edgeflow, 1.0f};

        float model[4] =  {1.0f, 1.0f / fps , 0.0f , 1.0f};
        float process_noise[2] = {kalman_process_noise, kalman_process_noise};


        if (reinitialize_kalman) {
          state_estimate_x[0] = 0.0f;
          state_estimate_x[1] = 0.0f;
          covariance_x[0] = 1.0f;
          covariance_x[1] = 1.0f;
          covariance_x[2] = 1.0f;
          covariance_x[3] = 1.0f;

          state_estimate_y[0] = 0.0f;
          state_estimate_y[1] = 0.0f;
          covariance_y[0] = 1.0f;
          covariance_y[1] = 1.0f;
          covariance_y[2] = 1.0f;
          covariance_y[3] = 1.0f;
        }

        // Measurements from velocity_x of optical flow and acceleration directly from scaled accelerometers
        measurements_x[0] = vel_body_x;
        measurements_x[1] = ACCEL_FLOAT_OF_BFP(accelx);
        measurements_y[0] = vel_body_y;
        measurements_y[1] = ACCEL_FLOAT_OF_BFP(accely);

        kalman_filter_linear_2D_float(model,  measurements_x, covariance_x, state_estimate_x
                                      , process_noise, measurement_noise);
        kalman_filter_linear_2D_float(model,  measurements_y, covariance_y, state_estimate_y
                                      , process_noise, measurement_noise);

        *vel_body_x_filter = state_estimate_x[0];
        *vel_body_y_filter = state_estimate_y[0];
        ////////////////////////////////////////

        if (reinitialize_kalman) {
          reinitialize_kalman = false;
        }

      }

    } else {
      wait_filter_counter++;
    }
  } else {
    reinitialize_kalman = true;
  }


}*/
