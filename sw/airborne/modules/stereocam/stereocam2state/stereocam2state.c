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

float distance_stereo = 2.0f;


struct MedianFilterInt medianfilter_x, medianfilter_y, medianfilter_dist;



#include "subsystems/radio_control.h"


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

  init_median_filter(&medianfilter_x);
  init_median_filter(&medianfilter_y);
  init_median_filter(&medianfilter_dist);

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
  struct Int16Vect3 vel_pixelwise;
  vel_pixelwise.x = vel_x_pixelwise_int;
  vel_pixelwise.z = vel_z_pixelwise_int;

/*  struct Int16Vect3 vel_global;
  vel_global.x = vel_x_global_int;
  vel_global.y = vel_y_global_int;
  vel_global.z = vel_z_global_int;*/

  // Derotate velocity and transform from frame to body coordinates
  // TODO: send resolution directly from stereocam
  float vel_body_x = 0;
  float vel_body_y = 0;

#if STEREOCAM2STATE_CAM_FORWARD == 1
  vel_body_x = (float)vel_pixelwise.z / RES;
  vel_body_y = -(float)vel_pixelwise.x / RES;
#else
  vel_body_x = - (float)vel_global.x / RES;
  vel_body_y = (float)vel_global.y / RES;
#endif

//optitrack data
/*
  struct FloatVect3 velocity_rot_gps;
  struct Int32Vect3 velocity_rot_gps_int;
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_vel);
*/

  //TODO: give velocity body in z direction?

  // Median filter and 2nd order butterworth filter
  float vel_body_x_median_filter = (float)update_median_filter(&medianfilter_x, (int32_t)(vel_body_x * 100)) / 100;
  float vel_body_y_median_filter = (float)update_median_filter(&medianfilter_y, (int32_t)(vel_body_y * 100)) / 100;

  distance_stereo = (float)update_median_filter(&medianfilter_dist, (int32_t)(agl)) / 10; ;


  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                              vel_body_x_median_filter,
                              vel_body_y_median_filter,
                              0.0f,
                              0.3f
                             );

  /*
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                vel_body_x,
                                vel_body_y,
                                0.0f,
                                0.3f
                               );
  */

  // Reusing the OPTIC_FLOW_EST telemetry messages, with some values replaced by 0

  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;

  // DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &dummy_int16, &dummy_int16,
		  // &vel_body_x_median_filter, &vel_body_y_median_filter,&dummy_float, &dummy_float, &dummy_float);

#endif

}
