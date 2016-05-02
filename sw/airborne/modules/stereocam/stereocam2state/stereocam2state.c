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

#include "filters/median_filter.h"

//#include "subsystems/gps.h"

#ifndef SENDER_ID
#define SENDER_ID 1
#endif

/** ABI binding for gps messages*/
#ifndef STEREOCAM_GPS_ID
#define STEREOCAM_GPS_ID ABI_BROADCAST
#endif
static abi_event gps_ev;


/** For extra functionality for derotation of velocity to state measurements*/
#ifndef USE_DEROTATION_OPTICFLOW
#define USE_DEROTATION_OPTICFLOW FALSE
#endif
#ifndef STATE_MEASURE_OPTICFLOW
#define STATE_MEASURE_OPTICFLOW TRUE
#endif

static float prev_phi;
static float prev_theta;

struct GpsStereoCam gps_stereocam;
struct MedianFilterInt filter_x, filter_y;


void stereocam_to_state(float dphi, float dtheta);

static void stereocam_gps_cb(uint8_t sender_id __attribute__((unused)),
                             uint32_t stamp __attribute__((unused)),
                             struct GpsState *gps_s)
{
  gps_stereocam.ecef_vel.x = gps_s->ecef_vel.x;
  gps_stereocam.ecef_vel.y = gps_s->ecef_vel.y;
  gps_stereocam.ecef_vel.z = gps_s->ecef_vel.z;

  gps_stereocam.ecef_pos.x = gps_s->ecef_pos.x;
  gps_stereocam.ecef_pos.y = gps_s->ecef_pos.y;
  gps_stereocam.ecef_pos.z = gps_s->ecef_pos.z;


}

void stereo_to_state_init(void)
{
  //subscribe to GPS abi-messages for state measurements
  AbiBindMsgGPS(STEREOCAM_GPS_ID, &gps_ev, stereocam_gps_cb);
  init_median_filter(&filter_x);
  init_median_filter(&filter_y);

}
void stereo_to_state_periodic(void)
{

  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
    float phi = stateGetNedToBodyEulers_f()->phi;
    float theta = stateGetNedToBodyEulers_f()->theta;
    float dphi =  phi - prev_phi;
    float dtheta = theta - prev_theta;

    stereocam_to_state(dphi, dtheta);

    prev_theta = theta;
    prev_phi = phi;
  }
}

void stereocam_to_state(float dphi, float dtheta)
{
  uint8_t frequency = 0;

  // Get info from stereocam data
  // TODO: when ready, remove all floats in this
  float vel_hor = ((float)(stereocam_data.data[8]) - 127) / 100;
  float vel_ver = ((float)(stereocam_data.data[9]) - 127) / 100;
  float vel_x = 0;
  float vel_y = 0;

  float alt_stereo = ((float)(stereocam_data.data[4])) / 10;
  //rotate to body coordinates
  vel_x = - (vel_ver);
  vel_y = (vel_hor);

  // Retreive high resolution data from stereocam_data.data
  int16_t vel_x_int = 0;
  int16_t vel_y_int = 0;
  int16_t vel_hor_int = (int16_t)stereocam_data.data[10] << 8;
  vel_hor_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_ver_int = (int16_t)stereocam_data.data[12] << 8;
  vel_ver_int |= (int16_t)stereocam_data.data[13];
  //rotate to body coordinates
  vel_x_int = - (vel_ver_int);
  vel_y_int = (vel_hor_int);

  // Calculate derotated velocity
#if USE_DEROTATION_OPTICFLOW
  float agl_stereo = (float)(stereocam_data.data[4]) / 10;

  float diff_flow_hor = dtheta * 128 / 1.04;
  float diff_flow_ver = dphi * 96 / 0.785;

  float diff_vel_hor = diff_flow_hor * agl_stereo * 12 * 1.04 / 128;
  float diff_vel_ver = diff_flow_ver * agl_stereo * 12 * 0.785 / 96;

  vel_x = - (vel_ver - diff_vel_ver);
  vel_y = (vel_hor - diff_vel_hor);
#endif




#if STATE_MEASURE_OPTICFLOW
  // Calculate velocity in body fixed coordinates from opti-track and the state filter
  struct NedCoor_f coordinates_speed_state;
  coordinates_speed_state.x = stateGetSpeedNed_f()->x;
  coordinates_speed_state.y = stateGetSpeedNed_f()->y;
  coordinates_speed_state.z = stateGetSpeedNed_f()->z;

  struct NedCoor_f opti_vel, opti_pos;
  struct EcefCoor_f ecef_pos, ecef_vel;
  ECEF_FLOAT_OF_BFP(ecef_vel, gps_stereocam.ecef_vel);
  ned_of_ecef_vect_f(&opti_vel, &state.ned_origin_f, &ecef_vel);

  //opti_vel.x = (float)(gps_stereocam.ecef_vel.x) / 100;
  //opti_vel.y = (float)(gps_stereocam.ecef_vel.y) / 100;
  //opti_vel.z = -(float)(gps_stereocam.ecef_vel.z) / 100;


  opti_pos.x = (float)(gps_stereocam.ecef_pos.x) / 100;
  opti_pos.y = (float)(gps_stereocam.ecef_pos.y) / 100;
  opti_pos.z = -(float)(gps_stereocam.ecef_pos.z) / 100;


  struct FloatVect3 velocity_rot_state;
  struct FloatVect3 velocity_rot_gps;

  float_rmat_vmult(&velocity_rot_state , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&coordinates_speed_state);
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_vel);

  float vel_x_opti = ((float)(velocity_rot_gps.x));
  //float vel_y_opti = ((float)(velocity_rot_gps.y));
  float vel_y_opti = ((float)(velocity_rot_gps.y));

  // Calculate velocity error
  float vel_x_error = vel_x_opti - vel_x;
  float vel_y_error = vel_y_opti - vel_y;

 /* stereocam_data.data[8] = (uint8_t)((vel_x * 10) + 127); // dm/s
  stereocam_data.data[9] = (uint8_t)((vel_y * 10) + 127); // dm/s
  stereocam_data.data[19] = (uint8_t)((vel_x_opti) * 10 + 127); // dm/s
  stereocam_data.data[20] = (uint8_t)((vel_y_opti) * 10 + 127); // dm/s
  stereocam_data.data[21] = (uint8_t)((vel_x_error) * 10 + 127); // dm/s
  stereocam_data.data[22] = (uint8_t)((vel_y_error) * 10 + 127); // dm/s
  stereocam_data.data[23] = (uint8_t)((velocity_rot_state.x) * 10 + 127); // dm/s
  stereocam_data.data[24] = (uint8_t)((velocity_rot_state.y) * 10 + 127); // dm/s*/

  int16_t vel_x_state = (int16_t)(vel_x_opti  * 100);
  int16_t vel_y_state = (int16_t)(vel_y_opti  * 100);

  //todo: retrieve optitrack in int16
  int16_t vel_x_opti_int = (int16_t)(opti_vel.x * 100);
  int16_t vel_y_opti_int = (int16_t)(opti_vel.y * 100);
  int16_t pos_x_opti_int = (int16_t)(gps_stereocam.ecef_pos.x);
  int16_t pos_y_opti_int = (int16_t)(gps_stereocam.ecef_pos.y);


  //Send measurement values in same structure as stereocam message for state measurements
  DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), &vel_x_int, &vel_y_int,
                           &vel_x_opti_int, &vel_y_opti_int, &pos_x_opti_int, &pos_y_opti_int ,&vel_x_state,&vel_y_state, stereocam_data.len,
                           stereocam_data.data);

#endif

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  struct Int32Vect2 vel;

  vel.x=vel_x_int;
  vel.y=vel_y_int;
  VECT2_STRIM(vel, -50,50);

  vel_x_int = vel.x;
  vel_y_int = vel.y;

  vel_x_int = (int16_t)update_median_filter(&filter_x,(int32_t)vel_x_int);
  vel_y_int = (int16_t)update_median_filter(&filter_y,(int32_t)vel_y_int);


    AbiSendMsgVELOCITY_ESTIMATE(SENDER_ID, now_ts,
                                (float)vel_x_int / 100,
                                (float)vel_y_int / 100,
                                0.0f,
                                0.3f
                               );


 // if (stateGetPositionNed_f()->z < -0.5)
  AbiSendMsgAGL(SENDER_ID, alt_stereo);

}
