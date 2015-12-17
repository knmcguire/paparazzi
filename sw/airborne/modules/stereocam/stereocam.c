/*
 * Copyright (C) 2015 Kirk Scheper
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/stereocam/stereocam.c
 *  @brief interface to TU Delft serial stereocam
 *  Include stereocam.xml to your airframe file.
 *  Parameters STEREO_PORT, STEREO_BAUD, SEND_STEREO and STEREO_BUF_SIZE should be configured with stereocam.xml.
 */

#include "modules/stereocam/stereocam.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereoprotocol.h"

///////////
#include "subsystems/abi.h"
#include "state.h"

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"
#include "subsystems/gps.h"

#ifndef SENDER_ID
#define SENDER_ID 1
#endif
///////////////

#ifndef SEND_STEREO
#define SEND_STEREO TRUE
#endif

// define coms link for stereocam
#define STEREO_PORT   (&((UART_LINK).device))
struct link_device *linkdev = STEREO_PORT;
#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)

// pervasive local variables
MsgProperties msgProperties;


uint16_t freq_counter = 0;
uint8_t frequency = 0;
uint32_t previous_time = 0;

#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 1024                     // size of circular buffer
#endif
uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
uint8array stereocam_data = {.len = 0, .data = msg_buf, .fresh = 0, .matrix_width = 0, .matrix_height = 0}; // buffer used to contain image without line endings

#define BASELINE_STEREO_MM 60.0
#define BRANDSPUNTSAFSTAND_STEREO 118.0*6


////////////////////////////////////////////
static float agl;
static float prev_phi;
static float prev_theta;
struct prev_velocity_t {
  float x;
  float y;
};
static struct prev_velocity_t prev_velocity;

struct EcefCoor_i gps_speed;
struct NedCoor_i opti_speed;


void stereocam_to_state(uint8array *stereocam_data, float agl, float dphi, float dtheta,
                        struct prev_velocity_t *prev_velocity,   struct EcefCoor_i gps_speed);
/////////////////////////
extern void stereocam_disparity_to_meters(uint8_t *disparity, float *distancesMeters, int lengthArray)
{

  int indexArray = 0;
  for (indexArray = 0; indexArray < lengthArray; indexArray++) {
    if (disparity[indexArray] != 0) {
      distancesMeters[indexArray] = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)disparity[indexArray] - 18.0)) /
                                    1000;
      //  printf("%i, distanceMeters: %f \n",indexArray,distancesMeters[indexArray]);
    } else {
      distancesMeters[indexArray] = 1000;
    }
  }
}

extern void stereocam_start(void)
{
  // initialize local variables
  msgProperties = (MsgProperties) {0, 0, 0};

  insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;

  //sys_time_init();
  freq_counter = 0;
  frequency = 0;
  previous_time = sys_time.nb_tick;

  stereocam_data.fresh = 0;
}

extern void stereocam_stop(void)
{
}

extern void stereocam_periodic(void)
{
  // read all data from the stereo com link, check that don't overtake extract
  while (linkdev->char_available(linkdev->periph) && stereoprot_add(insert_loc, 1, STEREO_BUF_SIZE) != extract_loc) {
    if (handleStereoPackage(StereoGetch(), STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                            &stereocam_data.fresh, &stereocam_data.len, &stereocam_data.matrix_width, &stereocam_data.matrix_height)) {
      freq_counter++;
      if ((sys_time.nb_tick - previous_time) > sys_time.ticks_per_sec) {  // 1s has past
        frequency = (uint8_t)((freq_counter * (sys_time.nb_tick - previous_time)) / sys_time.ticks_per_sec);
        freq_counter = 0;
        previous_time = sys_time.nb_tick;
      }
      ////////////////////////
      float phi = stateGetNedToBodyEulers_f()->phi;
      float theta = stateGetNedToBodyEulers_f()->theta;
      float dphi =  phi - prev_phi;
      float dtheta = theta - prev_theta;
      stereocam_to_state(&stereocam_data, agl, dphi, dtheta, &prev_velocity, gps.ecef_vel);
      prev_theta = theta;
      prev_phi = phi;
      ////////////////////
#if SEND_STEREO
      if (stereocam_data.len > 100) {
        DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), 100, stereocam_data.data);

      } else {
        DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), stereocam_data.len,
                                 stereocam_data.data);

      }
#endif
    }
  }
}

void stereocam_to_state(uint8array *stereocam_data, float agl, float dphi, float dtheta,
                        struct prev_velocity_t *prev_velocity,   struct EcefCoor_i gps_speed)
{

  // Get info from stereocam data
  float agl_stereo = (float)(stereocam_data->data[4]) / 10;
  float vel_hor = ((float)(stereocam_data->data[8]) - 127) / 100;
  float vel_ver = ((float)(stereocam_data->data[9]) - 127) / 100;

  // Calculate derotated velocity
  float diff_flow_hor = dtheta * 128 / 1.04;
  float diff_flow_ver = dphi * 96 / 0.785;

  float diff_vel_hor = diff_flow_hor * agl_stereo * 12 * 1.04 / 128;
  float diff_vel_ver = diff_flow_ver * agl_stereo * 12 * 0.785 / 96;

  // Derotate velocity and transform from frame to body coordinates
  //float vel_x = - (vel_ver - diff_vel_ver);
  //float vel_y =  (vel_hor + diff_vel_hor);
  float vel_x = - (vel_ver);
  float vel_y = (vel_hor);

  // Calculate velocity in body fixed coordinates from opti-track and the state filter
  struct NedCoor_f coordinates_speed_state;

  // Information about optitrack origins
  struct EcefCoor_i tracking_ecef;

  tracking_ecef.x = GPS_LOCAL_ECEF_ORIGIN_X;
  tracking_ecef.y = GPS_LOCAL_ECEF_ORIGIN_Y;
  tracking_ecef.z = GPS_LOCAL_ECEF_ORIGIN_Z;

  coordinates_speed_state.x = stateGetSpeedNed_f()->x;
  coordinates_speed_state.y = stateGetSpeedNed_f()->y;
  coordinates_speed_state.z = stateGetSpeedNed_f()->z;

  struct NedCoor_f opti_state;
  opti_state.x = (float)(gps_speed.y) / 100;
  opti_state.y = (float)(gps_speed.x) / 100;
  opti_state.z = -(float)(gps_speed.z) / 100;

  struct FloatVect3 velocity_rot_state;
  struct FloatVect3 velocity_rot_gps;

  float_rmat_vmult(&velocity_rot_state , stateGetNedToBodyRMat_f(), &coordinates_speed_state);
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(),   &opti_state);

  float vel_x_opti = -((float)(velocity_rot_gps.y));
  float vel_y_opti = ((float)(velocity_rot_gps.x));

  // Calculate velocity error
  float vel_x_error = vel_x_opti - vel_x;
  float vel_y_error = vel_y_opti - vel_y;

//TODO:: Check out why vel_x_opti is 10 x big as stereocamera's output
  stereocam_data->data[8] = (uint8_t)((vel_x * 10) + 127);
  stereocam_data->data[9] = (uint8_t)((vel_y * 10) + 127);
  stereocam_data->data[19] = (uint8_t)((vel_x_opti) * 10 + 127); // dm/s
  stereocam_data->data[20] = (uint8_t)((vel_y_opti) * 10 + 127); // dm/s
  stereocam_data->data[21] = (uint8_t)((vel_x_error) * 10 + 127); //dm/s
  stereocam_data->data[22] = (uint8_t)((vel_y_error) * 10 + 127); //dm/s
  stereocam_data->data[23] = (uint8_t)((velocity_rot_state.x) * 100 + 127); //dm/s
  stereocam_data->data[24] = (uint8_t)((velocity_rot_state.y) * 100 + 127); //dm/s
  uint32_t now_ts = get_sys_time_usec();

  //TODO:: Make variance dependable on line fit error
  if (!(abs(vel_y) > 0.5 || abs(vel_x) > 0.5) || abs(dphi) > 0.05 || abs(dtheta) > 0.05) {
    AbiSendMsgVELOCITY_ESTIMATE(SENDER_ID, now_ts,
                                vel_x,
                                vel_y,
                                0.0f,
                                0.3f
                               );
  }

  prev_velocity->x = vel_x;
  prev_velocity->y = vel_y;
}
