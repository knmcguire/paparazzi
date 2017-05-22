/*
 * Copyright (C) K. N. McGuire
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * @file "modules/range_module/range_module.c"
 * @author K. N. McGuire
 * This module contains functions to accomendate the use of single point range sensors.
 */

#include "modules/range_module/range_module.h"
#include "subsystems/abi.h"



//abi for range sensors
#ifndef RANGE_MODULE_SENDER_ID
#define RANGE_MODULE_SENDER_ID ABI_BROADCAST
#endif

static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top);
struct range_finders_ range_finders;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top)
{
  /* TODO: Toe removed
    static int32_t front_wall_detect_counter = 0;
    static const int32_t max_sensor_range = 2000;
  */


  //TODO: Make a seperate module
  // save range finders values
  range_finders.front = range_front;
  range_finders.right = range_right;
  range_finders.left = range_left;
  range_finders.back = range_back;
  range_finders.top = range_top;
  range_finders.bottom = range_bottom;
  /*    uint16_t tel_buf[4] = {0,range_right, 0 , range_left};
      uint8_t length = 4;*/

}

void range_init(void)
{
  AbiBindMsgRANGE_SENSORS(RANGE_MODULE_SENDER_ID, &range_sensors_ev, range_sensors_cb);

}


void range_run(void)
{
  float vel_body_x  = 0;
  float vel_body_y  = 0;
  float vel_body_z  = 0;

  int16_t avoid_inner_border = 800;
  int16_t avoid_outer_border = 1200;
  int16_t tinder_range = 2000;
  float min_vel_command = 0;
  float max_vel_command = 0.3;

  range_sensor_force_field(&vel_body_x, &vel_body_y, &vel_body_z,
                           avoid_inner_border, avoid_outer_border, tinder_range, min_vel_command, max_vel_command);


  AbiSendMsgFORCEFIELD_VELOCITY(RANGE_MODULE_SENDER_ID, vel_body_x, vel_body_y, vel_body_z);
}


void range_sensor_force_field(float *vel_body_x, float *vel_body_y, float *vel_body_z, int16_t avoid_inner_border, int16_t avoid_outer_border,
                              int16_t tinder_range, float min_vel_command, float max_vel_command)
{
  static const int16_t max_sensor_range = 2000;

  int16_t difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;
  float avoid_y_command = *vel_body_y;
  float avoid_z_command = *vel_body_z;

  // Balance avoidance command for y direction (sideways)
  if (range_finders.right < 1 || range_finders.right > max_sensor_range) {
    //do nothing
  } else if (range_finders.right < avoid_inner_border) {
    avoid_y_command -= max_vel_command;
  } else if (range_finders.right < avoid_outer_border) {
    // Linear
    avoid_y_command -= (max_vel_command - min_vel_command) *
                       ((float)avoid_outer_border - (float)range_finders.right)
                       / (float)difference_inner_outer;
  } else {}

  if (range_finders.left < 1 || range_finders.left > max_sensor_range) {
    //do nothing
  } else if (range_finders.left < avoid_inner_border) {
    avoid_y_command += max_vel_command;
  } else if (range_finders.left < avoid_outer_border) {
    // Linear
    avoid_y_command += (max_vel_command - min_vel_command) *
                       ((float)avoid_outer_border - (float)range_finders.left)
                       / (float)difference_inner_outer;
  } else {}

  // balance avoidance command for x direction (forward/backward)
  if (range_finders.front < 1 || range_finders.front > max_sensor_range) {
    //do nothing
  } else if (range_finders.front < avoid_inner_border) {
    avoid_x_command -= max_vel_command;
  } else if (range_finders.front < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command - min_vel_command) *
                       ((float)avoid_outer_border - (float)range_finders.front)
                       / (float)difference_inner_outer;
  } else if (range_finders.front > tinder_range) {
    avoid_x_command += max_vel_command;
  } else {}


  if (range_finders.back < 1 || range_finders.back > max_sensor_range) {
    //do nothing
  } else if (range_finders.back < avoid_inner_border) {
    avoid_x_command += max_vel_command;
  } else if (range_finders.back < avoid_outer_border) {
    // Linear
    avoid_x_command += (max_vel_command - min_vel_command) *
                       ((float)avoid_outer_border - (float)range_finders.back)
                       / (float)difference_inner_outer;
  } else {}

  *vel_body_x = avoid_x_command;
  *vel_body_y = avoid_y_command;
  *vel_body_z = avoid_z_command;

}

void stereo_force_field(float *vel_body_x, float distance_stereo, float avoid_inner_border, float avoid_outer_border,
                        float tinder_range, float min_vel_command, float max_vel_command)
{
  static const int16_t max_sensor_range = 2.0f;

  float difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;

  // Balance avoidance command for front direction (sideways)
  if (distance_stereo > max_sensor_range) {
    //do nothing
  } else if (distance_stereo < avoid_inner_border) {
    avoid_x_command -= max_vel_command;
  } else if (distance_stereo < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command - min_vel_command) *
                       (avoid_outer_border - distance_stereo)
                       / difference_inner_outer;
  } else {
    if (distance_stereo > tinder_range) {
      avoid_x_command += max_vel_command;
    }
  }

  *vel_body_x = avoid_x_command;
}
