/*
 * Copyright (C) 2014 Freek van Tienen
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
 */

/**
 * @file gps_datalink.c
 * @brief GPS system based on datalink
 *
 * This GPS parses the datalink REMOTE_GPS packet and sets the
 * GPS structure to the values received.
 */

#include "messages.h"
#include "generated/airframe.h" // AC_ID is required
#include "generated/flight_plan.h" // AC_ID is required
#include "subsystems/datalink/downlink.h"
#include "subsystems/abi.h"

// #include <stdio.h>

struct LtpDef_i ltp_def;
struct EnuCoor_i enu_pos, enu_speed;
struct EcefCoor_i ecef_pos, ecef_vel;
struct LlaCoor_i lla_pos;

bool_t gps_available;   ///< Is set to TRUE when a new REMOTE_GPS packet is received and parsed

/** GPS initialization */
void gps_impl_init(void)
{
  gps.fix = GPS_FIX_NONE;
  gps_available = FALSE;
  gps.gspeed = 700; // To enable course setting
  gps.cacc = 0; // To enable course setting

  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = NAV_LAT0;
  llh_nav0.lon = NAV_LON0;
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);
  ltp_def_from_ecef_i(&ltp_def, &ecef_nav0);
}

// Parse the REMOTE_GPS_SMALL datalink packet
void parse_gps_datalink_small(uint8_t num_sv, uint32_t pos_xyz, uint32_t speed_xyh, int8_t speed_z)
{
  // Position in ENU coordinates
  enu_pos.x = (int32_t)((pos_xyz >> 22) & 0x3FF); // bits 31-22 x position in cm
  if (enu_pos.x & 0x200) {
    enu_pos.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 12) & 0x3FF); // bits 21-12 y position in cm
  if (enu_pos.y & 0x200) {
    enu_pos.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.z = (int32_t)(pos_xyz >> 2 & 0x3FF); // bits 11-2 z position in cm
  // bits 1 and 0 are free

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&ecef_pos, &ltp_def, &enu_pos);
  gps.ecef_pos = ecef_pos;

  lla_of_ecef_i(&lla_pos, &ecef_pos);
  gps.lla_pos = lla_pos;

  enu_speed.x = (int32_t)((speed_xyh >> 22) & 0x3FF); // bits 31-22 speed x in cm/s
  if (enu_speed.x & 0x200) {
    enu_speed.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.y = (int32_t)((speed_xyh >> 12) & 0x3FF); // bits 21-12 speed y in cm/s
  if (enu_speed.y & 0x200) {
    enu_speed.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.z = speed_z;

  ecef_of_enu_vect_i(&gps.ecef_vel , &ltp_def , &enu_speed);

  gps.hmsl = ltp_def.hmsl + enu_pos.z * 10; // TODO: try to compensate for the loss in accuracy

  gps.course = (int32_t)((speed_xyh >> 2) & 0x3FF); // bits 11-2 heading in rad*1e2
  if (gps.course & 0x200) {
    gps.course |= 0xFFFFFC00;  // fix for twos complements
  }

  gps.course *= 1e5;
  gps.num_sv = num_sv;
  gps.tow = 0; // set time-of-week to 0
  gps.fix = GPS_FIX_3D; // set 3D fix to true
  gps_available = TRUE; // set GPS available to true

#if GPS_USE_LATLONG
  // Computes from (lat, long) in the referenced UTM zone
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
}

// Parse the REMOTE_GPS_SMALL datalink packet
void parse_remote_gps_datalink_small(struct GpsState *remote_gps, uint8_t num_sv, uint32_t pos_xyz, uint32_t speed_xyh,
                                     int8_t speed_z)
{
  // Position in ENU coordinates
  enu_pos.x = (int32_t)((pos_xyz >> 22) & 0x3FF); // bits 31-22 x position in cm
  if (enu_pos.x & 0x200) {
    enu_pos.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.y = (int32_t)((pos_xyz >> 12) & 0x3FF); // bits 21-12 y position in cm
  if (enu_pos.y & 0x200) {
    enu_pos.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_pos.z = (int32_t)((pos_xyz >> 2) & 0x3FF);  // bits 11-2 z position in cm
  // bits 1 and 0 are free

  // printf("ENU Pos: %u (%d, %d, %d)\n", pos_xyz, enu_pos.x, enu_pos.y, enu_pos.z);

  // Convert the ENU coordinates to ECEF
  ecef_of_enu_point_i(&ecef_pos, &ltp_def, &enu_pos);
  remote_gps->ecef_pos = ecef_pos;

  lla_of_ecef_i(&lla_pos, &ecef_pos);
  remote_gps->lla_pos = lla_pos;

  enu_speed.x = (int32_t)((speed_xyh >> 22) & 0x3FF); // bits 31-22 speed x in cm/s
  if (enu_speed.x & 0x200) {
    enu_speed.x |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.y = (int32_t)((speed_xyh >> 12) & 0x3FF); // bits 21-12 speed y in cm/s
  if (enu_speed.y & 0x200) {
    enu_speed.y |= 0xFFFFFC00;  // fix for twos complements
  }
  enu_speed.z = speed_z;  // speed y in cm/s

  // printf("ENU Speed: %u (%d, %d, %d)\n", speed_xy, enu_speed.x, enu_speed.y, enu_speed.z);

  ecef_of_enu_vect_i(&gps.ecef_vel , &ltp_def , &enu_speed);

  remote_gps->hmsl = ltp_def.hmsl + enu_pos.z * 10; // TODO: try to compensate for the loss in accuracy

  remote_gps->course = (int32_t)((speed_xyh >> 2) & 0x3FF); // bits 11-2 heading in rad*1e2
  if (remote_gps->course & 0x200) {
    remote_gps->course |= 0xFFFFFC00;  // fix for twos complements
  }

  remote_gps->course *= 1e5;
  remote_gps->num_sv = num_sv;
  remote_gps->tow = 0; // set time-of-week to 0
  remote_gps->fix = GPS_FIX_3D; // set 3D fix to true

#if GPS_USE_LATLONG
  // Computes from (lat, long) in the referenced UTM zone
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  remote_gps->utm_pos.east = utm_f.east * 100;
  remote_gps->utm_pos.north = utm_f.north * 100;
  remote_gps->utm_pos.alt = gps.lla_pos.alt;
  remote_gps->utm_pos.zone = nav_utm_zone0;
#endif

}

void send_remote_gps_datalink_small(void)
{
  uint8_t ac_id = AC_ID;

  enu_of_ecef_point_i(&enu_pos, &ltp_def, &ecef_pos);

  // Position in ENU coordinates
  uint32_t pos_xyz = (((uint32_t)enu_pos.x) & 0x3FF) << 22; // bits 31-22 x position in cm
  pos_xyz |= (((uint32_t)enu_pos.y) & 0x3FF) << 12;         // bits 21-12 y position in cm
  pos_xyz |= (((uint32_t)enu_pos.z) & 0x3FF) << 2;          // bits 11-2 z position in cm

  enu_of_ecef_vect_i(&enu_speed, &ltp_def, &gps.ecef_vel);

  // Speed in ENU coordinates
  uint32_t speed_xyh = (((uint32_t)enu_speed.x) & 0x3FF) << 22; // bits 31-22 speed x in cm/s
  speed_xyh |= (((uint32_t)enu_speed.y) & 0x3FF) << 12;         // bits 21-12 speed y in cm/s
  speed_xyh |= (((uint32_t)(gps.course / 1e5)) & 0x3FF) << 2;   // bits 11-2 heading in rad*1e2

  int8_t speed_z = enu_speed.z;

  DOWNLINK_SEND_TELEM_REMOTE_GPS_SMALL(DefaultChannel, DefaultDevice, &ac_id, &gps.num_sv, &pos_xyz, &speed_xyh,
                                       &speed_z);
}

/** Parse the REMOTE_GPS datalink packet */
void parse_gps_datalink(uint8_t numsv, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z, int32_t lat, int32_t lon,
                        int32_t alt,
                        int32_t hmsl, int32_t ecef_xd, int32_t ecef_yd, int32_t ecef_zd, uint32_t tow, int32_t course)
{
  gps.lla_pos.lat = lat;
  gps.lla_pos.lon = lon;
  gps.lla_pos.alt = alt;
  gps.hmsl        = hmsl;

  gps.ecef_pos.x = ecef_x;
  gps.ecef_pos.y = ecef_y;
  gps.ecef_pos.z = ecef_z;

  gps.ecef_vel.x = ecef_xd;
  gps.ecef_vel.y = ecef_yd;
  gps.ecef_vel.z = ecef_zd;

  gps.course = course;
  gps.num_sv = numsv;
  gps.tow = tow;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;

#if GPS_USE_LATLONG
  // Computes from (lat, long) in the referenced UTM zone
  struct LlaCoor_f lla_f;
  LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
  struct UtmCoor_f utm_f;
  utm_f.zone = nav_utm_zone0;
  // convert to utm
  utm_of_lla_f(&utm_f, &lla_f);
  // copy results of utm conversion
  gps.utm_pos.east = utm_f.east * 100;
  gps.utm_pos.north = utm_f.north * 100;
  gps.utm_pos.alt = gps.lla_pos.alt;
  gps.utm_pos.zone = nav_utm_zone0;
#endif

  // publish new GPS data
  uint32_t now_ts = get_sys_time_usec();
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  if (gps.fix == GPS_FIX_3D) {
    gps.last_3dfix_ticks = sys_time.nb_sec_rem;
    gps.last_3dfix_time = sys_time.nb_sec;
  }
  AbiSendMsgGPS(GPS_DATALINK_ID, now_ts, &gps);
}

