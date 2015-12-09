/*
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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
 * @file subsystems/navigation/traffic_info.h
 *
 * Information relative to the other aircrafts.
 *
 */

#ifndef TI_H
#define TI_H

#include "subsystems/gps.h"
#include "math/pprz_geodetic_float.h"

#define NB_ACS_ID 256
#define NB_ACS 24

struct ac_info_ {
  uint8_t ac_id;
  struct EcefCoor_f ecef_pos;  ///< position in ECEF in m
  //struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
  uint16_t gspeed;               ///< norm of 2d ground speed in cm/s
  uint16_t climb;      ///< climb speed in cm/s
  int32_t course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
  uint32_t tow;                  ///< GPS time of week in ms
};

extern uint8_t acs_idx;
extern uint8_t the_acs_id[NB_ACS_ID];
extern struct ac_info_ the_acs[NB_ACS];

extern void traffic_info_init(void);

extern struct ac_info_ *get_ac_info(uint8_t id);

extern void SetAcInfo(uint8_t id, uint32_t lat, uint32_t lon, uint32_t alt, uint32_t course, uint16_t gspeed,
                      uint16_t climb, uint32_t tow);

struct GpsState;
extern void SetAcInfoRemote(uint8_t _id, struct GpsState *remote_gps);

#endif
