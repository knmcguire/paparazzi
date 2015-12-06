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
 * @file subsystems/navigation/traffic_info.c
 *
 * Information relative to the other aircrafts.
 *
 */

#include <inttypes.h>
#include "subsystems/navigation/traffic_info.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/airframe.h"
#include "math/pprz_geodetic_float.h"

uint8_t acs_idx;
uint8_t the_acs_id[NB_ACS_ID];
struct ac_info_ the_acs[NB_ACS];

void traffic_info_init(void)
{
  the_acs_id[0] = 0;  // ground station
  the_acs_id[AC_ID] = 1;
  the_acs[the_acs_id[AC_ID]].ac_id = AC_ID;
  acs_idx = 2;
}

struct ac_info_ *get_ac_info(uint8_t id)
{
  return &the_acs[the_acs_id[id]];
}

// 0 is reserved for ground station (id=0)
// 1 is reserved for this AC (id=AC_ID)
void SetAcInfo(uint8_t _id, float _utm_x /*m*/, float _utm_y /*m*/, float _course/*rad(CW)*/, float _alt/*m*/, float _gspeed/*m/s*/, float _climb, uint32_t _itow) {
  if (acs_idx < NB_ACS) {
    if (_id > 0 && the_acs_id[_id] == 0) {
      the_acs_id[_id] = acs_idx++;
      the_acs[the_acs_id[_id]].ac_id = _id;
    }
    the_acs[the_acs_id[_id]].east = _utm_x -  nav_utm_east0;
    the_acs[the_acs_id[_id]].north = _utm_y - nav_utm_north0;
    the_acs[the_acs_id[_id]].course = _course;
    the_acs[the_acs_id[_id]].alt = _alt;
    the_acs[the_acs_id[_id]].gspeed = _gspeed;
    the_acs[the_acs_id[_id]].climb = _climb;
    the_acs[the_acs_id[_id]].itow = (uint32_t)_itow;
  }
}

void SetAcInfoEcef(uint8_t _id, struct GpsState *remote_gps)
{
  if (acs_idx < NB_ACS) {
    if (_id > 0 && the_acs_id[_id] == 0) {
      the_acs_id[_id] = acs_idx++;
      the_acs[the_acs_id[_id]].ac_id = _id;
    }
    the_acs[the_acs_id[_id]].east   = ((float)remote_gps->utm_pos.east)/100;
    the_acs[the_acs_id[_id]].north  = ((float)remote_gps->utm_pos.north)/100;
    the_acs[the_acs_id[_id]].course = ((float)remote_gps->course)/1e7;
    the_acs[the_acs_id[_id]].alt    = ((float)remote_gps->utm_pos.alt)/1000;
    the_acs[the_acs_id[_id]].gspeed = ((float)remote_gps->gspeed)/100;
    the_acs[the_acs_id[_id]].climb  = ((float)remote_gps->ecef_vel.z)/100;
    the_acs[the_acs_id[_id]].itow   = remote_gps->tow;
  }
}
