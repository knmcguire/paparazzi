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

uint8_t acsidx;
uint8_t the_acs_id[NB_ACS_ID];
struct ac_info_ the_acs[NB_ACS];

void traffic_info_init(void)
{
  memset(the_acs_id, 0, NB_ACS_ID);
  memset(the_acs, 0, NB_ACS);

  the_acs_id[0] = 0;  // ground station
  the_acs_id[AC_ID] = 1;
  the_acs[the_acs_id[AC_ID]].ac_id = AC_ID;
  acsidx = 2;
}

struct ac_info_ *get_ac_info(uint8_t id)
{
  return &the_acs[the_acs_id[id]];
}

// 0 is reserved for ground station (id=0)
// 1 is reserved for this AC (id=AC_ID)
void SetAcInfo(uint8_t id, uint32_t lat, uint32_t lon, uint32_t alt, uint32_t course, uint16_t gspeed, uint16_t climb,
               uint32_t tow)
{
  if (acsidx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {
      the_acs_id[id] = acsidx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }

    struct EcefCoor_i ecef_pos;
    struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
    ecef_of_lla_i(&ecef_pos, &lla);
    the_acs[the_acs_id[id]].ecef_pos = (struct EcefCoor_f) {(float)ecef_pos.x * 100, (float)ecef_pos.y * 100, (float)ecef_pos.z * 100};
    the_acs[the_acs_id[id]].course = course;
    the_acs[the_acs_id[id]].gspeed = gspeed;
    the_acs[the_acs_id[id]].climb = climb;
    the_acs[the_acs_id[id]].tow = tow;
  }
}

void SetAcInfoRemote(uint8_t id, struct GpsState *remote_gps)
{
  if (acsidx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {
      the_acs_id[id] = acsidx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }

    the_acs[the_acs_id[id]].ecef_pos = (struct EcefCoor_f) {(float)remote_gps->ecef_pos.x * 100, (float)remote_gps->ecef_pos.y * 100, (float)remote_gps->ecef_pos.z * 100};
    the_acs[the_acs_id[id]].course = remote_gps->course;
    the_acs[the_acs_id[id]].gspeed = remote_gps->gspeed;
    the_acs[the_acs_id[id]].climb = remote_gps->ecef_vel.z;
    the_acs[the_acs_id[id]].tow = remote_gps->tow;
  }
}
