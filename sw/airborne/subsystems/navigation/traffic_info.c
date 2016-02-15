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

#include "subsystems/navigation/traffic_info.h"

//#include "subsystems/navigation/common_nav.h"
#include "generated/airframe.h"     // AC_ID
#include "math/pprz_geodetic_int.h"
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

struct ac_info_ *get_ac_info(uint8_t _id)
{
  return &the_acs[the_acs_id[_id]];
}

void set_ac_info(uint8_t id, float utm_east, float utm_north, uint16_t utm_zone, float course, float alt,
                 float gspeed, float climb, uint32_t itow)
{
  if (acs_idx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {
      the_acs_id[id] = acs_idx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }
    if (utm_zone == UtmZoneOfLlaLonDeg(gps.lla_pos.lon)) {
      the_acs[the_acs_id[id]].east = utm_east;
      the_acs[the_acs_id[id]].north = utm_north;
      the_acs[the_acs_id[id]].course = course;
      the_acs[the_acs_id[id]].alt = alt;
      the_acs[the_acs_id[id]].gspeed = gspeed;
      the_acs[the_acs_id[id]].climb = climb;
      the_acs[the_acs_id[id]].itow = itow;
    } else { // store other uav in utm extended zone
      struct UtmCoor_f utm_f = {.east = utm_east, .north = utm_north, .alt = alt, .zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon)};
      struct UtmCoor_i utm_i;
      UTM_BFP_OF_REAL(utm_i, utm_f);

      struct LlaCoor_i lla;
      lla_of_utm_i(lla, utm_i);

      set_ac_info_lla(id, lla.lat, lla.lon, lla.alt, course, gspeed, climb, itow);
    }
  }
}

void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                     int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow)
{
  if (acs_idx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {
      the_acs_id[id] = acs_idx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }

    struct LlaCoor_i lla_i = {.lat = lat, .lon = lon, .alt = alt};
    struct UtmCoor_i utm_i;
    utm_i.zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon);   // use current zone as reference, i.e zone extend

    utm_of_lla_i(&utm_i, &lla_i);

    struct UtmCoor_f utm_f;
    UTM_FLOAT_OF_BFP(utm_f, utm_i);

    the_acs[the_acs_id[id]].east = utm_f.east;
    the_acs[the_acs_id[id]].north = utm_f.north;
    the_acs[the_acs_id[id]].alt = utm_f.alt;
    the_acs[the_acs_id[id]].course = RadOfDeg((float)course / 10.);
    the_acs[the_acs_id[id]].gspeed = (float)gspeed * 100;
    the_acs[the_acs_id[id]].climb = (float)climb * 100;
    the_acs[the_acs_id[id]].itow = itow;
  }
}
