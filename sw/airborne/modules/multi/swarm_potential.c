/*
 * Copyright (C) Kirk Scheper
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
/**
 * @file "modules/multi/swarm_potential.c"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#include "modules/multi/swarm_potential.h"
#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"
#include "subsystems/datalink/downlink.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "state.h"
#include "navigation.h"

#include "generated/airframe.h"           // AC_ID

struct force_ potential_force;

float force_pos_gain;
float force_speed_gain;
float force_climb_gain;

#ifndef FORCE_POS_GAIN
#define FORCE_POS_GAIN 1.
#endif

#ifndef FORCE_SPEED_GAIN
#define FORCE_SPEED_GAIN 1.
#endif

#ifndef FORCE_CLIMB_GAIN
#define FORCE_CLIMB_GAIN 1.
#endif

#ifndef FORCE_MAX_DIST
#define FORCE_MAX_DIST 100.
#endif

void swarm_potential_init(void)
{
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  potential_force.speed_x = 0.;
  potential_force.speed_y = 0.;
  potential_force.speed_z = 0.;

  force_pos_gain = FORCE_POS_GAIN;
  force_speed_gain = FORCE_SPEED_GAIN;
  force_climb_gain = FORCE_CLIMB_GAIN;
}

void swarm_potential_periodic(void) {
  uint16_t i;
  for (i = 0; i < NB_ACS; ++i) {
      if (the_acs[i].ac_id == 0 || the_acs[i].ac_id == AC_ID) { continue; }
      struct ac_info_ * ac = get_ac_info(the_acs[i].ac_id);
      potential_force.east = ac->east;
      potential_force.north = ac->north;
      potential_force.alt = ac->alt;
  }

  DOWNLINK_SEND_POTENTIAL(DefaultChannel, DefaultDevice, &potential_force.east, &potential_force.north,
                            &potential_force.alt, &potential_force.speed_x, &potential_force.speed_z);

  send_remote_gps_datalink_small();
}

int swarm_potential_task(void)
{
  uint8_t i;

  float ch = cosf(stateGetHorizontalSpeedDir_f());
  float sh = sinf(stateGetHorizontalSpeedDir_f());
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  // compute control forces
  int8_t nb = 0;
  for (i = 0; i < NB_ACS; ++i) {
    if (the_acs[i].ac_id == 0 || the_acs[i].ac_id == AC_ID) { continue; }
    struct ac_info_ * ac = get_ac_info(the_acs[i].ac_id);
    float delta_t = Max((int)(gps.tow - ac->itow) / 1000., 0.);
    // if AC not responding for too long, continue, else compute force
    if (delta_t > CARROT) { continue; }
    else {
      float sha = sinf(ac->course);
      float cha = cosf(ac->course);
      float de = ac->east  + sha * delta_t - stateGetPositionEnu_f()->x;
      float dn = ac->north + cha * delta_t - stateGetPositionEnu_f()->y;
      float da = ac->alt + ac->climb * delta_t - stateGetPositionEnu_f()->z;
      float dist = sqrtf(de * de + dn * dn + da * da);
      if (dist == 0.) { continue; }
      float dve = stateGetHorizontalSpeedNorm_f() * sh - ac->gspeed * sha;
      float dvn = stateGetHorizontalSpeedNorm_f() * ch - ac->gspeed * cha;
      float dva = stateGetSpeedEnu_f()->z - the_acs[i].climb;
      float scal = dve * de + dvn * dn + dva * da;
      float d3 = dist * dist * dist;
      potential_force.east += scal * de / d3;
      potential_force.north += scal * dn / d3;
      potential_force.alt += scal * da / d3;
      ++nb;
    }
  }
  if (nb == 0) { return 1; }

  // set commands
  //NavVerticalAutoThrottleMode(0.);

  // carrot
  potential_force.speed_x = -force_pos_gain * potential_force.east;
  potential_force.speed_y = -force_pos_gain * potential_force.north;
  potential_force.speed_z = -force_climb_gain * potential_force.alt;

#if 0
  VECT2_COPY(guidance_h.sp.pos, *stateGetPositionNed_i());
  guidance_h.sp.speed.x = potential_force.speed_x;
  guidance_h.sp.speed.y = potential_force.speed_y;

  GuidanceVSetRef(stateGetPositionNed_i()->z, stateGetSpeedNed_i()->z, 0);
  guidance_v_zd_sp = potential_force.speed_z;
#else
  //guidance_h.sp.pos.x += potential_force.speed_x*;
  //guidance_h.sp.pos.x += potential_force.speed_y*;

  //guidance_v_z_ref += potential_force.speed_z*PERIOD;
#endif

//  DOWNLINK_SEND_POTENTIAL(DefaultChannel, DefaultDevice, &potential_force.east, &potential_force.north,
//                          &potential_force.alt, &potential_force.speed, &potential_force.climb);
  return 1;
}
