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
 * @file "modules/multi/swarm_nn.c"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#include "modules/multi/swarm_nn.h"
#include "subsystems/gps.h"
#include "subsystems/gps/gps_datalink.h"
#include "subsystems/datalink/downlink.h"
//#include "guidance/guidance_h.h"
//#include "guidance/guidance_v.h"
#include "state.h"
#include "navigation.h"

#include "generated/flight_plan.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_double.h"

//#include "math/pprz_algebra_float.h"
//#include "math/pprz_algebra_int.h"
//#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/guidance/guidance_v.h"

#include "autopilot.h"

#include "subsystems/navigation/traffic_info.h"

#include "generated/airframe.h"           // AC_ID

#include "subsystems/abi.h"               // rssi

int8_t rssi[NB_ACS_ID];
int8_t tx_strength[NB_ACS_ID];

struct force_ potential_force;

float force_hor_gain;
float force_climb_gain;
float target_dist3;
bool use_waypoint;

#ifndef FORCE_HOR_GAIN
#define FORCE_HOR_GAIN 0.5
#endif

#ifndef FORCE_CLIMB_GAIN
#define FORCE_CLIMB_GAIN 0.5
#endif

#ifndef TARGET_DIST3
#define TARGET_DIST3 0.5
#endif

#ifndef USE_WAYPOINT
#define USE_WAYPOINT FALSE
#endif

#ifndef SP_WP
#define SP_WP WP_STDBY
#endif

abi_event ev;

void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi);
void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi) {
  tx_strength[the_acs_id[_ac_id]] = _tx_strength;
  rssi[the_acs_id[_ac_id]] = _rssi;
}

/*
 * return sign of input with bias 1 for an input of zero
 */
float sign(float x);
float sign(float x)
{
//  return (x < 0) ? -1 : (x > 0);
  return (x < 0) ? -1 : 1;
}

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_periodic(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_POTENTIAL(trans, dev, AC_ID, &potential_force.east, &potential_force.north,
                                &potential_force.alt, &potential_force.speed, &potential_force.climb);
}

#endif

void swarm_nn_init(void)
{
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  potential_force.speed = 0.;
  potential_force.climb = 0.;

  force_hor_gain = FORCE_HOR_GAIN;
  force_climb_gain = FORCE_CLIMB_GAIN;
  target_dist3 = TARGET_DIST3;
  use_waypoint = USE_WAYPOINT;

  AbiBindMsgRSSI(ABI_BROADCAST, &ev, rssi_cb);

#if PERIODIC_TELEMETRY
  //register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_POTENTIAL, send_periodic);
#endif
}
/*
 * Send my gps position to the other members of the swarm
 */
void swarm_nn_periodic(void) {
  /* The GPS messages are most likely too large to be send over either the datalink
   * The local position is an int32 and the 11 LSBs of the x and y axis are compressed into
   * a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
   * used.
   */
  uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(gps.course)/1e7)/2)) & 0x7FF) << 21; // bits 31-21 x position in cm
  multiplex_speed |= (((uint32_t)(gps.gspeed)) & 0x7FF) << 10;         // bits 20-10 y position in cm
  multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);               // bits 9-0 z position in cm

  int16_t alt = (int16_t)(gps.lla_pos.alt/10);

  //DOWNLINK_SEND_GPS_SMALL(DefaultChannel, DefaultDevice, &multiplex_speed, &gps.lla_pos.lat,
  //                         &gps.lla_pos.lon, &alt);
}
float layer1_out[6];
const float layer1_weights[4][5] = {
    {0.53,  -0.292,  0.553344,  0.122, 1.67},
    {-0.312,  -0.166,  -0.0762323,  0.568, -0.62},
    {0.548, 0.088, 0.989708,  0.98,  3.1},
    {-0.781943, -0.575229, -0.363866, 0.610577,  0.77547}};

float layer2_out[2];
const float layer2_weights[6][2] = {
    {-0.048,  -0.66 },
    {-0.692,  -0.439091 },
    {-0.712438, 0.967283},
    {-0.624,  -0.57},
    {-0.052,  0.39646},
    {0.882447,  -0.668099 }};

int swarm_nn_task(void)
{
  struct EnuCoor_f speed_sp = {.x=0., .y=0., .z=0.};

  // compute desired velocity
  int8_t nb = 0;
  uint8_t i, j;

  if (gps.fix == 0){return 1;}
  struct UtmCoor_i my_pos;
  my_pos.zone = 0;
  utm_of_lla_i(&my_pos, &gps.lla_pos);

  float rx = 0.;
  float ry = 0.;
  float rz = 0.;
  float d  = 0.;

  for (i = 0; i < acs_idx; i++) {
    if (the_acs[i].ac_id == 0 || the_acs[i].ac_id == AC_ID) { continue; }
    struct ac_info_ * ac = get_ac_info(the_acs[i].ac_id);
    //float delta_t = ABS((int32_t)(gps.tow - ac->itow) / 1000.);
    // if AC not responding for too long, continue, else compute force
    //if(delta_t > 5) { continue; }

    float de = (ac->utm.east - my_pos.east)/100.; // + sha * delta_t
    float dn = (ac->utm.north - my_pos.north)/100.; // cha * delta_t
    float da = (ac->utm.alt - my_pos.alt)/1000.; // ac->climb * delta_t   // currently wrong reference in other aircraft

    float dist2 = de * de + dn * dn;// + da * da;

    rx += de;
    ry += dn;
    rz += da;
    d += sqrtf(dist2);
  }

  memset(layer1_out, 0, 5);
  for (j = 0; j < 5; j++)
  {
    layer1_out[j] += rx*layer1_weights[0][j];
    layer1_out[j] += ry*layer1_weights[1][j];
    layer1_out[j] += d *layer1_weights[2][j];
    layer1_out[j] += 1.*layer1_weights[3][j];

    layer1_out[j] = tanh(layer1_out[j]);
  }

  memset(layer2_out, 0, 2);
  layer1_out[5] = 1;  // set bias node
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < 6; j++)
    {
      layer2_out[i] += layer1_out[j]*layer2_weights[j][i];
    }
    layer2_out[i] = tanh(layer2_out[i]);
  }

  speed_sp.x = layer2_out[0];
  speed_sp.y = layer2_out[1];
  speed_sp.z = 0;

  potential_force.east = speed_sp.x;
  potential_force.north = speed_sp.y;
  potential_force.alt = rx;

  potential_force.speed = ry;
  potential_force.climb = d;

  // limit commands
  BoundAbs(speed_sp.x, 1);
  BoundAbs(speed_sp.y, 1);
  BoundAbs(speed_sp.z, 1);

  autopilot_guided_move_ned(speed_sp.y, speed_sp.x, 0, 0);

  struct ac_info_ * ac1 = get_ac_info(the_acs[2].ac_id);
  struct ac_info_ * ac2 = get_ac_info(the_acs[3].ac_id);
  DOWNLINK_SEND_AC_INFO(DefaultChannel, DefaultDevice, &ac1->ac_id, &ac1->utm.east, &ac1->utm.north,
                          &ac2->ac_id, &ac2->utm.east, &ac2->utm.north);

  DOWNLINK_SEND_POTENTIAL(DefaultChannel, DefaultDevice, &potential_force.east, &potential_force.north,
                                  &potential_force.alt, &potential_force.speed, &potential_force.climb);

  return 1;
}
