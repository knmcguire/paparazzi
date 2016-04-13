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

#include "modules/multi/traffic_info.h"

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
#define FORCE_CLIMB_GAIN 0.1
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

// the following does not include the bias neuron
static const uint8_t nr_input_neurons = 5;
static const uint8_t nr_hidden_neurons = 8;
static const uint8_t nr_output_neurons = 2;

double input_layer_in[6]; //nr_input_neurons+1
double hidden_layer_out[9]; //nr_hidden_neurons+1
const double layer1_weights[6][8] =
{
{ -0.249, 0.6456, 0.2576, -0.0324,  -0.1644,  0.7892, -0.313, -0.5042,},
{ 0.0736, 0.9582, 0.6542, 0.5952, -0.7244,  -0.367, -0.2034,  -0.3356,},
{ 0.1306, -0.6844,  0.8562, 0.1484, -0.4876,  -0.345, -0.611, -0.7448,},
{ 0.039,  0.5906, 0.822,  0.8968, 0.2194, -0.8234,  -0.0694,  -0.3796,},
{ -0.044, -0.2248,  0.645,  -0.5442,  -0.5442,  -0.8088,  -0.67,  -0.9184,},
{ -0.142, -0.363, 2.412,  -1.586, 0.041,  -1.86,  0.443,  1.194,},};
    /* controid nn
{
{ 0.0066, 0.4742, 0.8316, -0.0842,  -0.563, 0.1264, -0.7996,  -0.0832,},
{ -0.0564,  0.2682, 0.0388, 0.009,  -0.075, 0.9812, -0.7254,  0.7934,},
{ 0.1564, -0.1882,  -0.1216,  0.0216, -0.3684,  0.448,  0.833,  0.5022,},
{ -0.8944,  -0.2914,  -0.2838,  -0.0084,  -0.9336,  0.3802, 0.1388, -0.3276,},
{ 0.1306, 0.7086, -0.18,  -0.4894,  0.924,  -0.5994,  0.4542, 0.9976,},
{ -0.593, -0.711, -0.9474,  -0.3176,  1.1,  2.585,  1.776,  -0.967,},};
*/

double layer2_out[2]; //nr_output_neurons
const double layer2_weights[9][2] =
{
{ -0.3672,  -0.3484,},
{ 0.3754, -0.537,},
{ 0.886,  0.1956,},
{ -0.1652,  0.684,},
{ -0.1538,  -0.1288,},
{ 0.4614, -0.0138,},
{ 0.2718, 0.1088,},
{ 0.1112, 0.0138,},
{ -0.161, -0.072,},};

/* controid nn
{
{ 0.5738, 0.2046,},
{ 0.3906, -0.6196,},
{ -0.4542,  0.2314,},
{ -0.5594,  0.7498,},
{ 0.3346, -0.5458,},
{ -0.2122,  0.371,},
{ 0.3254, -0.2864,},
{ 0.6944, -0.7304,},
{ -0.2692,  0.053,},};
 */

int swarm_nn_task(void)
{
  struct EnuCoor_f speed_sp = {.x=0., .y=0., .z=0.};

  // compute desired velocity
  uint8_t i, j;

  if (gps.fix == 0){return 1;}
  struct UtmCoor_i my_pos = (get_ac_info(the_acs[1].ac_id))->utm;
  //my_pos.zone = 0;
  //utm_of_lla_i(&my_pos, &gps.lla_pos);

  float rx = 0.;
  float ry = 0.;
  float rz = 0.;
  float d  = 0.;

  struct EnuCoor_f my_enu_pos = *stateGetPositionEnu_f();

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
    d  += sqrtf(dist2);
  }

  input_layer_in[0]= (double)rx;
  input_layer_in[1]= (double)ry;
  input_layer_in[2]= (double)d;
  input_layer_in[3]= 0;//(double)my_enu_pos.x;    // can add offset here later to move swarm centre
  input_layer_in[4]= 0;//(double)my_enu_pos.y;
  input_layer_in[nr_input_neurons] = 1.;   // bias neuron

  for (i = 0; i < nr_hidden_neurons; i++)
  {
    hidden_layer_out[i] = 0.;
    for (j = 0; j <= nr_input_neurons; j++)
    {
      hidden_layer_out[i] += input_layer_in[j]*layer1_weights[j][i];
    }
    hidden_layer_out[i] = tanh(hidden_layer_out[i]);
  }

  hidden_layer_out[nr_hidden_neurons] = 1.;  // set bias node
  for (i = 0; i < nr_output_neurons; i++)
  {
    layer2_out[i] = 0.;
    for (j = 0; j <= nr_hidden_neurons; j++)
    {
      layer2_out[i] += hidden_layer_out[j]*layer2_weights[j][i];
    }
    layer2_out[i] = tanh(layer2_out[i]);
  }

  speed_sp.x = force_hor_gain*(float)layer2_out[0];
  speed_sp.y = force_hor_gain*(float)layer2_out[1];
  speed_sp.z = 0;

  // limit commands
  BoundAbs(speed_sp.x, 1);
  BoundAbs(speed_sp.y, 1);
  BoundAbs(speed_sp.z, 1);

  potential_force.east = speed_sp.x;
  potential_force.north = speed_sp.y;
  potential_force.alt = rx;

  potential_force.speed = ry;
  potential_force.climb = d;

  autopilot_guided_move_ned(speed_sp.y, speed_sp.x, 0, 0);    // speed in enu

  struct ac_info_ * ac1 = get_ac_info(the_acs[2].ac_id);
  struct ac_info_ * ac2 = get_ac_info(the_acs[3].ac_id);

  int32_t tempx1 = ac1->utm.east - my_pos.east;
  int32_t tempy1 = ac1->utm.north - my_pos.north;
  int32_t tempx2 = ac2->utm.east - my_pos.east;
  int32_t tempy2 = ac2->utm.north - my_pos.north;

  float dist1 = sqrtf((float)(tempx1*tempx1)/10000. + ((float)(tempy1*tempy1))/10000.);
  float dist2 = sqrtf((float)(tempx2*tempx2)/10000. + ((float)(tempy2*tempy2))/10000.);

  DOWNLINK_SEND_AC_INFO(DefaultChannel, DefaultDevice, &ac1->ac_id, &tempx1, &tempy1,
                          &ac2->ac_id, &tempx2, &tempy2);

  DOWNLINK_SEND_SWARMNN(DefaultChannel, DefaultDevice, &potential_force.east, &potential_force.north,
                                  &potential_force.alt, &potential_force.speed, &potential_force.climb,
                                  &my_enu_pos.x, &my_enu_pos.y, &dist1, &dist2);

  return 1;
}
