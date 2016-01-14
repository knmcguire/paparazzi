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

#include "generated/flight_plan.h"

#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
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

#ifndef FORCE_HOR_GAIN
#define FORCE_HOR_GAIN 1.
#endif

#ifndef FORCE_CLIMB_GAIN
#define FORCE_CLIMB_GAIN 1.
#endif

#ifndef TARGET_DIST3
#define TARGET_DIST3 1.
#endif

abi_event ev;

void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi);
void rssi_cb(uint8_t sender_id __attribute__((unused)), uint8_t _ac_id, int8_t _tx_strength, int8_t _rssi) {
  tx_strength[the_acs_id[_ac_id]] = _tx_strength;
  rssi[the_acs_id[_ac_id]] = _rssi;
}

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851

#ifndef SWARM_PHI_PGAIN
#define SWARM_PHI_PGAIN 400
#endif
////PRINT_CONFIG_VAR(SWARM_PHI_PGAIN)

#ifndef SWARM_PHI_IGAIN
#define SWARM_PHI_IGAIN 20
#endif
//PRINT_CONFIG_VAR(SWARM_PHI_IGAIN)

#ifndef SWARM_THETA_PGAIN
#define SWARM_THETA_PGAIN 400
#endif
//PRINT_CONFIG_VAR(SWARM_THETA_PGAIN)

#ifndef SWARM_THETA_IGAIN
#define SWARM_THETA_IGAIN 20
#endif
//PRINT_CONFIG_VAR(SWARM_THETA_IGAIN)

/* Check the control gains */
#if (SWARM_PHI_PGAIN < 0)      ||  \
  (SWARM_PHI_IGAIN < 0)        ||  \
  (SWARM_THETA_PGAIN < 0)      ||  \
  (SWARM_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct swarm_stab_t swarm_stab = {
  .phi_pgain = SWARM_PHI_PGAIN,
  .phi_igain = SWARM_PHI_IGAIN,
  .theta_pgain = SWARM_THETA_PGAIN,
  .theta_igain = SWARM_THETA_IGAIN
};

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_periodic(struct transport_tx *trans, struct link_device *dev) {
  pprz_msg_send_POTENTIAL(trans, dev, AC_ID, &potential_force.east, &potential_force.north,
                              &potential_force.alt, &potential_force.speed, &potential_force.climb);
}

#endif

/**
 * Initialization of horizontal guidance module.
 */
void guidance_h_module_init(void)
{
}

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  /* Reset the integrated errors */
  swarm_stab.err_vx_int = 0;
  swarm_stab.err_vy_int = 0;

  /* Set roll/pitch to 0 degrees and psi to current heading */
  swarm_stab.cmd.phi = 0;
  swarm_stab.cmd.theta = 0;
  swarm_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
  /* Update the setpoint */
  stabilization_attitude_set_rpy_setpoint_i(&swarm_stab.cmd);

  /* Run the default attitude stabilization */
  stabilization_attitude_run(in_flight);
}

void swarm_potential_init(void)
{
  potential_force.east = 0.;
  potential_force.north = 0.;
  potential_force.alt = 0.;

  potential_force.speed = 0.;
  potential_force.climb = 0.;

  force_hor_gain = FORCE_HOR_GAIN;
  force_climb_gain = FORCE_CLIMB_GAIN;
  target_dist3 = TARGET_DIST3;

  AbiBindMsgRSSI(ABI_BROADCAST, &ev, rssi_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_POTENTIAL, send_periodic);
#endif
}

void swarm_potential_periodic(void) {
  /* The GPS messages are most likely too large to be send over either the datalink
   * The local position is an int32 and the 11 LSBs of the x and y axis are compressed into
   * a single integer. The z axis is considered unsigned and only the latter 10 LSBs are
   * used.
   */
  uint32_t multiplex_speed = (((uint32_t)(gps.course*10.0)) & 0x7FF) << 21; // bits 31-21 x position in cm
  multiplex_speed |= (((uint32_t)(gps.gspeed*100.0)) & 0x7FF) << 10;         // bits 20-10 y position in cm
  multiplex_speed |= (((uint32_t)(-gps.ned_vel.z*100.0)) & 0x3FF);               // bits 9-0 z position in cm

  multiplex_speed = sys_time.nb_sec;

  struct UtmCoor_i utm = utm_int_from_gps(&gps, 0);

  //DOWNLINK_SEND_GPS_SMALL(DefaultChannel, DefaultDevice, &multiplex_speed, &utm.east,
   //                         &utm.north, &utm.alt);
}

int swarm_potential_task(void)
{
  swarm_stab.desired_vx = 0.;
  swarm_stab.desired_vy = 0.;

  // compute desired velocity
  int8_t nb = 0;
  uint8_t i;
  for (i = 0; i < acs_idx; i++) {
    if (the_acs[i].ac_id == 0 || the_acs[i].ac_id == AC_ID) { continue; }
    struct ac_info_ * ac = get_ac_info(the_acs[i].ac_id);
    float delta_t = Max((int)(gps.tow - ac->itow) / 1000., 0.);
    // if AC not responding for too long, continue, else compute force
    //if(delta_t > 5) { continue; }
    //else {
      float sha = sinf(ac->course);
      float cha = cosf(ac->course);

      struct UtmCoor_f my_pos = utm_float_from_gps(&gps, 0);

      float de = ac->east - my_pos.east; // + sha * delta_t
      float dn = ac->north - my_pos.north; // cha * delta_t
      float da = ac->alt - my_pos.alt; // ac->climb * delta_t
      float dist2 = de * de + dn * dn + da * da;
      if (dist2 == 0.) { continue; }

      // attractive - repulsive
      // x^2 - d0^3/x
      potential_force.east = de*de - TARGET_DIST3/de;
      potential_force.north = dn*dn - TARGET_DIST3/dn;
      potential_force.alt = da*da - TARGET_DIST3/da;

      // carrot
      swarm_stab.desired_vx += -force_hor_gain * potential_force.east + ac->gspeed * sha;   // add other speed for cohesion
      swarm_stab.desired_vy += -force_hor_gain * potential_force.north + ac->gspeed * cha;
      //potential_force.speed_z += -force_climb_gain * potential_force.alt + ac->climb;

      ++nb;
      potential_force.east = my_pos.east;
      potential_force.north = my_pos.north;
      potential_force.alt = my_pos.alt;
   // }
  }

  potential_force.speed = swarm_stab.desired_vx;
  potential_force.climb = swarm_stab.desired_vy;
  // limit commands
#ifdef GUIDANCE_H_REF_MAX_SPEED
  BoundAbs(swarm_stab.desired_vx, GUIDANCE_H_REF_MAX_SPEED);
  BoundAbs(swarm_stab.desired_vy, GUIDANCE_H_REF_MAX_SPEED);
#endif

  // set commands
  /* Check if we are in the correct AP_MODE before setting commands */
  if (autopilot_mode != AP_MODE_MODULE) {
    return -1;
  }

  // get velocity in body coordinates
  //struct FloatVect3 *ned_vel = (struct FloatVect3)stateGetSpeedNed_f();
  struct FloatVect3 vel;

  float_rmat_vmult(&vel, stateGetNedToBodyRMat_f(), (struct FloatVect3*)stateGetSpeedNed_f());

  /* Calculate the error */
  float err_vx = swarm_stab.desired_vx - vel.x;
  float err_vy = swarm_stab.desired_vy - vel.y;

  /* Calculate the integrated errors */
  swarm_stab.err_vx_int += err_vx / 512;
  swarm_stab.err_vy_int += err_vy / 512;

  BoundAbs(swarm_stab.err_vx_int, CMD_OF_SAT);
  BoundAbs(swarm_stab.err_vy_int, CMD_OF_SAT);

  /* Calculate the commands */
  swarm_stab.cmd.phi   = swarm_stab.phi_pgain * err_vy
                             + swarm_stab.phi_igain * swarm_stab.err_vy_int;
  swarm_stab.cmd.theta = -(swarm_stab.theta_pgain * err_vx
                               + swarm_stab.theta_igain * swarm_stab.err_vx_int);

  /* Bound the roll and pitch commands */
  BoundAbs(swarm_stab.cmd.phi, CMD_OF_SAT);
  BoundAbs(swarm_stab.cmd.theta, CMD_OF_SAT);

  return 1;
}
