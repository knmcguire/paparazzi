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
 * @file "modules/multi/swarm_potential.h"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#ifndef SWARM_POTENTIAL_H
#define SWARM_POTENTIAL_H

#include "math/pprz_algebra_int.h"

struct force_ {
  float east;
  float north;
  float alt;
  float speed;
  float climb;
};

/* The opticflow stabilization */
struct swarm_stab_t {
  int32_t phi_pgain;        ///< The roll P gain on the err_vx
  int32_t phi_igain;        ///< The roll I gain on the err_vx_int
  int32_t theta_pgain;      ///< The pitch P gain on the err_vy
  int32_t theta_igain;      ///< The pitch I gain on the err_vy_int
  float desired_vx;         ///< The desired velocity in the x direction (cm/s)
  float desired_vy;         ///< The desired velocity in the y direction (cm/s)

  float err_vx_int;         ///< The integrated velocity error in x direction (m/s)
  float err_vy_int;         ///< The integrated velocity error in y direction (m/s)
  struct Int32Eulers cmd;   ///< The commands that are send to the hover loop
};
extern struct swarm_stab_t swarm_stab;

extern struct force_ potential_force;

extern float force_hor_gain;
extern float force_climb_gain;
extern float target_dist3;

extern void swarm_potential_init(void);
extern void swarm_potential_periodic(void);
extern int swarm_potential_task(void);

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);

#endif // SWARM_POTENTIAL_H
