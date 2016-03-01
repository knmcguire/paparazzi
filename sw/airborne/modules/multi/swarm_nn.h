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
 * @file "modules/multi/swarm_nn.h"
 * @author Kirk Scheper
 * This module is generates a command to avoid other vehicles based on their relative gps location
 */

#ifndef SWARM_NN_H
#define SWARM_NN_H

#include "math/pprz_algebra_int.h"

struct force_ {
  float east;
  float north;
  float alt;
  float speed;
  float climb;
};

extern struct force_ potential_force;

extern float force_hor_gain;
extern float force_climb_gain;
extern float target_dist3;
extern bool use_waypoint;

extern void swarm_nn_init(void);
extern void swarm_nn_periodic(void);
extern int swarm_nn_task(void);

#endif // swarm_nn_H
