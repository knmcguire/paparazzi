/*
 * Copyright (C) Mario Coppola
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
 * @file "modules/relativeavoidancefilter/relativeavoidancefilter.h"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#ifndef RELATIVEAVOIDANCEFILTER_H
#define RELATIVEAVOIDANCEFILTER_H

/* Standard Includes */
#include "math.h"
#include "stdlib.h"

/* Paparazzi -- Controller data */
#include "../../state.h" // To get state (pos/vel/acc/...) 
#include "../../firmwares/rotorcraft/navigation.h"
#include "../../firmwares/rotorcraft/guidance/guidance_h.h"
#include "../../firmwares/rotorcraft/autopilot.h"

/* Paparazzi -- Auxiliary stuff */
#include "../../math/pprz_algebra.h"
#include "../../math/pprz_algebra_float.h"
#include "../../math/pprz_algebra_int.h"

/* Own functions */

// #include "functions/PID.h" // Function(s) for a basic PID controller
// #include "functions/coordinateconversions.h"
// // #include "functions/randomgenerator.h" // Functions for random generations of number 

#include "functions/discreteekf.h" // Discrete Extended Kalman Filter base

#include "functions/cstylevector.h" // Vector "object" in C
#include "functions/fmatrix.h" // Matrix of float values functions
// #include "functions/shape.h" // Functions for shape (polygon) manipulation in cartesian plane
// #include "functions/arrayfunctions.h" // Functions for array manipulation

// // #include "functions/humanlike.h" // Functions for human-like obstacle avoidance
// #include "functions/collisioncone.h" // Collision cone functions

#include "functions/filterfunctions.h" // Process + Measurement functions of EKF

extern void rafilter_init(void);
extern void rafilter_periodic(void);

#endif

