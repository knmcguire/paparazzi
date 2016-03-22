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

/* Standard Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Own C includes
#include "functions/PID.h" // Function(s) for a basic PID controller
#include "functions/coordinateconversions.h"
#include "functions/randomgenerator.h" // Functions for random generations of number 

// Includes borrowed from Paparazzi
#include "functions/discreteekf.h"
#include "functions/cstylevector.h"
#include "functions/collisioncone.h"
#include "functions/fmatrix.h"
#include "functions/shape.h"
#include "functions/arrayfunctions.h"
#include "functions/humanlike.h"

extern void rafilter_init(void);
extern void rafilter_periodic(void);

#endif

