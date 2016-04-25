/* * Copyright (C) Mario Coppola * * This file is part of paparazzi * * paparazzi is free software; you can redistribute it and/or modify * it under the terms of the GNU General Public License as published by * the Free Software Foundation; either version 2, or (at your option) * any later version. * * paparazzi is distributed in the hope that it will be useful, * but WITHOUT ANY WARRANTY; without even the implied warranty of * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the * GNU General Public License for more details. * * You should have received a copy of the GNU General Public License * along with paparazzi; see the file COPYING.  If not, see * <http://www.gnu.org/licenses/>. *//** * @file "modules/relativeavoidancefilter/relativeavoidancefilter.h" * @author Mario Coppola * Relative Localization Filter for collision avoidance between drones */#ifndef RELATIVEAVOIDANCEFILTER_H#define RELATIVEAVOIDANCEFILTER_H	/* Paparazzi -- Controller data */#include "../../state.h" // To get state (pos/vel/acc/...)  #include "../../firmwares/rotorcraft/navigation.h"#include "../../firmwares/rotorcraft/guidance/guidance_h.h"#include "../../firmwares/rotorcraft/guidance/guidance_v.h"#include "../../firmwares/rotorcraft/autopilot.h"#include "modules/multi/traffic_info.h"#include "subsystems/datalink/telemetry.h"// ABI messages#include "subsystems/abi.h"/* Paparazzi -- Auxiliary stuff */#include "../../math/pprz_algebra.h"#include "../../math/pprz_algebra_float.h"#include "../../math/pprz_algebra_int.h"/* Own functions *//* Standard Includes */#include <math.h>#include <stdio.h>#include <stdlib.h>// Own C includes#include "functions/arrayfunctions.h" // Array operations#include "functions/shape.h" // Polygon shapes operations#include "functions/fmatrix.h" // Matrix operations#include "functions/PID.h" // PID controller functions#include "functions/coordinateconversions.h" // Coordinate frame operations#include "functions/discreteekf.h" // Discrete EKF operations#include "functions/collisioncone.h" // Collision cone (VO)#include "functions/randomgenerator.h" // Collision cone (VO)extern void rafilter_init(void);extern void rafilter_periodic(void);#endif