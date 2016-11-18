/*
 * Copyright (C) 2016 - IMAV 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/flight_plan_guided.c
 * @author IMAV 2016
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>


#include "mcu_periph/uart.h"
#include "modules/stereocam/stereocam.h"
#include "modules/stereocam/stereoprotocol.h"
#include "modules/stereocam/stereocam2state/stereocam2state.h"

// start and stop modules
#include "generated/modules.h"

#ifdef INS_BARO_AGL_OFFSET
#define LEGS_HEIGHT INS_BARO_AGL_OFFSET
#else
#define LEGS_HEIGHT 0.2
#endif

#define NOM_FLIGHT_ALT 1.7  // nominal flight altitude
float nom_flight_alt; // nominal flight altitude

#include "subsystems/abi.h"
static abi_event agl_ev;
static float filtered_agl = LEGS_HEIGHT;
static void agl_cb(uint8_t sender_id, float agl);

static void agl_cb(uint8_t sender_id, float agl)
{
  filtered_agl = filtered_agl * 0.9 + agl * 0.1;
}

void flight_plan_guided_init(void)
{
  nom_flight_alt = NOM_FLIGHT_ALT;
  AbiBindMsgAGL(1, &agl_ev, agl_cb); // ABI to the altitude above ground level
}


/* Kill throttle */
uint8_t KillEngines(void)
{
  autopilot_set_motors_on(FALSE);

  return false;
}


/* Start throttle */
uint8_t StartEngines(void)
{
  autopilot_set_motors_on(TRUE);

  return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {ins_reset_altitude_ref(); return false;}

bool TakeOff(float climb_rate)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_v_set_guided_vz(-climb_rate);
  guidance_h_set_guided_body_vel(0, 0);

  return false;
}


bool WaitUntilAltitude(float altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z < altitude) { return true; }

  return false;
}


bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  if (stateGetSpeedEnu_f()->z < speed) { return true; }

  return false;
}

float specialtimer = 0;

bool ResetSpecialTimer(void)
{
  specialtimer = 0;
  return false;
}

bool WaitUntilTimerOrAltitude(float sec, float fail_altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  specialtimer += 1.0f / ((float)NAV_FREQ);
  if (specialtimer < sec) { return true; }

  return false;
}


bool RotateToHeading(float heading)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading(heading);
  return false;
}


bool WaitforHeading(float heading)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading_rate(1.);
  if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.1) {
    guidance_h_set_guided_heading(heading);
    return false;
  }

  return true;
}


uint8_t Hover(float alt)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }
  // Horizontal velocities are set to zero
  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
  guidance_h_set_guided_body_vel(0, 0);
  guidance_v_set_guided_z(-alt);

  return false;
}

/* Move forward */
uint8_t MoveForward(float vx)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(vx, 0);
  }
  return false;
}

/* Move Right */
uint8_t MoveRight(float vy)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(0, vy);
  }
  return false;
}



