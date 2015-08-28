/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow/stabilization_opticflow.c
 * @brief Optical-flow based control for Linux based systems
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

// Own Header
#include "stabilization_opticflow.h"
#include "../opticflow_module.h"
// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/electrical.h"
#include "std.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/radio_control/rc_datalink.h"

//#ifndef CMD_OF_SAT
#define CMD_OF_SAT  1500 // 40 deg = 2859.1851
//#endif

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 20
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)

#ifndef VISION_DESIRED_VX
#define VISION_DESIRED_VX 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VX)

#ifndef VISION_DESIRED_VY
#define VISION_DESIRED_VY 0
#endif
PRINT_CONFIG_VAR(VISION_DESIRED_VY)

/* Check the control gains */
#if (VISION_PHI_PGAIN < 0)      ||  \
		(VISION_PHI_IGAIN < 0)        ||  \
		(VISION_THETA_PGAIN < 0)      ||  \
		(VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

/* Initialize the default gains and settings */
struct opticflow_stab_t opticflow_stab = {
		.phi_pgain = VISION_PHI_PGAIN,
		.phi_igain = VISION_PHI_IGAIN,
		.theta_pgain = VISION_THETA_PGAIN,
		.theta_igain = VISION_THETA_IGAIN,
		.desired_vx = VISION_DESIRED_VX,
		.desired_vy = VISION_DESIRED_VY
};

int control_check_roll;
int control_check_pitch;


/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
	/* Reset the integrated errors */
	opticflow_stab.err_vx_int = 0;
	opticflow_stab.err_vy_int = 0;

	/* Set rool/pitch to 0 degrees and psi to current heading */
	opticflow_stab.cmd.phi = 0;
	opticflow_stab.cmd.theta = 0;
	opticflow_stab.cmd.psi = stateGetNedToBodyEulers_i()->psi;

	control_check_roll=0;
	control_check_pitch=0;

}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{

	// TODO: change the desired vx/vy

//#ifndef TUNING
		///Pgain with RC
		float control_roll= radio_control.values[RADIO_ROLL]*0.2f;


		//printf("control_roll: %f %d\n",control_roll,control_check);
		if(control_roll>0&&control_check_roll!=1)
		{
			opticflow_stab.phi_pgain=opticflow_stab.phi_pgain+10;
			opticflow_stab.theta_pgain=opticflow_stab.theta_pgain+10;
			control_check_roll=1;
		}
		else
		{
			if(control_roll<0&&control_check_roll!=1)
			{
				opticflow_stab.phi_pgain=opticflow_stab.phi_pgain-10;
				opticflow_stab.theta_pgain=opticflow_stab.theta_pgain-10;
				control_check_roll=1;
			}
			else{}
		}
		if(control_roll==0)
		{
			control_check_roll=0;
		}

		///Igain with RC
		float control_pitch= radio_control.values[RADIO_PITCH]*0.2f;
		if(control_pitch<0&&control_check_pitch!=1)
		{
			opticflow_stab.phi_igain=opticflow_stab.phi_igain+10;
			opticflow_stab.theta_igain=opticflow_stab.theta_igain+10;
			control_check_pitch=1;
		}
		else
		{
			if(control_pitch>0&&control_check_pitch!=1)
			{
				opticflow_stab.phi_igain=opticflow_stab.phi_igain-10;
				opticflow_stab.theta_igain=opticflow_stab.theta_igain-10;
				control_check_pitch=1;
			}
			else{}
		}
		if(control_pitch==0)
		{
			control_check_pitch=0;
		}
//#endif

}




/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
	/* Update the setpoint */
	stabilization_attitude_set_rpy_setpoint_i(&opticflow_stab.cmd);

	/* Run the default attitude stabilization */
	stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] *result The opticflow calculation result used for control
 */
void stabilization_opticflow_update(struct opticflow_result_t *result)
{
	/* Check if we are in the correct AP_MODE before setting commands */
	if (autopilot_mode != AP_MODE_MODULE) {
		return;
	}

	/* Calculate the error if we have enough flow */
	float err_vx = 0;
	float err_vy = 0;
	if (result->tracked_cnt > 0) {
		err_vx = opticflow_stab.desired_vx - result->vel_x;
		err_vy = opticflow_stab.desired_vy - result->vel_y;
	}

	/* Calculate the integrated errors (TODO: bound??) */
	opticflow_stab.err_vx_int += err_vx /512/ 100;
	opticflow_stab.err_vy_int += err_vy /512/ 100;

	/* Calculate the commands */
	opticflow_stab.cmd.phi   = (opticflow_stab.phi_pgain * err_vx / 100
			+ opticflow_stab.phi_igain * opticflow_stab.err_vx_int);
	opticflow_stab.cmd.theta = -(opticflow_stab.theta_pgain * err_vy / 100
			+ opticflow_stab.theta_igain * opticflow_stab.err_vy_int);

	/* Bound the roll and pitch commands */
	BoundAbs(opticflow_stab.cmd.phi, CMD_OF_SAT);
	BoundAbs(opticflow_stab.cmd.theta, CMD_OF_SAT);
}
