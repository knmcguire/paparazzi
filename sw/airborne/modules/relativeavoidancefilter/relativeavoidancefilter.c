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
 * @file "modules/relativeavoidancefilter/relativeavoidancefilter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include "relativeavoidancefilter.h"

#define NSTATES 9  // Number of state values in EKF
#define NMEASUREMENTS 8 // Number of measurement values in EKF
#define RSSINOISE 5.0 // (Expected) Noise on RSSI
#define SPEEDNOISE 0.2 // (Expected) Noise on other measurements

#define CCSIZE 1.2 // Size of collision cone + MAV Size * 2
#define PSISEARCH 30.0 // Search space for Psi

#define STARTTIME 5.0 // Controller start time
#define HEIGHT_DES 1.0 //
#define KP_V 1.0 // Vertical controller gain
#define V_NOMINAL 0.5 // Nominal velocity

#define ASIDE 3.0 // Size of the arena

#ifndef RSSISENDER_ID
	#define RSSISENDER_ID ABI_BROADCAST
#endif

#define NUAVS 3 // Number of UAVs flying (including itself)

ekf_filter ekf[NUAVS-1]; // EKF structure
btmodel model[NUAVS-1];  // Bluetooth model structure 
int IDarray[NUAVS-1]; // Array of IDs of other MAVs
uint32_t now_ts[NUAVS-1]; // Time of last received message from each MAV

uint8_t nf; // Number of filters registered
float psi_des, v_des; // Desired psi_des

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi);
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi)
{
	int i= -1; // Initialize index

	// If it's a new ID we start a new EKF for it
	if ((!array_find_int(2, IDarray, ac_id, &i)) && (nf < NUAVS-1)) {
		IDarray[nf] = ac_id;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(NSTATES,NSTATES, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(NMEASUREMENTS,NMEASUREMENTS, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[nf].Q[0] = 0.01;
		ekf[nf].Q[NSTATES+1] = 0.01;
		ekf[nf].R[0] = pow(RSSINOISE,2.0);
		ekf[nf].X[0] = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1] = 1.0;
		ekf[nf].dt = 0.2;
		model[nf].Pn = -65.0;
		model[nf].gammal = 2.0;
		nf++;
	}

	// If we do recognize the ID, then we can update the measurement message data
	else if (i != -1) {

		// Update the time between messages
		ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6);
		now_ts[i] = get_sys_time_usec();

		// Get the aircraft info for that ID
		struct ac_info_ * ac = get_ac_info(ac_id);
		float trackedVx, trackedVy;
		polar2cart(ac->gspeed, ac->course, &trackedVx, &trackedVy); // get x and y velocities
		
		// Construct measurement vector for EKF using the latest data obtained for each case
		float Y[NMEASUREMENTS];
		Y[0] = rssi; // Bluetooth RSSI measurement
		Y[1] = stateGetSpeedNed_f()->x; // next three are updated in the periodic function
		Y[2] = stateGetSpeedNed_f()->y;
		Y[3] = stateGetNedToBodyEulers_f()->psi;
		Y[4] = trackedVx;
		Y[5] = trackedVy;
		Y[6] = ac->north;
		Y[7] = stateGetPositionNed_f()->z - ac->alt;

		// Run the steps of the EKF
		ekf_filter_predict(&ekf[i], &model[i]);
		ekf_filter_update(&ekf[i], Y);
	}
};

static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;
	for (i = 0; i < nf; i++) {
		pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
			&i, &ekf[i].X[0], &ekf[i].X[1]);
	}
};

void rafilter_init(void)
{   
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	nf = 0; // Number of known objects
	ra_active = true; // Activate avoidance filter
	
	psi_des = 0.4; // Initial desired direction
	v_des = V_NOMINAL; // Initial desired velocity

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);
	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
};

void rafilter_periodic(void)
{
	// Initialize variables
	float vx_des, vy_des;	
	float cc[NUAVS-1][6];

	if(ra_active)
	{
		float posx = stateGetPositionEnu_f()->x;
		float posy = stateGetPositionEnu_f()->y;
		float ownPsi = stateGetNedToBodyEulers_f()->psi; //change 
		bool flagglobal = false; // Null assumption
		
		if ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5))) {
			//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
			// printf("going back inside\n");
			ENUearthToNEDbody(-posx, -posy, ownPsi, &vx_des, &vy_des); // (Gazebo specific?)
			cart2polar(vx_des, vy_des, &v_des, &psi_des);
			v_des = V_NOMINAL;
		}
		else {
			polar2cart(v_des, psi_des, &vx_des, &vy_des);
			uint8_t i;
			for ( i = 0; i < nf; i++ ) {
				float dist = sqrt(pow(ekf[i].X[0],2) + pow(ekf[i].X[1],2));
				collisioncone_update(cc[i], ekf[i].X[0], ekf[i].X[1], ekf[i].X[4], ekf[i].X[5], dist+CCSIZE);
				if ( collisioncone_checkdanger( cc[i], vx_des, vy_des ) )
					flagglobal = true; // We have a problem
			}

			if (flagglobal) { // If the desired velocity doesn't work, then let's find the next best thing according to VO
				v_des = V_NOMINAL;
				collisioncone_findnewcmd(cc, &v_des, &psi_des, PSISEARCH, nf);
			}
		}

		polar2cart(v_des, psi_des, &vx_des, &vy_des);
		NEDbodyToENUearth(vx_des, vy_des, ownPsi, &raavoid_speed_f.y, &raavoid_speed_f.x);
		/*
		x and y need to be inverted.
		My understanding is that a change in x needs to correlate with a change in pitch (y-axis)
		and that a change in y needs to correlate with a change in roll (x-axis)
		which is what the controller sends the commands to
		*/

		// printf("AttCtrl: vx_err: %.1f \t vy_err: %.1f \t vx: %.1f \t vy: %.1f \t vx_cmd: %.1f \t vy_cmd: %.1f \t psi: %.5f \t posx %.1f \t posy %.1f \n", 
		// 	vx - ownVel->x, vy - ownVel->y, ownVel->x, ownVel->y, vx, vy, ownPsi, posx, posy);
	}

};