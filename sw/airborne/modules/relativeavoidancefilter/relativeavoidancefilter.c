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

#include "modules/relativeavoidancefilter/relativeavoidancefilter.h"

#define NSTATES 9  // Number of state values in EKF
#define NMEASUREMENTS 8 // Number of measurement values in EKF
#define CCSIZE 3.0 // Size of collision cone
#define NTAR 2 // Use 2 if flying in empty space, use 6 if flying in room environment
#define NUAVS 3 // Number of UAVs flying (including itself)
#define PSISEARCH 30.0 // Search space for Psi
#define RSSINOISE 5.0 // (Expected) Noise on RSSI
#define SPEEDNOISE 0.2 // (Expected) Noise on other measurements
#define HEIGHT_DES 1.0 //
#define STARTTIME 5.0 // Controller start time
#define KP_H 0.25 // Horizontal controller gain
#define KP_V 1.0 // Vertical controller gain
#define V_NOMINAL 1.0 // Nominal velocity

#define ASIDE 3.0 // Size of the arena

#ifndef RSSISENDER_ID
	#define RSSISENDER_ID ABI_BROADCAST
#endif

ekf_filter ekf[NUAVS-1];
int ntargets, nf;
int IDarray[NUAVS-1];
bool dirchanged;
BTmessage msg[NUAVS-1];

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi);
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi)
{
	int index = -1; // initialize index

	// If it's a new ID we start a new EKF for it
	if ((!array_find_int(2, IDarray, ac_id, &index)) && (nf < NUAVS-1)) {
		IDarray[nf] = ac_id;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices.
		fmat_scal_mult(NSTATES,NSTATES, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(NMEASUREMENTS,NMEASUREMENTS, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[nf].Q[0] = 0.01;
		ekf[nf].Q[NSTATES+1] = 0.01;
		ekf[nf].R[0] = RSSINOISE;
		ekf[nf].X[0] = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1] = 1.0;
		ekf[nf].dt = 0.2;
		nf++;
	}
	else if (index != -1) { // Store latest message
	 	msg[index].ID   = ac_id;
	 	msg[index].RSSI = rssi;
	}

};

// absolute

float absolute(float a)
{
	if (a < 0)
		return -a;
	if (a > 0)
		return a;
	else
		return a;
}

void rafilter_init(void)
{
	ntargets = NTAR;
	nf = 0;
	array_make_zeros_int(NUAVS-1, IDarray);
	dirchanged = false;
	ra_active = false;
};

float psi_des;
void rafilter_periodic(void)
{
	// Initialize variables
	float posx, posy, posz, ownVx, ownVy, ownPsi;
	float vx_des, vy_des, v_des, vx, vy;	
	float cc[NUAVS-1][6];
	float trackedVx, trackedVy, trackedPsi, trackedz;
	float Y[NMEASUREMENTS];
	int flag[NUAVS-1];
	bool flagglobal = false;
	array_make_zeros_int(NUAVS-1, flag);
	v_des = V_NOMINAL;

	// Own data	
	struct EnuCoor_f *ownPos = stateGetPositionEnu_f();
	struct EnuCoor_f *ownVel = stateGetSpeedEnu_f();
	posx = ownPos->x;
	posy = ownPos->y;
	posz = ownPos->z;
	ownVx = ownVel->x;
	ownVy = ownVel->y;
	ownPsi = stateGetNedToBodyEulers_f()->psi; //change 

	AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);

	for (int i = 0; i < nf; i++)
	{
		// Get the Bluetooth message
		AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);
		trackedVx = 0.0;
		trackedVy = 0.0;
		trackedz = 0.0;
		trackedPsi = M_PI;
	
		// Construct measurement vector for EKF
		Y[0] = msg[i].RSSI; // Bluetooth RSSI measurement
		Y[1] = ownVx;
		Y[2] = ownVy;
		Y[3] = ownPsi;
		Y[4] = trackedVx;
		Y[5] = trackedVy;
		Y[6] = trackedPsi;
		Y[7] = posz - trackedz;

		// Run the steps of the EKF
		ekf_filter_predict(&ekf[i]);
		ekf_filter_update(&ekf[i], Y);
	}

	if(ra_active)
	{
		// printf("aposx %.2f aposy %.2f \n", absolute(posx), absolute(posy));
		if ((absolute(posx) > (ASIDE-0.5)) || (absolute(posy) > (ASIDE-0.5))) {
			//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
			// printf("going back inside\n");
			ENUearthToNEDbody(-posx, -posy, ownPsi, &vx_des, &vy_des); // (Gazebo specific?)
			cart2polar(vx_des, vy_des, &v_des, &psi_des);
			v_des = V_NOMINAL;
		}
		else {
			// wrapToPi(&psi_des); // Keep current heading
			v_des = V_NOMINAL; // Fix to nominal velocity

			polar2cart(v_des, psi_des, &vx_des, &vy_des);
			flagglobal = false; // Null assumption
			for (int i = 0; i < nf; i++) {
				collisioncone_update(cc[i], ekf[i].X[0], ekf[i].X[1], ekf[i].X[4], ekf[i].X[5], CCSIZE);
				flag[i] = collisioncone_checkdanger( cc[i], vx_des, vy_des );
				if ( flag[i] == 1 )
					flagglobal = true;
			}

			if (flagglobal) { // If the desired velocity doesn't work, then let's find the next best thing according to VO
				collisioncone_findnewcmd(cc, &v_des, &psi_des, PSISEARCH, nf);
			}
		}

		polar2cart(v_des, psi_des, &vx_des, &vy_des);
		NEDbodyToENUearth(vx_des, vy_des, ownPsi, &vx, &vy);
		raavoid_speed_f.x = vy;
		raavoid_speed_f.y = vx;

		// printf("AttCtrl: vx_err: %.1f \t vy_err: %.1f \t vx: %.1f \t vy: %.1f \t vx_cmd: %.1f \t vy_cmd: %.1f \t psi: %.5f \t posx %.1f \t posy %.1f \n", 
		// 	vx - ownVel->x, vy - ownVel->y, ownVel->x, ownVel->y, vx, vy, ownPsi, posx, posy);
	}
};

