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

#define NSTATES 9 
#define NMEASUREMENTS 8
#define AVOIDANCE_CC 1
#define NTAR 2 // Use 2 if flying in empty space, use 6 if flying in room environment
#define NUAVS 3 //
#define PSISEARCH 30 //
#define RSSINOISE 5.0
#define SPEEDNOISE 0.2

#define ASIDE 2 // Size of the arena

#ifndef RSSISENDER_ID
	#define RSSISENDER_ID ABI_BROADCAST
#endif

 float px_des, py_des;
 float vx_des, vy_des;
 float vx, vy;
 float v_des, psi_des;
 float h_des;

 int ntargets = NTAR;
 int nf;
 int IDarray[NUAVS-1];

 ekf_filter ekf[NUAVS-1];
 float Y[NMEASUREMENTS];

 int flag[NUAVS-1];
 int flagglobal;
 int flagglobalcounter;
 float cc[NUAVS-1][6];
 int arenaside;

BTmessage msg;

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi);
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi)
{
 	msg.ID   = ac_id;
 	msg.RSSI = rssi;
};

void rafilter_init(void)
{
	randomgen_init(); // Initialize random seed

	flagglobal = 0;
	flagglobalcounter = 0;
	array_make_zeros_int(NUAVS-1, flag);
	
	ntargets = NTAR;
	nf = 0;
	arenaside = ASIDE;

	array_make_zeros_int(NUAVS-1, IDarray);

	getNewGoalPos(&px_des, &py_des, arenaside);
};

void rafilter_periodic(void)
{
	AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);

	// If it's a new ID we start a new EKF for it
	if ((array_isvaluein_int(2,IDarray,msg.ID) == 0) 
		&& (nf < NUAVS-1))
	{
		IDarray[nf] = msg.ID;

		// Initialize an ekf filter for each target tracker			
		ekf_filter_new(&ekf[nf]);

		// Set up the Q and R matrices.
		fmat_scal_mult(NSTATES,NSTATES, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(NMEASUREMENTS,NMEASUREMENTS, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[i].Q[0] = 0.01;
		ekf[i].Q[NSTATES+1] = 0.01;
		ekf[i].R[0] = RSSINOISE;

		// Initial positions cannot be zero or else you'll divide by zero
		ekf[i].X[0] = 1.0; 
		ekf[i].X[1] = 1.0;
		ekf[i].dt = 0.2;

		nf++;
	}

	struct NedCoor_f *ownPos = stateGetPositionNed_f();
	struct NedCoor_f *ownVel = stateGetSpeedNed_f();

	for (int i = 0; i < nf; i++)
	{
		// Get the Bluetooth message
		AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);

		// Construct measurement vector for EKF
		Y[0] = -60.0; // Bluetooth RSSI measurement
		Y[1] = ownVel->x;
		Y[2] = ownVel->y;
		Y[3] = 0.0;
		Y[4] = 0.0;
		Y[5] = 0.0;
		Y[6] = 0.0;
		Y[7] = 0.0;

		// Run the steps of the EKF
		ekf_filter_predict(&ekf[i]);
		ekf_filter_update(&ekf[i], Y);
	}

	float h_err = 0.0;
	float ownPsi = M_PI;

	if(abs(h_err) < 0.10)
	{
		vx = PID( -0.25, 0.0, 0.0, ownPos->x - px_des, 0.0, 0.0 );
		vy = PID( -0.25, 0.0, 0.0, ownPos->y - py_des, 0.0, 0.0 );

		ENUearthToNEDbody(vx, vy, ownPsi, &vx_des, &vy_des);	
		flagglobal = 0;

		for (int i = 0; i < nf; i++)
		{
			collisioncone_update(cc[i],
				ekf[i].X[0], ekf[i].X[1],
				ekf[i].X[4], ekf[i].X[5], 2.0);

			flag[i] = collisioncone_checkdanger( cc[i], vx_des, vy_des );

			if ( flag[i] == 1 )
				flagglobal = 1;
		}

		if (flagglobal == 1)
		{
			if (flagglobalcounter > 10)
				getNewGoalPos(&px_des, &py_des, arenaside);
			else
				flagglobalcounter++;

			cart2polar(vx_des, vy_des, &v_des, &psi_des);
			collisioncone_findnewcmd(cc, &v_des, &psi_des, PSISEARCH, nf);
			polar2cart(v_des, psi_des, &vx_des, &vy_des);
			NEDbodyToENUearth(vx_des, vy_des, ownPsi, &vx, &vy);

		}

	}
	
	if ((abs(px_des - ownPos->x) < 0.2)
		&& (abs(py_des - ownPos->y) < 0.2))
	{
		getNewGoalPos(&px_des, &py_des, arenaside);
		flagglobalcounter = 0;
	}

};

