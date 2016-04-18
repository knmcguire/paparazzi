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

#define PSISEARCH 30.0 // Search space for Psi
#define STARTTIME 5.0 // Controller start time

#ifndef RSSISENDER_ID
	#define RSSISENDER_ID ABI_BROADCAST
#endif

ekf_filter ekf[NUAVS-1]; // EKF structure
btmodel model[NUAVS-1];  // Bluetooth model structure 
int IDarray[NUAVS-1]; // Array of IDs of other MAVs
uint32_t now_ts[NUAVS-1]; // Time of last received message from each MAV

uint8_t nf; // Number of filters registered
float psi_des, v_des; // Desired psi_des

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi);
// static float rand_normal(float mean, float stdev);

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength __attribute__((unused)), int8_t rssi)
{
	int i= -1; // Initialize index

	// If it's a new ID we start a new EKF for it
	if ((!array_find_int(2, IDarray, ac_id, &i)) && (nf < NUAVS-1)) {
		IDarray[nf] = ac_id;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(9,9, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(8,8, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[nf].Q[0] = 0.01;
		ekf[nf].Q[9+1] = 0.01;
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
		float Y[8];
		Y[0] = rssi + rand_normal(0.0, 5.0); // Bluetooth RSSI measurement
		Y[1] = stateGetSpeedNed_f()->x + rand_normal(0.0, 0.2); // next three are updated in the periodic function
		Y[2] = stateGetSpeedNed_f()->y + rand_normal(0.0, 0.2);
		Y[3] = stateGetNedToBodyEulers_f()->psi + rand_normal(0.0, 0.2);
		Y[4] = 0.0 + rand_normal(0.0, 0.2); //trackedVx;
		Y[5] = 0.0 + rand_normal(0.0, 0.2); //trackedVy;
		Y[6] = 0.0 + rand_normal(0.0, 0.2); //ac->north;
		Y[7] = 0.0 + rand_normal(0.0, 0.2); //stateGetPositionNed_f()->z - ac->alt;

		// Run the steps of the EKF
		ekf_filter_predict(&ekf[i], &model[i]);
		ekf_filter_update(&ekf[i], Y);
		// MESSAGE RECEIVED!
		// printf("message received with rssi %d for drone %d number %d \n", rssi,ac_id,i);
		printf("\n");
		printf("Y: \t");
		fmat_print(1, 8, Y);
		printf("X: \t");
		fmat_print(1, 9, ekf[i].X);

	}
};

static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;
	for (i = 0; i < nf; i++) {
		pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
			&i, &ekf[i].X[0], &ekf[i].X[1], &ekf[i].X[8]);
	}
};

void rafilter_init(void)
{   
	randomgen_init();
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	nf = 0; // Number of known objects
	ra_active = true; // Activate avoidance filter
	
	psi_des = 0.0; // Initial desired direction
	v_des = V_NOMINAL; // Initial desired velocity

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(RSSISENDER_ID, &rssi_ev, bluetoothmsg_cb);
	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
};



int ii = 0;
#define SENDER_ID 1

void rafilter_periodic(void)
{	
	// FAKE MESSAGE for testing purposes from the 0.0 position!
	float d = sqrt(pow(stateGetPositionEnu_f()->x,2)+pow(stateGetPositionEnu_f()->y,2));
	AbiSendMsgRSSI(SENDER_ID, 2, 2,  (int)(-65 -2*10*log10(d)));

	if (ii < 100)
		guidance_h_mode_changed(4);
	else
		ii++;

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

		// if (guidance_h.mode == GUIDANCE_H_MODE_NAV) {
			polar2cart(v_des, psi_des, &vx_des, &vy_des);
			NEDbodyToENUearth(vx_des, vy_des, ownPsi, &raavoid_speed_f.y, &raavoid_speed_f.x);
		// }
		// else if (guidance_h.mode == GUIDANCE_H_MODE_HOVER) {
			// polar2cart(v_des, psi_des, &raavoid_speed_f.x, &raavoid_speed_f.y);
		// }
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