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
#include "modules/datalink/extra_pprz_dl.h"

#define PSISEARCH 15.0 		// Search grid for psi_des

ekf_filter ekf[NUAVS-1]; 	// EKF structure
btmodel model[NUAVS-1];  	// Bluetooth model structure 
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV

uint8_t nf; 				// Number of filters registered
float psi_des, v_des; 		// psi_des = desired course w.r.t. north, v_des = magnitude of velocity
float vx_des, vy_des;		// Desired velocities in NED frame
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
bool firsttime;
float calpsi;

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{
	int i= -1; // Initialize index
	printf("message received with rssi %d for drone %d number %d \n", rssi,ac_id,i);

	// If it's a new ID we start a new EKF for it
	if ((!array_find_int(2, IDarray, ac_id, &i)) && (nf < NUAVS-1)) {
		IDarray[nf] = ac_id;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(9,9, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(8,8, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[nf].Q[0]   = 0.01;
		ekf[nf].Q[9+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
		ekf[nf].X[0]   = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1]   = 1.0;
		ekf[nf].dt     = 0.2;
		model[nf].Pn   = -63.0 - 8.0 + source_strength; // -63 is from calibration with antenna that had 8dB power, so the source_strength recalibrates
		model[nf].gammal = 2.0;
		nf++;
	}

	// If we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == 1) ) {

		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		{
			// Update the time between messages
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6);
			now_ts[i] = get_sys_time_usec();
			
			// Get the aircraft info for that ID
			struct ac_info_ * ac = get_ac_info(ac_id);
			float trackedVx, trackedVy;
			float angle;
			deg2rad(ac->course/10, &angle); // decidegrees
			polar2cart(ac->gspeed/100, angle, &trackedVx, &trackedVy); // get x and y velocities (cm/s)

			// Construct measurement vector for EKF using the latest data obtained for each case
			float Y[8];
			RSSIarray[i] = rssi;
			Y[0] = rssi;// + rand_normal(0.0, 5.0); // Bluetooth RSSI measurement
			Y[1] = stateGetSpeedNed_f()->x + rand_normal(0.0, 0.2); // next three are updated in the periodic function
			Y[2] = stateGetSpeedNed_f()->y + rand_normal(0.0, 0.2);
			Y[3] = 0.0 + rand_normal(0.0, 0.2);
			Y[4] = trackedVx + rand_normal(0.0, 0.2);
			Y[5] = trackedVy + rand_normal(0.0, 0.2);
			Y[6] = 0.0 + rand_normal(0.0, 0.2); //ac->north;
			Y[7] = 0.0 + rand_normal(0.0, 0.2); //stateGetPositionNed_f()->z - ac->alt;

			// Run the steps of the EKF
			ekf_filter_predict(&ekf[i], &model[i]);
			ekf_filter_update (&ekf[i], Y);

		}
	}
};

static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;
	for (i = 0; i < nf; i++) {
		pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
			&i, &RSSIarray[i], 
			&ekf[i].X[0], &ekf[i].X[1], 
			&ekf[i].X[2], &ekf[i].X[3],
			&ekf[i].X[4], &ekf[i].X[5],
			&vx_des, &vy_des);
	}
};

void rafilter_init(void)
{
	firsttime = true;
	randomgen_init();    // Initialize the random generator (for simulation purposes)
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	nf = 0; 		      // Number of known objects
	

	psi_des = 0.0;
	v_des   = V_NOMINAL; // Initial desired velocity

	calpsi = getrand_float(-M_PI, M_PI); // Initial desired direction

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(ABI_BROADCAST, &rssi_ev, bluetoothmsg_cb);

	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
};

float timer, timerstart, t_elps;

void rafilter_periodic(void)
{	
	// FAKE MESSAGE for testing purposes from the 0.0 position if a BT source is not available!
	// float d = sqrt(pow(stateGetPositionEnu_f()->x,2)+pow(stateGetPositionEnu_f()->y,2));
	// AbiSendMsgRSSI(1, 2, 2,  (int)(-65 -2*10*log10(d)));

	/*********************************************
		Initialize variables
	*********************************************/
	float cc[NUAVS-1][6];
	float temp, course;

	float posx = stateGetPositionNed_f()->x;
	float posy = stateGetPositionNed_f()->y;

	float velx = stateGetSpeedNed_f()->x;
	float vely = stateGetSpeedNed_f()->y;

	cart2polar(velx, vely, &temp, &course);

	
	/*********************************************
		Sending speed directly between drones
	*********************************************/

	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
	                        21; // bits 31-21 x position in cm
	multiplex_speed |= (((uint32_t)(gps.gspeed)) & 0x7FF) << 10;         // bits 20-10 y position in cm
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);               // bits 9-0 z position in cm

	int16_t alt = (int16_t)(gps.hmsl / 10);

	DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat,
                          &gps.lla_pos.lon, &alt);


	/*********************************************
		Relative Avoidance Behavior
	*********************************************/

	if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
		timerstart = get_sys_time_usec()/pow(10,6); 		// Elapsed time
		v_des = 0.0;
	}
	else 
	{
		timer  = get_sys_time_usec()/pow(10,6);
		t_elps = timer - timerstart;

		/*********************************************
			Calibration maneuver
		*********************************************/
		if ( t_elps < 4.0 && firsttime ) {
			float vcal = 1.0;
			if      ( t_elps < 1.0 ) { psi_des =  calpsi            ; v_des = vcal; }
			else if ( t_elps < 2.0 ) { psi_des =  calpsi+(M_PI/2)   ; v_des = vcal; }
			else if ( t_elps < 3.0 ) { psi_des =  calpsi+(M_PI)     ; v_des = vcal; }
			else if ( t_elps < 4.0 ) { psi_des =  calpsi+(3*M_PI/2) ; v_des = vcal; }
		}

		else {
			firsttime = false;	
			bool flagglobal = false; // Null assumption
			
			if ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5))) {
				//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
				cart2polar(0-posx,0-posy, &v_des, &psi_des);
				v_des = V_NOMINAL;
			}
			else {
				polar2cart(v_des, psi_des, &vx_des, &vy_des);
				uint8_t i;
				for ( i = 0; i < nf; i++ ) {
					float dist = sqrt(pow(ekf[i].X[0],2) + pow(ekf[i].X[1],2));
					collisioncone_update(cc[i], ekf[i].X[0], ekf[i].X[1], ekf[i].X[4], ekf[i].X[5], dist+CCSIZE);
					if ( collisioncone_checkdanger( cc[i], vx_des, vy_des ) )
						flagglobal = true; // We could be colliding!
				}
		
				if (flagglobal) { // If the desired velocity doesn't work, then let's find the next best thing according to VO
					v_des = V_NOMINAL;
					collisioncone_findnewcmd(cc, &v_des, &psi_des, PSISEARCH, nf);
				}
			}

		}

		polar2cart(v_des, psi_des, &vx_des, &vy_des);
		autopilot_guided_move_ned(vx_des, vy_des, 0.0, 0.0);

	}

};