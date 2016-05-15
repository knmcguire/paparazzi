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

#define PSISEARCH 15.0 		// Search grid for psi_des
#define VCAL 1.0			// Velocity during calibration round
#define NUAVS 5			// Maximum expected number of drones

ekf_filter ekf[NUAVS-1]; 	// EKF structure
btmodel model[NUAVS-1];  	// Bluetooth model structure 
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
int8_t srcstrength[NUAVS-1];  // Source strength
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV

int nf; 				// Number of filters registered
float psi_des, v_des; 		// psi_des = desired course w.r.t. north, v_des = magnitude of velocity
float vx_des, vy_des;		// Desired velocities in NED frame
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
bool firsttime;			// Bool value that checks when the script becomes active for the first time
float calpsi;				// Calibration heading in the beginning
float magprev;				// Previous magnitude from 0,0 (for simulated wall detection)

float timer, timerstart, t_elps;

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{
	int i= -1; // Initialize index
	printf("message received with rssi %d for drone %d number %d \n", rssi, ac_id, i);
	// If it's a new ID we start a new EKF for it
	if ((!array_find_int(NUAVS-1, IDarray, ac_id, &i)) && (nf < NUAVS-1)) {
		IDarray[nf] = ac_id;
		srcstrength[nf] = source_strength;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(9,9, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(8,8, ekf[nf].R, pow(SPEEDNOISE,2), ekf[nf].R);
		ekf[nf].Q[0]   = 0.01;
		ekf[nf].Q[9+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
		ekf[nf].X[0]   = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1]   = 1.0;
		ekf[nf].dt     = 0.2; // Initial assumption (STDMA code runs at 5Hz)
		model[nf].Pn   = -63.0 - 8.0 + (float)source_strength; // -63 is from calibration with antenna that had 8dB power, so the source_strength recalibrates
		model[nf].gammal = 2.0;
		nf++;
	}

	// If we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) ) {
		RSSIarray[i] = rssi;

		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		{
			// Update the time between messages
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6);
			now_ts[i] = get_sys_time_usec();
			
			// Get the aircraft info for that ID
			struct ac_info_ * ac = get_ac_info(ac_id);
			float angle, trackedVx, trackedVy;
			
			deg2rad   (((float)ac->course)/10.0, &angle); // decidegrees
			polar2cart(((float)ac->gspeed)/100.0, angle, &trackedVx, &trackedVy); // get x and y velocities (cm/s)

			// See UtmCoor for north pos in z
			// Construct measurement vector for EKF using the latest data obtained for each case

			float Y[8];
			Y[0] = (float)rssi;// + rand_normal(0.0, 5.0); // Bluetooth RSSI measurement
			Y[1] = stateGetSpeedNed_f()->x + rand_normal(0.0, 0.2); // next three are updated in the periodic function
			Y[2] = stateGetSpeedNed_f()->y + rand_normal(0.0, 0.2);
			Y[3] = 0.0 + rand_normal(0.0, 0.2);
			Y[4] = trackedVx + rand_normal(0.0, 0.2);
			Y[5] = trackedVy + rand_normal(0.0, 0.2);
			Y[6] = 0.0 + rand_normal(0.0, 0.2); //ac->north;
			Y[7] = (-(float)ac->utm.alt/1000.0) - (stateGetPositionNed_f()->z); //utm.alt is in mm above 0
			
			// Run the steps of the EKF
			ekf_filter_predict(&ekf[i], &model[i]);
			ekf_filter_update (&ekf[i], Y);

		}
	}
};

bool alternate;
static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;

	if (nf > 1)
	{
		if (alternate)
		{
			alternate = false;
			i = 1;
		}
		else
		{
			alternate = true;
			i = 0;
		}
	}
	else 
	{
			i = 0;
	}
	
	// for (i = 0; i < nf; i++) {
		pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
			&IDarray[i],			    // ID or filtered aircraft number
			&RSSIarray[i], 		    // Received ID and RSSI
			&srcstrength[i],		    // Source strength
			&ekf[i].X[0], &ekf[i].X[1],  // x and y pos
			&ekf[i].X[2], &ekf[i].X[3],  // Own vx and vy
			&ekf[i].X[4], &ekf[i].X[5],  // Received vx and vy
			&ekf[i].X[6], &ekf[i].X[7],  // Orientation own , orientation other
			&ekf[i].X[8], 			    // Height separation
			&vx_des, &vy_des);		    // Commanded velocities
	// }
};

void rafilter_init(void)
{
	randomgen_init();    // Initialize the random generator (for simulation purposes)
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	
	alternate = 0;
	nf 		= 0; 	   // Number of active filters
	psi_des   = 0.0;	   // Initialize
	v_des     = V_NOMINAL; // Initial desired velocity
	firsttime = true;
	magprev   = 3.0; 	   // Just a random high value
	calpsi    = getrand_float(-M_PI, M_PI); // Initial desired direction

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(ABI_BROADCAST, &rssi_ev, bluetoothmsg_cb);

	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
};


void rafilter_periodic(void)
{	
	// FAKE MESSAGE for testing purposes from the 0.0 position if a BT source is not available!
	// float d = sqrt(pow(stateGetPositionEnu_f()->x,2)+pow(stateGetPositionEnu_f()->y,2));
	// AbiSendMsgRSSI(1, 2, 2,  (int)(-63 -2*10*log10(d)));

	/*********************************************
		Sending speed directly between drones
	*********************************************/
	float velx = stateGetSpeedNed_f()->x;
	float vely = stateGetSpeedNed_f()->y;
	float temp, crs;

	// Convert Course to the proper format
	cart2polar(velx, vely, &temp, &crs); 	// Get the total speed and course
	wrapTo2Pi(&crs); 					// Wrap to 2 Pi since the sent result is unsigned
	int32_t course = (int32_t)(crs*(1e7)); 	// Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)

	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
	                        21; // bits 31-21 x position in cm
	multiplex_speed |= (((uint32_t)(gps.gspeed)) & 0x7FF) << 10;         // bits 20-10 y position in cm
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);           // bits 9-0 z position in cm

	int16_t alt = (int16_t)(gps.hmsl / 10);

	DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat,
                          &gps.lla_pos.lon, &alt);


	/*********************************************
		Relative Avoidance Behavior
	*********************************************/

	if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
		timerstart = get_sys_time_usec()/pow(10,6); 	// Elapsed time
		v_des = 0.0;
	}
	else {
		float cc[nf][6];

		// Pos in X and Y just for arena border detection!
		float posx = stateGetPositionNed_f()->x; 
		float posy = stateGetPositionNed_f()->y;

		timer  = get_sys_time_usec()/pow(10,6);
		t_elps = timer - timerstart;

		/*********************************************
			Calibration maneuver
		*********************************************/

		if ( t_elps < 5.0 && firsttime ) {
			if      ( t_elps < 1.0 ) { psi_des = calpsi            ; }
			else if ( t_elps < 2.0 ) { psi_des = calpsi+(M_PI/2)   ; }
			else if ( t_elps < 3.0 ) { psi_des = calpsi+(M_PI)     ; }
			else if ( t_elps < 4.0 ) { psi_des = calpsi+(3*M_PI/2) ; }
			else if ( t_elps < 5.0 ) { psi_des = calpsi		     ; }
			v_des = VCAL;
		}

		else {
		
			firsttime = false;				// Switch -- no longer the first time
			bool flagglobal = false; 		// Null assumption
			bool wallgettingcloser = false; 	// Null assumption

			// Wall detection algorithm
			if (sqrt(pow(posx,2) + pow(posy,2)) > magprev) {
				wallgettingcloser = true;
			}
			magprev = sqrt(pow(posx,2) + pow(posy,2));

			if ( ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5))) && wallgettingcloser) {
				//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
				cart2polar(0-posx,0-posy, &v_des, &psi_des);
				v_des = V_NOMINAL;
			}
			else {
				#ifdef FOLLOW
				if (magprev < 1.5)
				{
					cart2polar(ekf[0].X[0], ekf[0].X[1], &v_des, &psi_des);
					v_des = V_NOMINAL;
				}
				else
				{
				#endif

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

				#ifdef FOLLOW
				}
				#endif
			}

		}

		polar2cart(v_des, psi_des, &vx_des, &vy_des);
		autopilot_guided_move_ned(vx_des, vy_des, 0.0, 0.0);
	
	}

};