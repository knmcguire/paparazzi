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
#include "subsystems/datalink/telemetry.h"
#include "modules/multi/traffic_info.h"
#include "modules/stdma/stdma.h"

#define CRSSEARCH 15.0 		// Search grid for crs_des
#define NUAVS 5				// Maximum expected number of drones
#define MAF_SIZE_POS 1  	// Moving Average Filter size; 1 = No filter
#define MAF_SIZE_VEL 1  	// Moving Average Filter size; 1 = No filter

#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif

ekf_filter ekf[NUAVS-1]; 	// EKF structure
btmodel model[NUAVS-1];  	// Bluetooth model structure 
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
int8_t srcstrength[NUAVS-1];// Source strength
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV

int nf; 					// Number of filters registered
float crs_des, v_des; 		// crs_des = desired course w.r.t. north, v_des = magnitude of velocity
float vx_des, vy_des;		// Desired velocities in NED frame
float vx_des_b, vy_des_b;		// Desired velocities in NED frame
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
float magprev;				// Previous magnitude from 0,0 (for simulated wall detection)
float pxother, pyother;

float ccvec[NUAVS-1][4];
float x_est[NUAVS-1][MAF_SIZE_POS], y_est[NUAVS-1][MAF_SIZE_POS];
float vx_est[NUAVS-1][MAF_SIZE_VEL], vy_est[NUAVS-1][MAF_SIZE_VEL];

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);
/*
static abi_event vel_est_ev;
static void vel_est_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise);
*/

struct NedCoor_i gps_pos_cm_ned_i;
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ned_of_ecef_point_i(&gps_pos_cm_ned_i, &ins_int.ltp_def, &gps_s->ecef_pos);
}

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{ 
	int i= -1; // Initialize index (null assumption, no drone is present)
	bool firsttime;
	// If it's a new ID we start a new EKF for it
	// TODO: aircraft ID are now hard coded here, make it more general (set in airframe file?)
	// The hardcoding was done to make sure that no other bluetooth devices or drone accidentally interfere with testing
	if ((!array_find_int(NUAVS-1, IDarray, ac_id, &i)) && (nf < NUAVS-1) )
	{ // Check if a new aircraft ID is present, continue
		// firsttime = false;
		IDarray[nf] = ac_id; 				// Store ID
		srcstrength[nf] = source_strength;  // Store source strength
		ekf_filter_new(&ekf[nf]); 			// Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(EKF_N,EKF_N, ekf[nf].Q, pow(0.5,2.0), ekf[nf].Q);
		fmat_scal_mult(EKF_M,EKF_M, ekf[nf].R, pow(SPEEDNOISE,2.0), ekf[nf].R);
		ekf[nf].Q[0]   	   = 0.01; // Reccomended 0.01 to give this process a high level of trust
		ekf[nf].Q[EKF_N+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
		ekf[nf].X[0]   = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1]   = 1.0;

		// If not sending the orientation but only NED velocity, then orientation noises may be set to 0
		// TODO: Maybe remove this idea altogether and just make it based on ned only all the time?
		ekf[nf].dt       = 0.2; 							      // Initial assumption (STDMA code runs at 5Hz)
		// model[nf].Pn     = -63.0 - 8.0 + (float)source_strength;  // -63 is from calibration with antenna that had 8dB power, so the source_strength recalibrates
		model[nf].Pn     = -55;
		model[nf].gammal = 2.0;	 // Assuming free space loss
		nf++; // Number of filter is present is increased!
	}

	// If we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) ) {
		RSSIarray[i] = (float)rssi; //logging
		float ownVx = stateGetSpeedEnu_f()->y; //vel_body.x; //From optical flow directly
		float ownVy = stateGetSpeedEnu_f()->x; //vel_body.y;
		
		// Bind velocities to a maximum to avoid occasional NaN or inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) // only in guided mode (flight) (take off in NAV)
		{
			// firsttime == true;
			// Update the time between messages
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6);

			float trackedVx, trackedVy;
			// Get the aircraft info for that ID
			polar2cart(acInfoGetGspeed(ac_id), acInfoGetCourse(ac_id), &trackedVx, &trackedVy); // get x and y velocities (m/s)
			pxother = acInfoGetPositionEnu_f(ac_id)->x;
			pyother = acInfoGetPositionEnu_f(ac_id)->y;
			
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);

			// Construct measurement vector for EKF using the latest data obtained for each case
			float Y[EKF_M];
			Y[0] = (float)rssi;
			Y[1] = ownVx; 	   // Own velocity already in Earth NED frame
			Y[2] = ownVy;
			Y[3] = trackedVx;  // Velocity tracked from other drone (already in Earth NED frame!)
			Y[4] = trackedVy;
			Y[5] = acInfoGetPositionUtm_f(ac_id)->alt - stateGetPositionEnu_f()->z;
			
			// Run the steps of the EKF
			ekf_filter_predict(&ekf[i], &model[i]);
			ekf_filter_update(&ekf[i], Y);
				
			/*
			 * Xvector: dotx_other and doty_other are expressed in own bodyframe
			 * X = [x y dotx_own doty_own dotx_other doty_other psi_own psi_other dh]
			 * Yvector: dotx_other and dotyother are not rotated yet
			 * Y = [RSSI dotx_own doty_own psi_own dotx_other doty_other psi_other dh]
			 */
			// Moving average filter to state (MAF_SIZE of 1, average with direct last measurment)
			ccvec[i][0] = movingaveragefilter(x_est[i],  MAF_SIZE_POS, ekf[i].X[0]);
			ccvec[i][1] = movingaveragefilter(y_est[i],  MAF_SIZE_POS, ekf[i].X[1]);
			ccvec[i][2] = movingaveragefilter(vx_est[i], MAF_SIZE_VEL, ekf[i].X[4]);
			ccvec[i][3] = movingaveragefilter(vy_est[i], MAF_SIZE_VEL, ekf[i].X[5]);
		}
		// else if (firsttime == false) { // Initial estimate is towards the initial direction of flight (filter of other drone!)
		else
		{
			ekf[i].X[0] = 1.0; // Initial positions cannot be zero or else you'll divide by zero
			ekf[i].X[1] = 1.0;
			ekf[i].X[2] = 0.0;
			ekf[i].X[3] = 0.0;
			ekf[i].X[4] = 0.0;// -stateGetPositionNed_f()->x; // Initial positions cannot be zero or else you'll divide by zero
			ekf[i].X[5] = 0.0;// -stateGetPositionNed_f()->y;
			ekf[i].X[6] = 0.0;// -stateGetPositionNed_f()->x; // Initial positions cannot be zero or else you'll divide by zero					
		}
		// Update latest time
		now_ts[i] = get_sys_time_usec();

	}
};

bool alternate;
static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;

	// TODO: MAKE THIS SWITCHING NOT LAZY BUT PROPER FOR UNLIMITED MAVS
	// array_shiftleft(vec, nf, 1);
	// vec is a vector of 0 to nf defined each time nf increases
	if (nf == 2) {
		if (alternate) {
			alternate = false;
			i = 1;
		}
		else {
			alternate = true;
			i = 0;
		}
	}
	else { //only data on first
		i = 0;
	}

	float px, py, pz;
	px = (float)gps_pos_cm_ned_i.x/100.0;
	py = (float)gps_pos_cm_ned_i.y/100.0;
	pz = (float)gps_pos_cm_ned_i.z/100.0;

	uint8_t id = (uint8_t)IDarray[i];

	pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
		&id,			     // ID or filtered aircraft number
		&RSSIarray[i], 		    	 // Received ID and RSSI
		&srcstrength[i],		     // Source strength
		&px, &py, &pz, 				 // &gps_pos_cm_ned_i.z    //
		&ekf[i].X[0], &ekf[i].X[1],  // x and y pos
		&ekf[i].X[2], &ekf[i].X[3],  // Own vx and vy
		&ekf[i].X[4], &ekf[i].X[5],  // Received vx and vy
		&ekf[i].X[6],  				 // Height separation
		&vx_des, &vy_des);		     // Commanded velocities

	float temp = 0;
	pprz_msg_send_GPS_ERROR(trans, dev, AC_ID,
		&pxother,&pyother,&temp,&temp,&temp,&temp);
};

void relativeavoidancefilter_init(void)
{
	randomgen_init();    	// Initialize the random generator (for simulation purposes)
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	
	alternate = 0;			// Logging variable
	nf 		  = 0; 		   	// Number of active filters
	crs_des   = 0.0;	   	// Initialize
	v_des     = V_NOMINAL; 	// Initial desired velocity
	magprev   = 3.0; 	   	// Just a random high value

	for (int i = 0; i < NUAVS-1; i++) {
		fmat_make_zeroes( x_est[i],  1, MAF_SIZE_POS );
		fmat_make_zeroes( y_est[i],  1, MAF_SIZE_POS );
		fmat_make_zeroes( vx_est[i], 1, MAF_SIZE_POS );
		fmat_make_zeroes( vy_est[i], 1, MAF_SIZE_POS );
	}

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(ABI_BROADCAST, &rssi_ev, bluetoothmsg_cb);
	AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
	// AbiBindMsgVELOCITY_ESTIMATE(ABI_BROADCAST, &vel_est_ev, vel_est_cb);
	
	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_GPS_ERROR, send_rafilterdata);
};


void relativeavoidancefilter_periodic(void)
{	
	// FAKE MESSAGE for testing purposes from the 0.0 position if a BT source is not available!
	// AbiSendMsgRSSI(1, 2, 2, 30);

	/*********************************************
		Sending speed directly between drones
	*********************************************/
	// Convert Course to the proper format (NED)
	float spd, crs;
	cart2polar(-stateGetSpeedEnu_f()->y, -stateGetSpeedEnu_f()->x, &spd, &crs); // Get the total speed and course
	wrapTo2Pi(&crs); 					    								  // Wrap to 2 Pi since the sent result is unsigned

	int32_t course = (int32_t)(crs*(1e7)); // Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)
	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
	                        21; 									  // bits 31-21 course (where the magnitude is pointed at)
	multiplex_speed |= (((uint32_t)(spd*100)) & 0x7FF) << 10;         // bits 20-10 speed in cm/s
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);        // bits 9-0 z velocity in cm/s

	// int16_t alt = (int16_t)(gps.hmsl / 10); 					  // height in cm
	int16_t alt = (int16_t)(stateGetPositionEnu_f()->z*100.0);

    // Message through USB bluetooth dongle to other drones
    // Pocketdrones (STDMA Running within Paparazzi)
	DOWNLINK_SEND_GPS_SMALL(stdma_trans, bluegiga_p, &multiplex_speed, &gps.lla_pos.lat, &gps.lla_pos.lon, &alt);

	/*********************************************
		Relative Avoidance Behavior
	*********************************************/
	if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {

		//switch height to RC controlled height (for pocket drone without IR sensor!)
	    // guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);

		float cc[nf][6];

		// Pos in X and Y just for arena border detection!
		// float posx = stateGetPositionEnu_f()->y;
		// float posy = stateGetPositionEnu_f()->x;
		float posx = (float)gps_pos_cm_ned_i.x/100.0;
		float posy = (float)gps_pos_cm_ned_i.y/100.0;

		bool collision_imminent = false; // Null assumption
		bool wall_imminent 		= false; // Null assumption

		// Wall detection
		if (sqrt(pow(posx,2) + pow(posy,2)) > magprev) {
			wall_imminent = true;
		}
		magprev = sqrt(pow(posx,2) + pow(posy,2));

		// If approaching wall, then change direction to avoid wall
		if ( ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5))) && wall_imminent ) {
		// if ( ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5)))) {
			//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
			cart2polar(-posx,-posy, &v_des, &crs_des);
			v_des = V_NOMINAL;
		}
		// Otherwise worry about other drones
		else {
			polar2cart(v_des, crs_des, &vx_des, &vy_des); // vx_des & vy_des = desired velocity
			uint8_t i;
			for ( i = 0; i < nf; i++ ) { // nf is amount of filters running
				float dist = sqrt(pow(ekf[i].X[0],2) + pow(ekf[i].X[1],2));
				float eps = 1.0*ASIDE*tan(1.7/2) - MAVSIZE - ASIDE;
				
				// cc[i] = collisioncone with respect to other robot i
				collisioncone_update(cc[i], ccvec[i][0], ccvec[i][1], ccvec[i][2], ccvec[i][3], dist+MAVSIZE+eps);	// x y xdot_0 ydot_0 (characteristics collision cones)
				
				if ( collisioncone_checkdanger( cc[i], vx_des, vy_des )) {
					collision_imminent = true; 		// We could be colliding!
				}
			}
	
			if (collision_imminent) { // If the desired velocity doesn't work, then let's find the next best thing according to VO
				v_des = V_NOMINAL;
				collisioncone_findnewcmd(cc, &v_des, &crs_des, CRSSEARCH, nf); // Go clockwise until save direction in found
			}
		}

		polar2cart(v_des, crs_des, &vx_des, &vy_des);  		// new desired speed in North (x) and East(y)
		
		/* Optitrack Guided commands */
		// autopilot_guided_move_ned(vx_des, vy_des, 0.0, 0.0);  	//send to guided mode -- use this if flying with optitrack
		
		/* Opticflow Guided commands */
		// vx_des_b = vx_des*cos(-stateGetNedToBodyEulers_f()->psi) - vy_des*sin(-stateGetNedToBodyEulers_f()->psi);
		// vy_des_b = vx_des*sin(-stateGetNedToBodyEulers_f()->psi) + vy_des*cos(-stateGetNedToBodyEulers_f()->psi);
		// guidance_h_set_guided_body_vel(vx_des, vy_des);

		guidance_h_set_guided_vel(vx_des,vy_des);
		
		guidance_v_set_guided_z(-1.5);
		// guidance_h_set_guided_heading(0.0); % not reccommended if without a good heading estimate

	}

};