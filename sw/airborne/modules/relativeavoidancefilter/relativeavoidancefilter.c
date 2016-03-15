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

int nfilters, ntargets, ntargets0, nstates, nmeasurements;
float arenaside;

ekf_filter ekf[2];

void rafilter_init(void)
{
	ntargets = 2;
	ntargets0 = 1;
	nfilters = 0;
	arenaside = 2;

};

void rafilter_periodic(void)
{

	struct NedCoor_f *ownPos = stateGetPositionNed_f();
	struct NedCoor_f *ownVel = stateGetSpeedNed_f();

	// randomgen_init(); // Initialize random seed

	float px = 0.0;// getrand_float(-arenaside, arenaside);
	float py = 0.0; //getrand_float(-arenaside, arenaside);

	float v_des = 1.0; // Desired total velocity	
	// float h_des = 1.0; // Desired height

	/* Initialize variables */
	float cc[3][6];
	float range, bearing;
	// float ownVx, ownVy, ownPsi,trackedVx, trackedVy, trackedPsi;
	float psi_des, x_err, y_err, h_err;

	int flag[3] = {0, 0, 0};
	int flagglobal = 0;

	int count, conflictcount;
	float psi_add;
	float psi0, ng;
	float vx, vy;	

	float vv[2];
	deg2rad(30.0,&psi_add);
	int i;

	if (nfilters < 2)
	{
		for (i = 0; i < ntargets-ntargets0; i++)
		{
		/* Initialize an ekf filter for each target tracker */
			ekf_filter_setup(&ekf[i], Q, R);
			ekf_filter_reset(&ekf[i], X, P);
			nfilters++;
		}
	}


	for (i = 0; i < ntargets-ntargets0; i++)
	{
		
		// Construct measurement vector for EKF
		Y[0] = -60.0; // Bluetooth RSSI measurement
		Y[1] = ownVel->x;
		Y[2] = ownVel->y;
		Y[3] = 0.0;
		Y[4] = 0.0;
		Y[5] = 0.0;
		Y[6] = 0.0;
		Y[7] = 0.0;

		/***********************************************/

		// TODO: Add functions for changing Q and R online
		// TODO: Add online parameter estimation

		// Run the steps of the EKF
		ekf_filter_predict(&ekf[i]);
		ekf_filter_update(&ekf[i], Y);

		// Get the outputs of the ekf into the main
		ekf_filter_get_state(&ekf[i], X, P);


		// if (extremetest(X[0], X[1], X[2], X[3], X[4], X[5], 5))
		// {
		// 	// Run HL algorithm

		// }

		cart2polar(X[0], X[1], &range, &bearing);
		collisioncone_update(cc[i], bearing, range, 1, X[4], X[5]);

		// fmat_print(9,1,X);

		vv[0] = Y[1];
		vv[1] = Y[2];

		// /* Check if the current velocity is ok */
		if (shape_checkifpointinarea(cc[i], 6, vv))
		{
			flag[i] = 1;
			flagglobal = 1;
		// conflictcount++;
		}
		else
		{
			flag[i] = 0;
		}

		if ((flag[i] == 1) && (flagglobal == 1))
		{
			// printf("yay\n");
		} 

	}

	h_err = 0.0;

	if(abs(h_err) < 0.10)
	{	

		x_err = ownPos->x - px;
		y_err = ownPos->y - py;

		vx = PID( -0.5, 0.0, 0.0, x_err, 0.0, 0.0 );
		vy = PID( -0.5, 0.0, 0.0, y_err, 0.0, 0.0 );

		cart2polar(vx, vy, &v_des, &psi_des);
		wrapTo2Pi(&psi_des);
		keepbounded(&v_des,0.0,1.0);

		psi0 = psi_des;
		
		polar2cart(v_des, psi_des, &vx, &vy);

		count = 0;
		ng = 1.0;

		// vv[0] =   vx*cos( ownPsi ) - vy*sin( ownPsi );
		// vv[1] = - vx*sin( ownPsi ) - vy*cos( ownPsi );

		if (flagglobal == 0)
			conflictcount = 0;

		while ((flagglobal == 1) && (count < 6))
		{

			/* Check if we succeed */
			for (i = 0; i < ntargets-ntargets0; i++)
			{
				if (shape_checkifpointinarea(cc[i], 6, vv) == 1)
				{
					flagglobal = 1;
					break;
				}
				else
				{
					flagglobal = 0;
				}
			}

			if (flagglobal == 0)
				break;

			psi_des = psi0 + ng * count * psi_add;
			wrapTo2Pi(&psi_des);

			if ((count > 5) || conflictcount > 5)
			{
				px = 0.0; //getrand_float(-arenaside, arenaside);
				py = 0.0; //getrand_float(-arenaside, arenaside);
			}
			else
			{
				v_des = 1.0;
			}

			polar2cart(v_des, psi_des, &vx, &vy);
			
			//Reciprocality is assumed!
			vx = vx/2.0;
			vy = vy/2.0;

			vv[0] = vx; // vx*cos( ownPsi ) - vy*sin( ownPsi );
			vv[1] = vy; //- vx*sin( ownPsi ) - vy*cos( ownPsi );

			ng = ng * (-1);
			
			if (ng > 0)
				{count ++;}

		}


		if ( (abs(ownPos->x) > arenaside)
			|| (abs(ownPos->y) > arenaside) )
		{

			px = 0.0; //getrand_float(-arenaside, arenaside);
			py = 0.0; //getrand_float(-arenaside, arenaside);

			x_err = ownPos->x - px;
			y_err = ownPos->y - py;

			vx = PID( -0.25, 0.0, 0.0, x_err, 0.0, 0.0 );
			vy = PID( -0.25, 0.0, 0.0, y_err, 0.0, 0.0 );

			keepbounded(&vx, -1.0, 1.0);
			keepbounded(&vy, -1.0, 1.0);

			// cout << name << ": " << vx << " " << vy << endl;
		}

		// cout << name << " " << psi_des << endl;
		// cmddata.linear.x = vx;
		// cmddata.linear.y = vy;

	}


	if ((abs(px - ownPos->x) < 0.1)
		&& (abs(py - ownPos->y) < 0.1))
	{
		px = 0.0; //getrand_float(-arenaside, arenaside);
		py = 0.0; //getrand_float(-arenaside, arenaside);
	}

	

};

