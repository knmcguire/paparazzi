#include "collisioncone.h"


int collisioncone_assess( float *cc,
	float relx, float rely,
	float ownvx, float ownvy,
	float relvx, float relvy,
	float radius)
{
	collisioncone_update(cc,	relx, rely, relvx, relvy, radius);
	return collisioncone_checkdanger(cc, ownvx, ownvy);
}

/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
*/
void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius)
{

	float range, bearing;
	cart2polar(relx, rely, &range, &bearing);

	/*
	The other two edges are symmetrical about a diagonal extending along the localization bearing.
	*/
	cc[0] = 0.0;
	cc[1] = 0.0;

	// This is the edge extending upwards to the right (+y)
	cc[2] = range+1; // x_body
	cc[3] = (range+1)*tan(atan((range+1)/radius)); // y_body

	// This is the edge exteding upwards to the left (-y)
	cc[4] = range+1; // x_body
	cc[5] = -cc[3]; // y_body

	shape_rotateatorigin(cc, 6, bearing);
	shape_shift(cc, 6, relvx, relvy);
	
	/* In MATLAB for data analysis in a global world frame:
		1. Rotate about [x1 y1] = [0 0] by orientation vector
		2. Shift to global coordinate
	*/

}

/*

	Updated the coordinates of the triangle for the collision cone of an obstacle.

	The second one is adjusted to be the fusion of its original self and the first one.

	It could speed things up a little provided that multiple cones share a similar area and that obstacles are not moving, such that the origins of the triangle are shared.

*/
void collisioncone_fuse( float *cc0, float *cc1)
{

	// 2*PI is added everywhere to avoid the issue where a boundary is in between -ang and +ang
	if ((atan2(cc1[3], cc1[2]) + 2*M_PI > atan2(cc0[3], cc0[2]) + 2*M_PI) 
		&& (atan2(cc1[3], cc1[2]) + 2*M_PI < atan2(cc0[5], cc0[4]) + 2*M_PI))
	{
		cc1[2] = cc0[2];
		cc1[3] = cc0[3];
	}

	if ((atan2(cc1[5], cc1[4]) + 2*M_PI > atan2(cc0[3], cc0[2]) + 2*M_PI)
		&& (atan2(cc1[5], cc1[4]) + 2*M_PI < atan2(cc0[5], cc0[4]) + 2*M_PI))
	{
		cc1[4] = cc0[4];
		cc1[5] = cc0[5];
	}

}


int collisioncone_checkdanger( float *cc, float ownvx, float ownvy)
{
	float vv[2];

	vv[0] = ownvx;
	vv[1] = ownvy;

	/* Check if the current velocity is ok.
	If point is in area then we get a flag */
	return shape_checkifpointinarea(cc, 6, vv);
};


int collisioncone_findnewcmd( float cc[3][6], 
	float *v_des, float *psi_des, 
	float psisearch, int nfilters )
{	

	int flag = 1;
	int count = 1;
	int ng = 1;
	int i;

	float psi_add;
	deg2rad(psisearch, &psi_add);

	float psi0 = *psi_des;
	float vx,vy;

	while ((flag == 1)	&& (count < 5))
	{

		polar2cart(*v_des, *psi_des, &vx, &vy);

		//Reciprocity is assumed!
		vx = vx/2;
		vy = vy/2;

		/* Check if we succeed */
		for (i = 0; i < nfilters; i++)
		{

			flag = collisioncone_checkdanger(cc[i], vx, vy);

			if (flag == 1)
				break;

		}

		if (flag == 0)
			break;

		*psi_des = psi0 + (ng * count * psi_add);
		wrapTo2Pi(psi_des);

		ng = ng * (-1);
		if (ng > 0)
			count++;

	}

	return count;

};
