#include "collisioncone.h"


/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
*/
void collisioncone_update( float *cc,
	float bearing, float range, float radius,
	float obstvx, float obstvy )
{
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
	shape_shift(cc, 6, obstvx, obstvy);
	
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
