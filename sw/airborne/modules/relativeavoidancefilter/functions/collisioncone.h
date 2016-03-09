#ifndef COLLISIONCONE_H
#define COLLISIONCONE_H

#include "math.h"
#include "shape.h"

#define PI 3.141592653589793

void collisioncone_update( float *cc,
	float bearing, float range, float radius,
	float relvx, float relvy );

void collisioncone_fuse( float *cc0, float *cc1 );

#endif