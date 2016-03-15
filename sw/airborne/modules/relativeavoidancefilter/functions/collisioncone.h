#ifndef COLLISIONCONE_H
#define COLLISIONCONE_H

#include "math.h"
#include "shape.h"
#include "coordinateconversions.h"


int collisioncone_assess( float *cc,
	float relx, float rely,
	float ownvx, float ownvy,
	float relvx, float relvy,
	float radius);

void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius);

void collisioncone_fuse( float *cc0, float *cc1 );

int collisioncone_checkdanger( float *cc, 
	float ownvx, float ownvy);

int collisioncone_findnewcmd(	float cc[][6], 
	float *v_des, float *psi_des,
	float psisearch, int nfilters );

#endif