#include "coordinateconversions.h"

/* Function to convert polar coordinates to cartesian

Compliments of 
http://stackoverflow.com/questions/29089710/pointers-in-c-programming-coordinate-conversion */
void polar2cart(float radius, float radians, float *x, float *y)
{
	*x = radius * cos(radians); 
	*y = radius * sin(radians);
};

/* Function to convert cartesian coordinates to polar */
void cart2polar(float x, float y, float *radius, float *radians)
{
	*radius  = sqrt(pow(x,2) + pow(y,2));
	*radians = atan2(y,x);
};

/* Function to convert radians to degrees */
void rad2deg(float rad, float *deg)
{

	*deg = 180.0 / M_PI * rad;
}

/* Function to convert degrees to radians */
void deg2rad(float deg, float *rad)
{
	*rad = M_PI / 180.0 * deg;
}

/* Wraps an angle in radians between -PI and +PI */
void wrapToPi(float *ang)
{
	if (*ang > M_PI)
	{		
		*ang = *ang - 2*M_PI;
	}
	else if (*ang < -M_PI)
	{
		*ang = *ang + 2*M_PI;
	}
}


/* Wraps an angle in radians between 0 and 2PI */
void wrapTo2Pi(float *ang)
{
	if (*ang > 2*M_PI)
	{		
		*ang = *ang - 2*M_PI;
	}
	else if (*ang < 0.0)
	{
		*ang = *ang + 2*M_PI;
	}
}

/* Keeps a value between two bounds */
void keepbounded(float *value, float min, float max)
{
	if (*value < min)
	{		
		*value = min;
	}
	else if (*value > max)
	{
		*value = max;
	}
}
