#include "PID.h"

/* 
Function for a basic Proportional Integral Differential (PID) controller 
*/
float PID(float Kp, float Ki, float Kd, float err, float errprev, float dt)
{
	float errdev;
	float errint;

	if (Ki != 0.0)
		{errint = (err - errprev) * dt;}
	else
		{errint = 0.0;}

	if (Kd != 0.0)
		{errdev = Ki * ((err - errprev) / dt);}
	else
		{errdev = 0.0;}

	return Kp * err + Ki * errint + Kd * errdev;
};
