#ifndef PID_H
#define PID_H

#include "randomgenerator.h"

/* 
Function for a basic Proportional Integral Differential (PID) controller 
*/
extern float PID(float Kp, float Ki, float Kd, float err, float errprev, float dt);
void getNewGoalPos(float *px, float *py, float arenasize);

#endif
