/*
 * range_sensor_wall_following.c
 *
 *  Created on: Aug 24, 2017
 *      Author: knmcguire
 */


#include "modules/range_sensor/range_sensor_forcefield.h"

#include "subsystems/abi.h"


float heading_WF;
float vel_body_y_WF;
bool range_sensor_detect_WF;

//abi for range sensors
#ifndef RANGE_MODULE_RECIEVE_ID
#define RANGE_MODULE_RECIEVE_ID ABI_BROADCAST
#endif

static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top);
struct range_finders_ range_finders;
struct range_finders_ range_finders_prev;

static void range_sensors_cb(uint8_t UNUSED(sender_id),
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left, int16_t range_bottom, int16_t range_top)
{


  range_finders.front = (float)range_front / 1000;
  range_finders.right = (float)range_right / 1000;
  range_finders.back = (float)range_back / 1000;
  range_finders.left = (float)range_left / 1000;
  range_finders.top = (float)range_top / 1000;
  range_finders.bottom = (float)range_bottom / 1000;


}


void range_sensor_wall_following_init(void)
{
  heading_WF =0;
  vel_body_y_WF =0;
  range_sensor_detect_WF=FALSE;

  AbiBindMsgRANGE_SENSORS(RANGE_MODULE_RECIEVE_ID, &range_sensors_ev, range_sensors_cb);
}

void range_sensor_wall_following_run(void)
{
	//calculate heading
	heading_WF = 0;
	vel_body_y_WF=0;
	range_sensor_detect_WF=FALSE;

	range_sensor_wall_following_heading_calculate(&heading_WF,&vel_body_y_WF,&range_sensor_detect_WF);

	printf("heading and wall distance %f %f  %f %d\n", heading_WF, vel_body_y_WF,range_finders.front, range_sensor_detect_WF);
	AbiSendMsgRANGE_WALLFOLLOWING(RANGE_WALLFOLLOWING_ID, heading_WF,vel_body_y_WF,range_sensor_detect_WF);

	AbiSendMsgOBSTACLE_DETECTION(RANGE_OBSTACLE_DETECT_ID, range_finders.front, 0);


	if(heading_WF != 0)
		range_finders_prev = range_finders;
}


void range_sensor_wall_following_heading_calculate(float *new_heading, float *new_vel_y_body, bool *new_range_sensor_detect)
{

    //right range sensor
	if(range_finders.right<2&&range_finders_prev.right<2){
	float range_diff_right = range_finders.right - range_finders_prev.right;
	*new_range_sensor_detect = TRUE;

	*new_heading +=  range_diff_right;
	*new_vel_y_body = (range_finders.right - 1);

	if(*new_heading > 1)
		*new_heading = 1;
	if(*new_heading < -1)
		*new_heading = -1;

     }else { *new_heading = 0;*new_vel_y_body=0;
     new_range_sensor_detect=FALSE;}
/*	if(range_diff_right<0){
		*new_heading -= 0.1;
	}else if(range_diff_right>0){
		*new_heading += 0.1;
	}else{
		// Do not do anything
	}*/

}

