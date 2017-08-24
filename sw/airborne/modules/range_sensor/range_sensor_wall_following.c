/*
 * range_sensor_wall_following.c
 *
 *  Created on: Aug 24, 2017
 *      Author: knmcguire
 */


#include "modules/range_sensor/range_sensor_forcefield.h"

#include "subsystems/abi.h"


float heading_WF;

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


  AbiBindMsgRANGE_SENSORS(RANGE_MODULE_RECIEVE_ID, &range_sensors_ev, range_sensors_cb);
}

void range_sensor_wall_following_run(void)
{
	  //calculate heading
	  heading_WF = 0;
	  range_sensor_wall_following_heading_calculate(&heading_WF);
	  AbiSendMsgRANGE_WALLFOLLOWING(RANGE_WALLFOLLOWING_ID, heading_WF);

	  if(heading_WF != 0)
	  printf("heading_WF: %f\n", heading_WF);
	  range_finders_prev = range_finders;
}


void range_sensor_wall_following_heading_calculate(float *new_heading)
{
    //right range sensor
	float range_diff_right = range_finders.right - range_finders_prev.right;


	*new_heading +=  20*range_diff_right;

	if(*new_heading > 1)
		*new_heading = 1;
	if(*new_heading < -1)
		*new_heading = -1;
/*	if(range_diff_right<0){
		*new_heading -= 0.1;
	}else if(range_diff_right>0){
		*new_heading += 0.1;
	}else{
		// Do not do anything
	}*/

}

