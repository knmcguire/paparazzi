/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.c"
 * @author Kimberly McGuire
 * This module sends the data retreived from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#include "modules/stereocam/stereocam2state/stereocam2state.h"

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "filters/median_filter.h"
#include "state.h"

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_orientation_conversion.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#ifndef STEREOCAM2STATE_RECEIVED_DATA_TYPE
#define STEREOCAM2STATE_RECEIVED_DATA_TYPE 0
#endif

#ifndef STEREOCAM2STATE_CAM_FORWARD
#define STEREOCAM2STATE_CAM_FORWARD 1
#endif

static abi_event gps_ev;
struct NedCoor_f opti_vel;
struct NedCoor_i opti_vel_int;
struct FloatVect3 velocity_rot_gps;
struct Int32Vect3 velocity_rot_gps_int;



#include "subsystems/datalink/telemetry.h"

struct MedianFilter3Int filter_1;
struct MedianFilter3Int filter_2;

void stereocam_to_state(void);
/*
#if PERIODIC_TELEMETRY

static void stereocam_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STEREOCAM_OPTIC_FLOW(trans, dev, AC_ID,
      &vel_body_x_int, &vel_body_y_int, &velocity_rot_gps_int.x,&velocity_rot_gps_int.y); // TODO: no noise measurement here...
}
#endif
*/

static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  opti_vel.x = (float)(gps_s->ecef_vel.x) / 100;
  opti_vel.y = (float)(gps_s->ecef_vel.y) / 100;
  opti_vel.z = (float)(gps_s->ecef_vel.z) / 100;
  opti_vel_int.x = gps_s->ecef_vel.x;
  opti_vel_int.y = gps_s->ecef_vel.y;
  opti_vel_int.z = gps_s->ecef_vel.z;
}

void stereo_to_state_init(void)
{
  InitMedianFilterVect3Int(filter_1);
  InitMedianFilterVect3Int(filter_2);

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);

  /*
  #if PERIODIC_TELEMETRY
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STEREOCAM_OPTIC_FLOW, stereocam_telem_send);
  #endif

  */

}

void stereo_to_state_periodic(void)
{
  if (stereocam_data.fresh) {
    stereocam_to_state();
    stereocam_data.fresh = 0;
  }
}

void stereocam_to_state(void)
{

  // Get info from stereocam data
  // 0 = stereoboard's #define SEND_EDGEFLOW
#if STEREOCAM2STATE_RECEIVED_DATA_TYPE == 0
  // opticflow
  int16_t div_x = (int16_t)stereocam_data.data[0] << 8;
  div_x |= (int16_t)stereocam_data.data[1];
  int16_t flow_x = (int16_t)stereocam_data.data[2] << 8;
  flow_x |= (int16_t)stereocam_data.data[3];
  int16_t div_y = (int16_t)stereocam_data.data[4] << 8;
  div_y |= (int16_t)stereocam_data.data[5];
  int16_t flow_y = (int16_t)stereocam_data.data[6] << 8;
  flow_y |= (int16_t)stereocam_data.data[7];

  float fps = (float)stereocam_data.data[9];
  int8_t agl = stereocam_data.data[8]; // in cm

  // velocity
  /*int16_t vel_x_int = (int16_t)stereocam_data.data[10] << 8;
  vel_x_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_y_int = (int16_t)stereocam_data.data[12] << 8;
  vel_y_int |= (int16_t)stereocam_data.data[13];*/


  int16_t vel_x_global_int = (int16_t)stereocam_data.data[10] << 8;
  vel_x_global_int |= (int16_t)stereocam_data.data[11];
  int16_t vel_y_global_int = (int16_t)stereocam_data.data[12] << 8;
  vel_y_global_int |= (int16_t)stereocam_data.data[13];
  int16_t vel_z_global_int = (int16_t)stereocam_data.data[14] << 8;
  vel_z_global_int |= (int16_t)stereocam_data.data[15];

  int16_t vel_x_pixelwise_int = (int16_t)stereocam_data.data[16] << 8;
  vel_x_pixelwise_int |= (int16_t)stereocam_data.data[17];
  int16_t vel_z_pixelwise_int = (int16_t)stereocam_data.data[18] << 8;
  vel_z_pixelwise_int |= (int16_t)stereocam_data.data[19];

  /* int16_t vel_x_stereo_avoid_pixelwise_int = (int16_t)stereocam_data.data[20] << 8;
   vel_x_stereo_avoid_pixelwise_int |= (int16_t)stereocam_data.data[21];
    int16_t vel_z_stereo_avoid_pixelwise_int = (int16_t)stereocam_data.data[22] << 8;
    vel_z_stereo_avoid_pixelwise_int |= (int16_t)stereocam_data.data[23];*/
  uint8_t edgeflow_avoid_mode = stereocam_data.data[20];

  int16_t vel_x_stereo_avoid_pixelwise_int = 0;
  int16_t vel_z_stereo_avoid_pixelwise_int = 0;

  int16_t RES = 100;

  struct Int16Vect3 vel, vel_global;
  vel.x = vel_x_pixelwise_int;
  vel.y = vel_z_pixelwise_int;

  vel_global.x = vel_x_global_int;
  vel_global.y = vel_y_global_int;
  vel_global.z = vel_z_global_int;

  UpdateMedianFilterVect3Int(filter_1, vel);
  UpdateMedianFilterVect3Int(filter_2, vel_global);

  //float vel_x_global_f = (float)vel_x_global_int / RES;
  //float vel_y_global_f = (float)vel_y_global_int / RES;
  float vel_x_global_f = (float)vel_global.x / RES;
  float vel_y_global_f = (float)vel_global.y / RES;

  // Derotate velocity and transform from frame to body coordinates
  // TODO: send resolution directly from stereocam
  int16_t vel_body_x_int;
  int16_t vel_body_y_int;
  int16_t vel_body_x_global_int;
  int16_t vel_body_y_global_int;
  int16_t vel_body_z_global_int;
  int16_t vel_x_stereo_avoid_body_pixelwise_int;
  int16_t vel_y_stereo_avoid_body_pixelwise_int;

#if STEREOCAM2STATE_CAM_FORWARD ==1
  float vel_x = (float)vel.x / RES;
  float vel_y = (float)vel.y / RES;

  float vel_body_x =  - vel_y;
  float vel_body_y =  vel_x;
  vel_body_x_int = - vel.y;
  vel_body_y_int =  vel.x;
  vel_body_x_global_int = - vel_global.z;
  vel_body_y_global_int =  vel_global.x;
  vel_body_z_global_int =  - vel_global.y;
  vel_x_stereo_avoid_body_pixelwise_int = - vel_z_stereo_avoid_pixelwise_int;
  vel_y_stereo_avoid_body_pixelwise_int =  vel_x_stereo_avoid_pixelwise_int;

#else
  float vel_x = (float)vel_x_global_int / RES;
  float vel_y = (float)vel_y_global_int / RES;
  float vel_body_x = - vel_x;
  float vel_body_y = vel_y;
#endif

  struct FloatVect3 velocity_rot_gps;
  struct Int32Vect3 velocity_rot_gps_int;
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_vel);
  int32_rmat_vmult(&velocity_rot_gps_int , stateGetNedToBodyRMat_i(), (struct Int32Vect3 *)&opti_vel_int);


  int16_t vel_x_opti_int = - (int16_t)(velocity_rot_gps.y * 100);
  int16_t vel_y_opti_int = -(int16_t)(velocity_rot_gps.x * 100);
  int16_t vel_z_opti_int = -(int16_t)(velocity_rot_gps.z * 100);


  float avoid_turn = 30.0f * 3.14f / 180.0f;
  float avoid_turn_rate = 20.0f * 3.14f / 180.0f;
  float forward_speed = 0.5f;
  static bool drone_is_turning = false;
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  static float prev_heading = 0;
  //printf("current_heading %f\n",current_heading);
  uint8_t turn_delay = 50;
  static bool flip_switch = true;
  static uint8_t turn_counter = 0;

  bool drone_has_to_turn = false;


  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {

	  if(flip_switch==true)
	  {
		  guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
		  turn_counter = 0;
		  flip_switch = false;
		  drone_is_turning = false;
		  prev_heading = 0;
		  guidance_h_set_guided_body_vel(0.0f, 0.0f);
		   guidance_h_set_guided_heading_rate(0.0f);

	  }

	  if(edgeflow_avoid_mode == 4)
	        guidance_h_set_guided_body_vel(0.3f, 0.0f);
	  else
	  {
          guidance_h_set_guided_body_vel(-0.1f, 0.0f);
          if (drone_is_turning == false)
          {
          guidance_h_set_guided_heading(current_heading - avoid_turn );
          drone_is_turning = true;
          turn_counter =0;
          }else{
        	  turn_counter ++;
          }
	  }

	  if (turn_counter==30)
	  {
		  drone_is_turning = false;
	  }

  //    guidance_h_set_guided_heading_rate(-1*avoid_turn_rate);


   /* switch (edgeflow_avoid_mode) {
      case 4:
        // fly forward with constant speed
        guidance_h_set_guided_body_vel(0.3f, 0.0f);
  //      printf("go forward\n");
       // drone_has_to_turn = false;
        break;
      case 11:
      case 12:
      case 21:
      case 22:
          guidance_h_set_guided_body_vel(-0.2f, 0.0f);
          break;

      case 11:
          guidance_h_set_guided_body_vel(0.0f, -0.1f);
          guidance_h_set_guided_heading_rate(-1*avoid_turn_rate);
     //     printf("go left\n");

          break;
      case 21:
          guidance_h_set_guided_body_vel(0.0f, -0.1f);
          guidance_h_set_guided_heading_rate(-1*avoid_turn_rate);
      //    printf("go left\n");

    	  break;
      case 12:
          guidance_h_set_guided_body_vel(0.0f, -0.1f);
          guidance_h_set_guided_heading_rate(-1*avoid_turn_rate);
       //   printf("go right\n");

          break;
      case 22:
          guidance_h_set_guided_body_vel(0.0f, -0.1f);
          guidance_h_set_guided_heading_rate(-1*avoid_turn_rate);
        //  printf("go right\n");

    	  break;
      case 11:
          guidance_h_set_guided_body_vel(-0.1f, 0.0f);
        // avoid_turn = -1*avoid_turn;
        avoid_turn = -1*avoid_turn_rate;
        drone_has_to_turn = true;
        break;
      case 12:
        //fly forward with constant speed and move slightly to right
         //avoid_turn = avoid_turn;
         //avoid_turn = avoid_turn_rate;;
          guidance_h_set_guided_body_vel(-0.1f, 0.0f);

        drone_has_to_turn = true;
        break;
      case 21:
        //Stop!! and move to the left until mode 4
        guidance_h_set_guided_body_vel(-0.1f, 0.0f); //a bit of backwards movement to counteract drift
       // avoid_turn = -1*avoid_turn;
       avoid_turn = -1*avoid_turn_rate;
       drone_has_to_turn = true;
       break;
      case 22:
        //stop!! and move to the right until mode 4
        guidance_h_set_guided_body_vel(-0.1f, 0.0f); //a bit of backwards movement to counteract drift
        drone_has_to_turn = true;
        break;
      default:
        //stop!
       //  drone_has_to_turn = false;
        guidance_h_set_guided_body_vel(0.0, 0.0);
        guidance_h_set_guided_heading(0.0);
    }
*/



/*

	   if(drone_is_turning == false&& drone_has_to_turn == true) {
		   guidance_h_set_guided_heading_rate(avoid_turn_rate);
		   drone_is_turning = true;
		   prev_heading = current_heading;
		   //printf("start turning\n");
	   }
  // printf("turn counter: %d\n", turn_counter);
	   if(drone_is_turning==true && fabs( prev_heading - current_heading ) > fabs(avoid_turn))
	   {
		//   printf("stop turning %f\n",fabs( prev_heading - current_heading ));

		   //printf("check %d\n", turn_counter);
		   guidance_h_set_guided_heading_rate(0.0f);

	   }

	   if( drone_is_turning == true && turn_counter < turn_delay)
		   turn_counter ++;
	   else {
		   guidance_h_set_guided_heading_rate(0.0f);

		   turn_counter = 0;
		   drone_is_turning = false;
	   }
*/



  }
  else
  {
	  flip_switch = true;
  }


  //  vel_body_x = vel_body_x + (float)vel_x_stereo_avoid_body_pixelwise_int / RES;
  // vel_body_y = vel_body_y + (float)vel_y_stereo_avoid_body_pixelwise_int / RES;

  //Send velocity estimate to state
  //TODO:: Make variance dependable on line fit error, after new horizontal filter is made
  uint32_t now_ts = get_sys_time_usec();

  if (!(abs(vel_body_x) > 0.5 || abs(vel_body_x) > 0.5)) {
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                vel_body_x,
                                vel_body_y,
                                0.0f,
                                0.3f
                               );
  }

  // Reusing the OPTIC_FLOW_EST telemetry messages, with some values replaced by 0

  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;
  static int16_t counter = 0;



  //DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &dummy_int16, &dummy_int16,
  //    &vel_x, &vel_y,&dummy_float, &dummy_float, &dummy_float);

  /*DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &dummy_int16, &dummy_int16,
            &velocity_rot_gps.x, &velocity_rot_gps.y, &dummy_float, &dummy_float, &dummy_float);*/

// DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int, &vel_body_x_global_int, &vel_body_y_global_int, &vel_body_z_global_int, &vel_x_opti_int, &vel_y_opti_int, &vel_z_opti_int);
  if (counter == 5) {
    DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int,
                                       &vel_body_x_global_int, &vel_body_y_global_int,
                                       &vel_body_z_global_int, &vel_x_opti_int,  &vel_y_opti_int, &vel_z_opti_int, &vel_x_stereo_avoid_body_pixelwise_int,
                                       &vel_y_stereo_avoid_body_pixelwise_int,&edgeflow_avoid_mode);
    //DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int, &vel_body_x_global_int, &vel_body_y_global_int,
    //   &vel_body_z_global_int, &vel_x_opti_int,  &vel_y_opti_int, &vel_z_opti_int, &vel_x_stereo_avoid_body_pixelwise_int, &vel_y_stereo_avoid_body_pixelwise_int);

    // DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int, &vel_body_x_global_int, &vel_body_y_global_int, &vel_body_z_global_int, &vel_x_opti_int,  &vel_y_opti_int, &vel_z_opti_int);

    //DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int, (int16_t *)&velocity_rot_gps_int.x, (int16_t *)&velocity_rot_gps_int.y);

    /* DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice, &fps, &dummy_uint16, &dummy_uint16, &flow_x, &flow_y, &dummy_int16, &dummy_int16,
          &vel_x, &vel_y, &velocity_rot_gps.x, &velocity_rot_gps.y, &dummy_float, &dummy_float, &dummy_float);*/
    counter = 0;
  } else {
    counter++;
  }
#endif

}
