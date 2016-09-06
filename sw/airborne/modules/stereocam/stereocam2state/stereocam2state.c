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

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_indi.h"

#include "subsystems/imu.h"

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

struct MedianFilterInt filter_1;
struct MedianFilterInt filter_2;

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
  init_median_filter(&filter_1);
  init_median_filter(&filter_2);

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

  if(fps == 0)
  {
	  fps = 25.0f;
  }
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

  uint8_t edgeflow_avoid_mode = stereocam_data.data[20];
  /* int16_t vel_x_stereo_avoid_pixelwise_int = (int16_t)stereocam_data.data[20] << 8;
   vel_x_stereo_avoid_pixelwise_int |= (int16_t)stereocam_data.data[21];
    int16_t vel_z_stereo_avoid_pixelwise_int = (int16_t)stereocam_data.data[22] << 8;
    vel_z_stereo_avoid_pixelwise_int |= (int16_t)stereocam_data.data[23];*/

  int16_t vel_x_stereo_avoid_pixelwise_int = 0;
  int16_t vel_z_stereo_avoid_pixelwise_int = 0;

  int16_t RES = 100;

  struct Int16Vect3 vel, vel_global;
  vel.x = vel_x_pixelwise_int;
  vel.y = vel_z_pixelwise_int;

  vel_global.x = vel_x_global_int;
  vel_global.y = vel_y_global_int;
  vel_global.z = vel_z_global_int;

  ///vel.x = update_median_filter2(&filter_1,vel.x,10);
  //vel.y = update_median_filter2(&filter_2,  vel.y,5);



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


  float vel_body_x_filter = 0;
  float vel_body_y_filter = 0;

  struct FloatVect3 velocity_rot_gps;
  struct Int32Vect3 velocity_rot_gps_int;
  float_rmat_vmult(&velocity_rot_gps , stateGetNedToBodyRMat_f(), (struct FloatVect3 *)&opti_vel);
  int32_rmat_vmult(&velocity_rot_gps_int , stateGetNedToBodyRMat_i(), (struct Int32Vect3 *)&opti_vel_int);


  int16_t vel_x_opti_int = - (int16_t)(velocity_rot_gps.y * 100);
  int16_t vel_y_opti_int = -(int16_t)(velocity_rot_gps.x * 100);
  int16_t vel_z_opti_int = -(int16_t)(velocity_rot_gps.z * 100);



  // Avoidance logic
  float avoid_turn = 30.0f * 3.14f / 180.0f;
  float avoid_turn_strong = 110.0f * 3.14f / 180.0f;
  float avoid_turn_rate = 50.0f * 3.14f / 180.0f;


  float forward_speed = 0.5f;
  static bool drone_is_turning = false;
  float current_heading = stateGetNedToBodyEulers_f()->psi;
  float current_pitch = stateGetNedToBodyEulers_f()->theta;

  float yaw_rate   = stateGetBodyRates_f()->r;
  //vel_body_x = vel_body_x - yaw_rate;

  static float prev_heading = 0;
  uint8_t wait_delay = 40;

  static bool flip_switch = true;
  static uint8_t turn_counter = 0;
  static uint8_t wait_counter = 0;

  static bool obstacle_detected =  false;


  static uint8_t behavior_mode = 0;


// check if in guidance mode
  if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {

	  // if flipswitch is true
    if (flip_switch == true) {
      guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
      turn_counter = 0;
      flip_switch = false;
      drone_is_turning = false;
      obstacle_detected =  false;
      prev_heading = 0;
      guidance_h_set_guided_body_vel(0.0f, 0.0f);
      guidance_h_set_guided_heading_rate(0.0f);
      behavior_mode = 0;

    }




// detect if obstacle is detected
    if (edgeflow_avoid_mode == 11 || edgeflow_avoid_mode == 12 || edgeflow_avoid_mode == 22 || edgeflow_avoid_mode == 21) {
      obstacle_detected = true;
    }

    if (edgeflow_avoid_mode == 4 || edgeflow_avoid_mode == 10) {
      obstacle_detected = false;
    }

// switchs modes after counter
    if (behavior_mode == 0 && obstacle_detected == false && wait_counter == wait_delay) {
      behavior_mode = 1;
      wait_counter = 0;
    }

    if (behavior_mode == 0 && obstacle_detected == true && wait_counter == wait_delay) {
      behavior_mode = 2;
      prev_heading = current_heading;
      wait_counter = 0;
    }

    if (behavior_mode == 1 && obstacle_detected == true) {
      behavior_mode = 0;
      wait_counter = 0;

    }

    if (behavior_mode == 2 && avoid_turn_strong<fabs(current_heading - prev_heading)) {
      behavior_mode = 0;
      wait_counter = 0;
    }

    // Change measurements
    if (behavior_mode == 2) {
      //vel_body_x = 0;//0.1;
      //vel_body_y = 0;//-0.1;

    }

    if (behavior_mode == 1) {
      if (edgeflow_avoid_mode == 4)
      {}
      if (edgeflow_avoid_mode == 10) {
        vel_body_x = -0.1;
      }
    }

    // increment counter if in wait mode or turn mode
    if (behavior_mode == 0 ) {
      wait_counter ++;
    }

/*    // Bound measurments if above a certain value.
    if (fabs(vel_body_x) > 1.0) {
      vel_body_x = vel_body_x * 1.0 / fabs(vel_body_x);
    }

    if (fabs(vel_body_y) > 1.0) {
      vel_body_y = vel_body_y * 1.0 / fabs(vel_body_y);
    }*/


    // KALMAN filter
      struct Int32Vect3 acc_meas_body;
      struct Int32RMat *body_to_imu_rmat = orientationGetRMat_i(&imu.body_to_imu);
      int32_rmat_transp_vmult(&acc_meas_body, body_to_imu_rmat, &imu.accel);

      static uint8_t wait_filter_counter = 0;
      static float previous_state_x[2] = {0.0f, 0.0f};
      static float covariance_x[4] = {1.0f, 1.0f, 1.0f, 1.0f};
      float measurements_x[2];

      static float previous_state_y[2] = {0.0f, 0.0f};
      static float covariance_y[4] = {1.0f, 1.0f, 1.0f, 1.0f};
      float measurements_y[2];

      float process_noise[2] = {0.01f, 0.01f};
      float measurement_noise[2] = {0.2f, 1.0f};






      if (wait_filter_counter > 100) {

        measurements_x[0] = vel_body_x;
        measurements_x[1] = ACCEL_FLOAT_OF_BFP(acc_meas_body.x);

        measurements_y[0] = vel_body_y;
        measurements_y[1] = ACCEL_FLOAT_OF_BFP(acc_meas_body.y);
        // Bound measurments if above a certain value.
        if (!(fabs(vel_body_x) > 1.0 || fabs(vel_body_y) > 1.0)) {

        kalman_filter(measurements_x, covariance_x,
                      previous_state_x, process_noise, measurement_noise, fps);
        kalman_filter(measurements_y, covariance_y,
                      previous_state_y, process_noise, measurement_noise, fps);
        }

       vel_body_x_filter = previous_state_x[0];
        vel_body_y_filter =  previous_state_y[0];


      } else {
      	wait_filter_counter++;
      }

      /////////////////////////////////////////////

    // send velocity to state
//    uint32_t now_ts = get_sys_time_usec();
//    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
//                                vel_body_x_filter,
//                                vel_body_y_filter,
//                                0.0f,
//                                0.2f
//                               );

    switch (behavior_mode) {
      case 0:
//        if (wait_counter < 5) {
//
//          guidance_h_set_guided_body_vel(-0.2, 0.0);
//        } else {
//          guidance_h_set_guided_body_vel(0.0, 0.2);
//        }

        guidance_h_set_guided_heading_rate(0.0);
         guidance_h_set_guided_body_vel(0.0, 0.0);


        break;
      case 1:
          guidance_h_set_guided_heading_rate(0.0);

        guidance_h_set_guided_body_vel(0.3f, 0.0f);

        break;
      case 2:
       // guidance_h_set_guided_body_vel(0.0, -0.2);

        guidance_h_set_guided_heading_rate(-avoid_turn_rate);

        guidance_h_set_guided_body_vel(0.05, 0.0);


        // guidance_h_set_guided_body_vel(0.5, 0.5);
        break;
      default:
        guidance_h_set_guided_body_vel(0.0, 0.0);
        guidance_h_set_guided_heading(0);

    }





  } else {
    flip_switch = true;
  }


  // Reusing the OPTIC_FLOW_EST telemetry messages, with some values replaced by 0

  uint16_t dummy_uint16 = 0;
  int16_t dummy_int16 = 0;
  float dummy_float = 0;
  static int16_t counter = 0;


  if (counter == 5) {
    DOWNLINK_SEND_STEREOCAM_OPTIC_FLOW(DefaultChannel, DefaultDevice, &vel_body_x_int, &vel_body_y_int,
                                       &vel_body_x_global_int, &vel_body_y_global_int,
                                       &vel_body_z_global_int, &vel_x_opti_int,  &vel_y_opti_int, &behavior_mode,
                                       &vel_body_x, &vel_body_y, &vel_body_x_filter, &vel_body_y_filter, &edgeflow_avoid_mode);
    counter = 0;
  } else {
    counter++;
  }
#endif

}


void kalman_filter(float *measurements, float *covariance, float *state
                   , float *process_noise, float *measurement_noise, float fps)
{
  // _______ Preparation kalman filter _______ \\

  // process model (linear)
  float _G[2][2];
  MAKE_MATRIX_PTR(G, _G, 2);
  G[0][0] = 1.0f;
  G[0][1] = 1.0f / fps;
  G[1][0] = 0.0f;
  G[1][1] = 1.0f;


  // transpose of G
  float _Gtrans[2][2];
  MAKE_MATRIX_PTR(Gtrans, _Gtrans, 2);
  float_mat_copy(Gtrans, G, 2, 2);
  float_mat_transpose(Gtrans, 2);

  // measurement model (linear)
  float _H[2][2];
  MAKE_MATRIX_PTR(H, _H, 2);
  H[0][0] = 1.0f;
  H[0][1] = 0.0f;
  H[1][0] = 0.0f;
  H[1][1] = 1.0f;

  //transpose of H
  float _Htrans[2][2];
  MAKE_MATRIX_PTR(Htrans, _Htrans, 2);
  float_mat_copy(Htrans, H, 2, 2);
  float_mat_transpose(Htrans, 2);

  //Previous state
  float _Xprev[1][2];
  MAKE_MATRIX_PTR(Xprev, _Xprev, 2);
  Xprev[0][0] = state[0];
  Xprev[1][0] = state[1]; //state[1];

  //Previous covariance
  float _Pprevious[2][2];
  MAKE_MATRIX_PTR(Pprevious, _Pprevious, 2);
  Pprevious[0][0] = covariance[0];
  Pprevious[0][1] = covariance[1];
  Pprevious[1][0] = covariance[2];
  Pprevious[1][1] = covariance[3];

  //measurements;
  float _Z[1][2];
  MAKE_MATRIX_PTR(Z, _Z, 2);
  Z[0][0] = measurements[0];
  Z[1][0] = measurements[1];

  //Process noise model
  float _Q[2][2];
  MAKE_MATRIX_PTR(Q, _Q, 2);
  Q[0][0] = process_noise[0];
  Q[0][1] = 0.0f;
  Q[1][0] = 0.0f;
  Q[1][1] =  process_noise[1];

  //measurement nosie model
  float _R[2][2];
  MAKE_MATRIX_PTR(R, _R, 2);
  R[0][0] = measurement_noise[0];
  R[0][1] = 0.0f;
  R[1][0] = 0.0f;
  R[1][1] = measurement_noise[1];

  //Variables during kalman computation:
  float _Xpredict[1][2];
  MAKE_MATRIX_PTR(Xpredict, _Xpredict, 2);
  float _Xnext[1][2];
  MAKE_MATRIX_PTR(Xnext, _Xnext, 2);

  float _Ppredict[2][2];
  MAKE_MATRIX_PTR(Ppredict, _Ppredict, 2);
  float _Pnext[2][2];
  MAKE_MATRIX_PTR(Pnext, _Pnext, 2);

  float _K[2][2];
  MAKE_MATRIX_PTR(K, _K, 2);

  float _eye[2][2];
  MAKE_MATRIX_PTR(eye, _eye, 2);
  eye[0][0] = 1.0f;
  eye[0][1] = 0.0f;
  eye[1][0] = 0.0f;
  eye[1][1] = 1.0f;

  float _temp_mat[2][2];
  MAKE_MATRIX_PTR(temp_mat, _temp_mat, 2);
  float _temp_mat2[2][2];
  MAKE_MATRIX_PTR(temp_mat2, _temp_mat2, 2)
  float _temp_mat3[2][2];
  MAKE_MATRIX_PTR(temp_mat3, _temp_mat3, 2)

  float _temp_vec[1][2];
  MAKE_MATRIX_PTR(temp_vec, _temp_vec, 2);
  float _temp_vec2[1][2];
  MAKE_MATRIX_PTR(temp_vec2, _temp_vec2, 2)


  //_______  KALMAN FILTER_______ \\
  //_______  calculate state predict  _______ \\

  //Xpredict = G* Xprev;
  float_mat_mul(Xpredict, G, Xprev, 2, 2, 1);

  // _______  calculate covariance predict _______ \\
  // Ppredict = G*Pprevious*Gtrans + Q
  //...Pprevious*Gtrans...
  float_mat_mul(temp_mat, Pprevious, Gtrans, 2, 2, 2);
  //G*Pprevious*Gtrans...
  float_mat_mul(temp_mat2, G, temp_mat, 2, 2, 2);
  //G*Pprevious*Gtrans+Q
  float_mat_sum(Ppredict, temp_mat2, Q, 2, 2);

  //_______ Calculate Kalman gain _______ \\
  // K = Ppredict * Htrans /( H * Ppredict * Htrans + R)
  // ... Ppredict * Htrans ...
  float_mat_mul(temp_mat, Ppredict, Htrans, 2, 2, 2);
  //... H * Predict * Htrans
  float_mat_mul(temp_mat2, H, temp_mat, 2, 2, 2);
  //..( H * Ppredict * Htrans + R)
  float_mat_sum(temp_mat3, temp_mat2, R, 2, 2);
  //...inv( H * Ppredict * Htrans + R)
  float det_temp2 = 1 / (temp_mat3[0][0] * temp_mat3[1][1] - temp_mat3[0][1] * temp_mat3[1][0]);
  temp_mat2[0][0] =  det_temp2 * (temp_mat3[1][1]);
  temp_mat2[0][1] =  det_temp2 * (-1 * temp_mat3[1][0]);
  temp_mat2[1][0] =  det_temp2 * (-1 * temp_mat3[0][1]);
  temp_mat2[1][1] =  det_temp2 * (temp_mat3[0][0]);
  // K = Ppredict * Htrans / *inv( H * Ppredict * Htrans + R)
  float_mat_mul(K, temp_mat, temp_mat2, 2, 2, 2);



  // _______  Calculate next state  _______ \\
  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  // ... Htrans * Xpredict)
  float_mat_mul(temp_vec, Htrans, Xpredict, 2, 2, 1);

  //... (Z - Htrans * Xpredict)
  float_mat_diff(temp_vec2, Z, temp_vec, 2, 1);

  // ... K *(Z - Htrans * Xpredict)
  float_mat_mul(temp_vec, K, temp_vec2, 2, 2, 1);


  //Xnext = Xpredict + K *(Z - Htrans * Xpredict)
  float_mat_sum(Xnext, Xpredict, temp_vec, 2, 1);

  // _______ calculate next covariance matrix _______ \\
  // Pnext = (eye(2) - K*H)*P_predict
  // ...K*H...
  float_mat_mul(temp_mat, K, H, 2, 2, 2);
  //(eye(2) - K*H)
  float_mat_diff(temp_mat2, eye, temp_mat, 2, 2);
  // Pnext = (eye(2) - K*H)*P_predict
  float_mat_mul(Pnext, temp_mat2, Ppredict, 2, 2, 2);


  //save values for next state
  covariance[0] = Pnext[0][0];
  covariance[1] = Pnext[0][1];;
  covariance[2] = Pnext[1][0];;
  covariance[3] = Pnext[1][1];;


  state[0] = Xnext[0][0];
  state[1] = Xnext[1][0];
//
/*    printf("fps %f\n",fps);
    printf("Xprev %f, %f\n",Xprev[0][0],Xprev[1][0]);
    printf("Xpredict %f, %f\n",Xpredict[0][0],Xpredict[1][0]);
    printf("Pprevious %f,%f,%f,%f\n", Pprevious[0][0],Pprevious[0][1],Pprevious[1][0],Pprevious[1][1]);
    printf("Ppredict %f,%f,%f,%f\n", Ppredict[0][0],Ppredict[0][1],Ppredict[1][0],Ppredict[1][1]);
    printf("Pnext %f,%f,%f,%f\n", Pnext[0][0],Pnext[0][1],Pnext[1][0],Pnext[1][1]);
    printf("Z %f, %f \n", Z[0][0],Z[1][0]);
    printf("Kalman %f,%f,%f,%f\n", K[0][0],K[0][1],K[1][0],K[1][1]);
    printf("Xnext %f, %f\n",Xnext[0][0],Xnext[1][0]);*/
}
