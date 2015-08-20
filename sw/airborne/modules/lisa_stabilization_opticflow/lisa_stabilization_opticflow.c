/*
 * Copyright (C) K N McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/lisa_stabilization_opticflow/lisa_stabilization_opticflow.c"
 * @author K N McGuire
 * Guidance stabilization based on optical flow detected with a downfacing stereocam on a Lisa s nanoquad
 */

// Own Header
#include "modules/lisa_stabilization_opticflow/lisa_stabilization_opticflow.h"

// Stabilization
#include "modules/lisa_stereo/lisa_stereo.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/telemetry.h"

#define CMD_OF_SAT  1500 // 40 deg = 2859.1851
#define DISTANCE_PINHOL 0.03
#define FOV_X 54.7
#define FOV_Y 45.0
#define IMAGEWIDTH 128
#define IMAGEHEIGHT 96
#define FPS 25

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_PHI_PGAIN)

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 0
#endif
PRINT_CONFIG_VAR(VISION_PHI_IGAIN)

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 400
#endif
PRINT_CONFIG_VAR(VISION_THETA_PGAIN)

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 0
#endif
PRINT_CONFIG_VAR(VISION_THETA_IGAIN)


int16_t trans_x;
int16_t trans_y;
int16_t slope_x;
int16_t slope_y;
int16_t height;

float radperpx;
float angle_disp;
float height_meters;

float velocity_x;
float velocity_y;
float desired_vx;
float desired_vy;

float err_vx_int;
float err_vy_int;

int cmd_phi;
int cmd_theta;

int phi_gain_p=VISION_PHI_PGAIN;
int phi_gain_i=VISION_PHI_IGAIN;
int theta_gain_p=VISION_THETA_PGAIN;
int theta_gain_i=VISION_THETA_IGAIN;


uint8_t send_data_2;

struct Int32Eulers cmd;

static struct opticflow_result_t opticflow_result;

static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
    pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
                                 &opticflow_result.fps, &opticflow_result.corner_cnt,
                                 &opticflow_result.tracked_cnt, &opticflow_result.flow_x,
                                 &opticflow_result.flow_y, &opticflow_result.flow_der_x,
                                 &opticflow_result.flow_der_y, &opticflow_result.vel_x,
                                 &opticflow_result.vel_y, &opticflow_result.div_size,
                                 &cmd_phi, &cmd_theta);
}
void lisa_stab_of_init(void)
{
    trans_x=0;
    trans_y=0;
    slope_x=0;
    slope_y=0;
    height=0;

    radperpx=0;
    angle_disp=0;
    height_meters=0;

    velocity_x=0;
    velocity_y=0;

    desired_vx=0;
    desired_vy=0;

    err_vx_int=0;
    err_vy_int=0;

    phi_gain_p=400;
    phi_gain_i=20;
    theta_gain_p=400;
    theta_gain_i=20;

    // register_periodic_telemetry(DefaultPeriodic, "OPTIC_FLOW_EST", opticflow_telem_send);

    send_data_2=0;
}
void lisa_stab_of_start(void)
{
}

void lisa_stab_of_periodic(void)
{

    //collect flow info from uart
    trans_x=(msg_buf[1]-100);
    trans_y=(msg_buf[3]-100);
    /*slope_x=(msg_buf[0]-100)/1000;
    slope_y=(msg_buf[2]-100)/1000;*/
    height=(msg_buf[4]);

    //estimate height based on displacement
    radperpx=(FOV_X*M_PI/180)/128;
    angle_disp=height/2*radperpx;
    height_meters=DISTANCE_PINHOL/tan(angle_disp);

    if(isnan(height_meters)||isinf(height_meters))
        height_meters=0.0;

    //Determine Velocity with pixel displacement and height
    radperpx=(FOV_X*M_PI/180)/IMAGEWIDTH;
    angle_disp=trans_x*radperpx;
    velocity_x=height_meters*tan(angle_disp)*FPS;

    radperpx=(FOV_Y*M_PI/180)/IMAGEHEIGHT;
    angle_disp=trans_y*radperpx;
    velocity_y=-height_meters*tan(angle_disp)*FPS;


    //check if autopilot is in AP
    if (autopilot_mode != AP_MODE_MODULE) {
        err_vx_int=0;
        err_vy_int=0;
        cmd_phi=0;
        cmd_theta=0;
    }else{

        // Velocity error
    float err_vx = 0;
    float err_vy = 0;
    err_vx = desired_vx - velocity_x;
    err_vy = desired_vy - velocity_y;

    // Intergral error
    err_vx_int += err_vx/512;
    err_vy_int += err_vy/512;

    //Commands
    cmd_phi   = (((float)phi_gain_p)/100 * err_vx
            + ((float)phi_gain_i)/100 * err_vx_int);
    cmd_theta = -(((float)theta_gain_p)/100  * err_vy
                  + ((float)theta_gain_i)/100* err_vy_int);

    BoundAbs(cmd_phi, CMD_OF_SAT);
    BoundAbs(cmd_theta, CMD_OF_SAT);

    }
//storing in optical flow structure
    opticflow_result.flow_x=trans_x;
    opticflow_result.flow_y=trans_y;
    opticflow_result.vel_x=velocity_x;
    opticflow_result.vel_y=velocity_y;
    opticflow_result.tracked_cnt=100;
    opticflow_result.corner_cnt=100;

    //velocity_calculate(&opticflow_result);

    cmd.phi=cmd_phi;
    cmd.theta=cmd_theta;

    //send messages with 4 hz
    send_data_2 = (send_data_2 + 1) % 128;
    if (send_data_2 == 0)
    {

        // DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &trans_x_int, &a, &a, &a, 5, msg_buf);

        DOWNLINK_SEND_OPTIC_FLOW_EST(DefaultChannel, DefaultDevice,
                                     &opticflow_result.fps, &opticflow_result.corner_cnt,
                                     &opticflow_result.tracked_cnt, &opticflow_result.flow_x,
                                     &opticflow_result.flow_y, &opticflow_result.flow_der_x,
                &opticflow_result.flow_der_y, &opticflow_result.vel_x,
                &opticflow_result.vel_y, &opticflow_result.div_size,
                &cmd_phi,&cmd_theta);
    }

}


void velocity_calculate(struct opticflow_result_t *opticflow_result)
{


}

void send_edge_flow_velocity(void)
{

}


        // Implement own Horizontal loops
extern void guidance_h_module_enter(void)
{

    err_vx_int=0;
    err_vy_int=0;

    cmd.phi=0;
cmd.theta=0;
cmd.psi=stateGetNedToBodyEulers_i()->psi;
}

extern void guidance_h_module_read_rc(void)
{

}
extern void guidance_h_module_run(bool_t in_flight)
{
    /* Update the setpoint */
    stabilization_attitude_set_rpy_setpoint_i(&cmd);

    /* Run the default attitude stabilization */
    stabilization_attitude_run(in_flight);
}

