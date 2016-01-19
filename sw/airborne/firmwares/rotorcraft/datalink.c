/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/datalink.c
 * Handling of messages coming from ground and other A/Cs.
 *
 */

#define DATALINK_C
#define MODULES_DATALINK_C

#include "subsystems/datalink/datalink.h"

#include "generated/modules.h"

#include "generated/settings.h"
#include "subsystems/datalink/downlink.h"
#include "messages.h"
#include "dl_protocol.h"
#include "mcu_periph/uart.h"

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_DATALINK
#include "subsystems/radio_control.h"
#endif

#if USE_GPS
#include "subsystems/gps.h"
#endif
#if defined GPS_DATALINK
#include "subsystems/gps/gps_datalink.h"
#endif

#ifdef TRAFFIC_INFO
#include "subsystems/navigation/traffic_info.h"
#endif // TRAFFIC_INFO

#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"

#include "math/pprz_geodetic_int.h"
#include "state.h"
#include "led.h"

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMM(_x) (((float)(_x))/1000.)

#define IdOfMsg(x) (x[1])
#define SenderIdOfMsg(x) (x[0])

#if USE_NPS
bool_t datalink_enabled = TRUE;
#endif

void dl_parse_msg(void)
{

  uint8_t sender_id = SenderIdOfMsg(dl_buffer);
  uint8_t msg_id = IdOfMsg(dl_buffer);

  /* parse telemetry messages coming from other AC */
  if (sender_id != 0) {
    switch (msg_id) {
#ifdef TRAFFIC_INFO
      case DL_GPS_SMALL: {
        uint32_t multiplex_speed = DL_GPS_SMALL_multiplex_speed(dl_buffer);

        // Position in ENU coordinates
        int16_t course = (int32_t)((multiplex_speed >> 21) & 0x7FF); // bits 31-21 course in decideg
        if (course & 0x400) {
          course |= 0xFFFFF800;  // fix for twos complements
        }
        int16_t gspeed = (int32_t)((multiplex_speed >> 10) & 0x7FF); // bits 20-10 ground speed cm/s
        if (gspeed & 0x400) {
          gspeed |= 0xFFFFF800;  // fix for twos complements
        }
        int16_t climb = (int16_t)(multiplex_speed >> 2 & 0x7FF); // bits 9-0 z climb speed in cm/s
        if (climb & 0x400) {
          climb |= 0xFFFFF800;  // fix for twos complements
        }

        SetAcInfo(sender_id,
          MOfCm(DL_GPS_SMALL_utm_east(dl_buffer)),    /*m*/
          MOfCm(DL_GPS_SMALL_utm_north(dl_buffer)),   /*m*/
          RadOfDeg(((float)course) / 10.),            /*rad(CW)*/
          MOfCm(DL_GPS_SMALL_alt(dl_buffer)),         /*m*/
          MOfCm(gspeed),                              /*m/s*/
          MOfCm(climb),                               /*m/s*/
          gps_tow_from_sys_ticks(sys_time.nb_tick));

          int32_t east = DL_GPS_SMALL_utm_east(dl_buffer);
          int32_t north = DL_GPS_SMALL_utm_north(dl_buffer);
          int16_t alt = DL_GPS_SMALL_alt(dl_buffer);

          DOWNLINK_SEND_GPS_SMALL(DefaultChannel, DefaultDevice, &multiplex_speed, &east, &north, &alt);
      }
      break;

      case DL_GPS: {
        SetAcInfo(sender_id,
          MOfCm(DL_GPS_utm_east(dl_buffer)),    /*m*/
          MOfCm(DL_GPS_utm_north(dl_buffer)),   /*m*/
          RadOfDeg(((float)DL_GPS_course(dl_buffer)) / 10.), /*rad(CW)*/
          MOfCm(DL_GPS_alt(dl_buffer)),        /*m*/
          MOfCm(DL_GPS_speed(dl_buffer)),       /*m/s*/
          MOfCm(DL_GPS_climb(dl_buffer)),       /*m/s*/
          (uint32_t)DL_GPS_itow(dl_buffer));
      }
      break;
      case DL_GPS_LLA: {
        SetAcInfoLLA(sender_id,
          DL_GPS_LLA_lat(dl_buffer),    /*1e7deg*/
          DL_GPS_LLA_lon(dl_buffer),    /*1e7deg*/
          DL_GPS_LLA_alt(dl_buffer),    /*mm*/
          DL_GPS_LLA_course(dl_buffer), /*decideg*/
          DL_GPS_LLA_speed(dl_buffer),  /*cm/s*/
          DL_GPS_LLA_climb(dl_buffer),  /*cm/s*/
          DL_GPS_LLA_itow(dl_buffer));  /*ms*/
      }
      break;
#endif
      default: {
        break;
      }
    }
    return;  // msg was telemetry not datalink so return
  }

  /* parse telemetry messages coming from ground station */
  switch (msg_id) {

    case  DL_PING: {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

    case DL_SETTING : {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

    case DL_GET_SETTING : {
      if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t i = DL_GET_SETTING_index(dl_buffer);
      float val = settings_get_value(i);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
    }
    break;

#ifdef USE_NAVIGATION
    case DL_BLOCK : {
      if (DL_BLOCK_ac_id(dl_buffer) != AC_ID) { break; }
      nav_goto_block(DL_BLOCK_block_id(dl_buffer));
    }
    break;

    case DL_MOVE_WP : {
      uint8_t ac_id = DL_MOVE_WP_ac_id(dl_buffer);
      if (ac_id != AC_ID) { break; }
      if (stateIsLocalCoordinateValid()) {
        uint8_t wp_id = DL_MOVE_WP_wp_id(dl_buffer);
        struct LlaCoor_i lla;
        lla.lat = DL_MOVE_WP_lat(dl_buffer);
        lla.lon = DL_MOVE_WP_lon(dl_buffer);
        /* WP_alt from message is alt above MSL in mm
         * lla.alt is above ellipsoid in mm
         */
        lla.alt = DL_MOVE_WP_alt(dl_buffer) - state.ned_origin_i.hmsl +
                  state.ned_origin_i.lla.alt;
        waypoint_move_lla(wp_id, &lla);
      }
    }
    break;
#endif /* USE_NAVIGATION */
#ifdef RADIO_CONTROL_TYPE_DATALINK
    case DL_RC_3CH :
#ifdef RADIO_CONTROL_DATALINK_LED
      LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
      parse_rc_3ch_datalink(
        DL_RC_3CH_throttle_mode(dl_buffer),
        DL_RC_3CH_roll(dl_buffer),
        DL_RC_3CH_pitch(dl_buffer));
      break;
    case DL_RC_4CH :
      if (DL_RC_4CH_ac_id(dl_buffer) == AC_ID) {
#ifdef RADIO_CONTROL_DATALINK_LED
        LED_TOGGLE(RADIO_CONTROL_DATALINK_LED);
#endif
        parse_rc_4ch_datalink(DL_RC_4CH_mode(dl_buffer),
                              DL_RC_4CH_throttle(dl_buffer),
                              DL_RC_4CH_roll(dl_buffer),
                              DL_RC_4CH_pitch(dl_buffer),
                              DL_RC_4CH_yaw(dl_buffer));
      }
      break;
#endif // RADIO_CONTROL_TYPE_DATALINK
#if USE_GPS
#ifdef GPS_DATALINK
    case DL_REMOTE_GPS_SMALL : {
      // Check if the GPS is for this AC
      if (DL_REMOTE_GPS_SMALL_ac_id(dl_buffer) != AC_ID) { break; }

      parse_gps_datalink_small(
        DL_REMOTE_GPS_SMALL_numsv(dl_buffer),
        DL_REMOTE_GPS_SMALL_pos_xyz(dl_buffer),
        DL_REMOTE_GPS_SMALL_speed_xyz(dl_buffer),
        DL_REMOTE_GPS_SMALL_heading(dl_buffer));
    }
    break;

    case DL_REMOTE_GPS : {
      // Check if the GPS is for this AC
      if (DL_REMOTE_GPS_ac_id(dl_buffer) != AC_ID) { break; }

      // Parse the GPS
      parse_gps_datalink(
        DL_REMOTE_GPS_numsv(dl_buffer),
        DL_REMOTE_GPS_ecef_x(dl_buffer),
        DL_REMOTE_GPS_ecef_y(dl_buffer),
        DL_REMOTE_GPS_ecef_z(dl_buffer),
        DL_REMOTE_GPS_lat(dl_buffer),
        DL_REMOTE_GPS_lon(dl_buffer),
        DL_REMOTE_GPS_alt(dl_buffer),
        DL_REMOTE_GPS_hmsl(dl_buffer),
        DL_REMOTE_GPS_ecef_xd(dl_buffer),
        DL_REMOTE_GPS_ecef_yd(dl_buffer),
        DL_REMOTE_GPS_ecef_zd(dl_buffer),
        DL_REMOTE_GPS_tow(dl_buffer),
        DL_REMOTE_GPS_course(dl_buffer));
    }
    break;
#endif // GPS_DATALINK

    case DL_GPS_INJECT : {
      // Check if the GPS is for this AC
      if (DL_GPS_INJECT_ac_id(dl_buffer) != AC_ID) { break; }

      // GPS parse data
      gps_inject_data(
        DL_GPS_INJECT_packet_id(dl_buffer),
        DL_GPS_INJECT_data_length(dl_buffer),
        DL_GPS_INJECT_data(dl_buffer)
      );
    }
    break;
#endif  // USE_GPS

    case DL_GUIDED_SETPOINT_NED:
      if (DL_GUIDED_SETPOINT_NED_ac_id(dl_buffer) != AC_ID) { break; }
      uint8_t flags = DL_GUIDED_SETPOINT_NED_flags(dl_buffer);
      float x = DL_GUIDED_SETPOINT_NED_x(dl_buffer);
      float y = DL_GUIDED_SETPOINT_NED_y(dl_buffer);
      float z = DL_GUIDED_SETPOINT_NED_z(dl_buffer);
      float yaw = DL_GUIDED_SETPOINT_NED_yaw(dl_buffer);
      switch (flags) {
        case 0x00:
        case 0x02:
          /* local NED position setpoints */
          autopilot_guided_goto_ned(x, y, z, yaw);
          break;
        case 0x01:
          /* local NED offset position setpoints */
          autopilot_guided_goto_ned_relative(x, y, z, yaw);
          break;
        case 0x03:
          /* body NED offset position setpoints */
          autopilot_guided_goto_body_relative(x, y, z, yaw);
          break;
        case 0x70:
          /* local NED with x/y/z as velocity and yaw as absolute angle */
          autopilot_guided_move_ned(x, y, z, yaw);
          break;
        default:
          /* others not handled yet */
          break;
      }

#ifdef TRAFFIC_INFO
    case DL_ACINFO: {
      if (DL_ACINFO_ac_id(dl_buffer) == AC_ID) { break; }
      uint8_t id = DL_ACINFO_ac_id(dl_buffer);
      float ux = MOfCm(DL_ACINFO_utm_east(dl_buffer));
      float uy = MOfCm(DL_ACINFO_utm_north(dl_buffer));
      float a = MOfCm(DL_ACINFO_alt(dl_buffer));
      float c = RadOfDeg(((float)DL_ACINFO_course(dl_buffer)) / 10.);
      float s = MOfCm(DL_ACINFO_speed(dl_buffer));
      float cl = MOfCm(DL_ACINFO_climb(dl_buffer));
      uint32_t t = DL_ACINFO_itow(dl_buffer);
      SetAcInfo(id, ux, uy, c, a, s, cl, t);
    }
    break;
#endif
    default:
      break;
  }
  /* Parse modules datalink */
  modules_parse_datalink(msg_id);
}
