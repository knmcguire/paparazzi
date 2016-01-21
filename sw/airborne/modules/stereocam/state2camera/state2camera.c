/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/stereocam/state2camera/state2camera.c"
 * @author Roland
 * Sends rotation using the stereoboard protocol over the UART.
 */

#include "modules/stereocam/state2camera/state2camera.h"
#include "modules/stereocam/stereoprotocol.h"
#include "subsystems/abi.h"
#include "state.h"
#include "mcu_periph/uart.h"
#include "subsystems/imu.h"
#include "math/pprz_algebra_int.h"
static int frame_number_sending = 0;
float lastKnownHeight = 0.0;
int pleaseResetOdroid = 0;

void write_serial_rot()
{
  struct Int32RMat *ltp_to_body_mat = stateGetNedToBodyRMat_i();
  static int16_t lengthArrayInformation = 11 * sizeof(int16_t);
  uint8_t ar[lengthArrayInformation];
  int16_t *pointer = (int16_t *) ar;
  /* for (int indexRot = 0; indexRot < 9; indexRot++) {
     pointer[indexRot] = ltp_to_body_mat->m[indexRot];
   }*/
  struct Int32Eulers  euler_angles, euler_rates;

  INT32_EULERS_DOT_OF_RATES(euler_rates, euler_angles, imu.gyro );



  pointer[0] = (int16_t)(stateGetNedToBodyEulers_f()->phi * 100);
  pointer[1] = (int16_t)(stateGetNedToBodyEulers_f()->theta * 100);
  pointer[2] = (int16_t)(stateGetNedToBodyEulers_f()->psi* 100 );

  pointer[3] = (int16_t)(stateGetPositionNed_f()->z* 100 );


  //pointer[4] = (int16_t)(RATE_FLOAT_OF_BFP(euler_rates.phi) * 100);
 // pointer[5] = (int16_t)(RATE_FLOAT_OF_BFP(euler_rates.theta)* 100);
  pointer[4] = (int16_t)(RATE_FLOAT_OF_BFP(imu.gyro.p) * 100);
    pointer[5] = (int16_t)(RATE_FLOAT_OF_BFP(imu.gyro.q)* 100);


  //pointer[9] = (int32_t)(state.alt_agl_f * 100);  //height above ground level in CM.
  pointer[10] = frame_number_sending++;
  stereoprot_sendArray(&((UART_LINK).device), ar, lengthArrayInformation, 1);
}
