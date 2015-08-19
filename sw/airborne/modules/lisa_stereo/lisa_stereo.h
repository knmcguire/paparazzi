/*
 * Copyright (C) 2015 Kirk Scheper
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
 *
 */

/** @file modules/lisa_stereo.h
 *  @brief interface to stereocam
 */

#ifndef LISA_STEREO_H_
#define LISA_STEREO_H_
#include <stdlib.h>
#include "mcu_periph/uart.h"


extern uint8_t msg_buf[];    // buffer used to contain image without line endings
extern uint8_t *img_buf;


extern void lisa_stereo_start(void);
extern void lisa_stereo_stop(void);
extern void lisa_stereo_periodic(void);

#endif /* LISA_STEREO_H_ */
