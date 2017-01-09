/*
 * Copyright (C) K. N. McGuire
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
 * @file "modules/decawave/decawave.c"
 * @author K. N. McGuire
 * 
 */

#include "modules/decawave/decawave.h"
#include "modules/decawave/decadriver/deca_device_api.h"
#include "modules/decawave/decadriver/deca_regs.h"
#include "platform/sleep.h"
 #include "platform/lcd.h"
#include "platform/port.h"
#include <sys/time.h>
#include "mcu_periph/spi.h"


/* Example application name and version to display on LCD screen. */
#define APP_NAME "SIMPLE TX v1.2"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/**
 * Application entry point.
 */

bool initialized = false;
bool transmit = false;
bool transmit_succes = false;

void decaware_event_init_check()
{
	// check if init worked
if(initialized == false)
	{
	if (dwt_initialise(DWT_LOADNONE) != DWT_ERROR)
		{
			initialized = true;
		}
	}

}


void decaware_event_transmit_succes()
{
	// check if init worked
if(transmit == true)
	{
	if (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		{
			transmit_succes = true;
		}
	}

}

void decaware_event_init()
{
	// check if init worked
if(initialized == false)
	{
	if (dwt_initialise(DWT_LOADNONE) != DWT_ERROR)
		{
			initialized = true;
		}
	}

}

void decawave_init() {
	 /* Start with board specific hardware init. */
	    peripherals_init();

	    /* Display application name on LCD. */
	    //lcd_display_str(APP_NAME);

	    /* Reset and initialise DW1000. See NOTE 2 below.
	     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	     * performance. */
	    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */

	    // SPI set rate low (how to do this???)
	    spi_set_rate_low();
	    // made an event function for this
/*	    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
	    {
	        //lcd_display_str("INIT FAILED");
	        while (1)
	        { };
	    }*/
	    // SPI set rate high (how to do this???)

	    spi_set_rate_high();

	    /* Configure DW1000. See NOTE 3 below. */
	    dwt_configure(&config);
}
void decawave_run() {

    /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
    dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

    /* Start transmission. */
    if (transmit_succes == true)
    {
    dwt_starttx(DWT_START_TX_IMMEDIATE);
    transmit = true;
    }

    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
     * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
     * function to access it.*/
    // DO NOT USE A WHILE LOOP, NO FLIGHT!!

    if (transmit_succes ==  true)
    {
    	  /* Clear TX frame sent event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    transmit_succes = false;
    transmit = false;
    }

    /* Execute a delay between transmissions. */
   // sleep_ms(TX_DELAY_MS);
//    sys_time_msleep(TX_DELAY_MS);
    // periodic function will already handle this.
    usleep(TX_DELAY_MS * 1000);


    /* Increment the blink frame sequence number (modulo 256). */
    tx_msg[BLINK_FRAME_SN_IDX]++;
}


