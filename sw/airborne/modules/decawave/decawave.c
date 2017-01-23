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
/*
#include "modules/decawave/decadriver/deca_device_api.h"
#include "modules/decawave/decadriver/deca_regs.h"
#include "platform/sleep.h"
 #include "platform/lcd.h"
#include "platform/port.h"
*/
#include <sys/time.h>
#include "mcu_periph/spi.h"
#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include <string.h>
#include <stdlib.h>
#include "inttypes.h"

#include "modules/decawave/libDW1000/dw1000.h"
#include "modules/decawave/libDW1000/libdw1000.h"
#include "modules/decawave/libDW1000/libdw1000Spi.h"
#include "modules/decawave/libDW1000/libdw1000Types.h"

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

static dwOps_t dwOps;
//static InterruptIn _irq = InterruptIn(PA_1);


#define ANTENNA_OFFSET 154.3
#define ANTENNA_DELAY  (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick

bool initialized = false;
bool transmit = false;
bool transmit_succes = false;
static volatile bool isInit = false;

struct spi_transaction decawave_spi_link_transaction;
uint8_t decawave_spi_link_data[255];
uint8_t decawave_spi_link_data_input[255];

static volatile bool decawave_spi_link_ready = true;

static void decawave_spi_link_trans_cb(struct spi_transaction *trans);


/*void decaware_event_init_check()
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

}*/

static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                     const void* data, size_t dataLength)
{

	/*
    //xSemaphoreTake(spiSemaphore, portMAX_DELAY);
    _spiMutex.lock();
    select();
    memcpy(spiBuffer, header, headerLength);
    spiTransfer(spiBuffer, headerLength);
    memcpy(spiBuffer, data, dataLength);
    spiTransfer(spiBuffer, dataLength);
    deselect();
    _spiMutex.unlock();
    //xSemaphoreGive(spiSemaphore);*/

	memcpy(	decawave_spi_link_transaction.output_buf, header, headerLength * sizeof(uint8_t));
	memcpy(	decawave_spi_link_transaction.output_buf + headerLength, data, dataLength * sizeof(uint8_t));

	//copy header and buffer in one

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &decawave_spi_link_transaction);
}


static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                    void* data, size_t dataLength)
{
    //xSemaphoreTake(spiSemaphore, portMAX_DELAY);
   /* _spiMutex.lock();
    select();
    memcpy(spiBuffer, header, headerLength);
    spiTransfer(spiBuffer, headerLength);
    spiTransfer((uint8_t*)data, dataLength, true);
    deselect();
    _spiMutex.unlock();*/
    //xSemaphoreGive(spiSemaphore);


}

void stopTRX()
{
    uint8_t bla = 0x40;
    dwSpiWrite(dwm, SYS_CTRL, 0, &bla, 1);                       // disable tranceiver go back to idle mode
}


void decawave_init() {

decawave_spi_link_transaction.select        = SPISelectUnselect;
decawave_spi_link_transaction.cpol          = SPICpolIdleHigh; //find out
decawave_spi_link_transaction.cpha          = SPICphaEdge2;    //find out
decawave_spi_link_transaction.dss           = SPIDss8bit;
decawave_spi_link_transaction.bitorder      = SPIMSBFirst;
decawave_spi_link_transaction.cdiv          = SPIDiv64;

decawave_spi_link_transaction.slave_idx     = SPI_SLAVE1;//DECAWAVE_SPI_LINK_SLAVE_NUMBER;
decawave_spi_link_transaction.output_length = 255;
decawave_spi_link_transaction.output_buf    = (uint8_t *) &decawave_spi_link_data;
decawave_spi_link_transaction.input_length  = 255;
decawave_spi_link_transaction.input_buf     = (uint8_t *) &decawave_spi_link_data_input;
decawave_spi_link_transaction.after_cb      = decawave_spi_link_trans_cb;

   dwInit(dwm, &dwOps);
  // initialized= dwConfigure(dwm);
  //  deselect();
 //   _spi.format(8.0);
   // _spi.frequency(FAST_SPI);
  //  dwSoftReset(dwm);
   //  stopTRX();


    // Initialize the driver
    // Init libdw




  /* int result = dwConfigure(dwm);
    if (result != 0) {
        isInit = false;
        //DEBUG_PRINT("Failed to configure DW1000!\r\n");
        return;
    } else {
        //DEBUG_PRINT("Worked!\r\n");
    }*/

//    isInit = true;

}

#include "subsystems/datalink/telemetry.h"

void decawave_run() {
if(initialized==true)
	DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);


}

static void decawave_spi_link_trans_cb(struct spi_transaction *trans)
{
	decawave_spi_link_ready =  true;
}
