/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "../decadriver/deca_device_api.h"
#include "port.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
	while (port_SPIx_busy_sending()); //wait for tx buffer to empty

	port_SPIx_disable();

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
#include "mcu_periph/spi.h"

struct spi_transaction decawave_spi_link_transaction;
struct uint8_t decawave_spi_link_data[255];
static volatile bool decawave_spi_link_ready = true;

static void decawave_spi_link_trans_cb(struct spi_transaction *trans);

void initspi(void)
{

	  high_speed_logger_spi_link_data.id = 0;

decawave_spi_link_transaction.select        = SPISelectUnselect;
 decawave_spi_link_transaction.cpol          = SPICpolIdleHigh; //find out
 decawave_spi_link_transaction.cpha          = SPICphaEdge2;    //find out
 decawave_spi_link_transaction.dss           = SPIDss8bit;
 decawave_spi_link_transaction.bitorder      = SPIMSBFirst;
 decawave_spi_link_transaction.cdiv          = SPIDiv64;
 decawave_spi_link_transaction.slave_idx     = SPI_SLAVE0;//DECAWAVE_SPI_LINK_SLAVE_NUMBER;
 decawave_spi_link_transaction.output_length = 255;
 decawave_spi_link_transaction.output_buf    = (uint8_t *) &decawave_spi_link_data;
 decawave_spi_link_transaction.input_length  = 0;
 decawave_spi_link_transaction.input_buf     = NULL;
 decawave_spi_link_transaction.after_cb      = decawave_spi_link_trans_cb;

 int i=0;
}

static void decawave_spi_link_trans_cb(struct spi_transaction *trans)
{
	decawave_spi_link_ready =  true;
}


int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{


	uint8_t header_body_array[255];

	memcpy(header_body_array, headerBuffer, headerLength * sizeof(*uint8_t));
	memcpy(header_body_array + headerLength, bodyBuffer, bodylength * sizeof(*uint8_t));

	//copy header and buffer in one

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &decawave_spi_link_transaction);


    // old code
   /* decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPIx->DR = headerBuffer[i];

    	while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

    	SPIx->DR ;
    }

    for(i=0; i<bodylength; i++)
    {
     	SPIx->DR = bodyBuffer[i];

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

		SPIx->DR ;
	}

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;
*/
    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{

	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    SPIx_CS_GPIO->BRR = SPIx_CS;

    for(i=0; i<headerLength; i++)
    {
    	SPIx->DR = headerBuffer[i];

     	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

     	readBuffer[0] = SPIx->DR ; // Dummy read as we write the header
    }

    for(i=0; i<readlength; i++)
    {
    	SPIx->DR = 0;  // Dummy write as we read the message body

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
 
	   	readBuffer[i] = SPIx->DR ;//port_SPIx_receive_data(); //this clears RXNE bit
    }

    SPIx_CS_GPIO->BSRR = SPIx_CS;

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
