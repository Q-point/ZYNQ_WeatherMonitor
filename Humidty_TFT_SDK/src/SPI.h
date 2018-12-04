

#ifndef __SPI_H__
#define __SPI_H__

#include <stdio.h>
#include "platform.h"
#include "xstatus.h"
#include "xspi.h"
#include "xil_printf.h"
#include "xparameters.h"	/* EDK generated parameters */
#include "sleep.h"


/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define SPI_DEVICE_ID		XPAR_AXI_QUAD_SPI_0_DEVICE_ID

/*
 *  This is the size of the buffer to be transmitted/received in this example.
 */
#define BUFFER_SIZE		17


int XSpi_Init(XSpi *SpiPtr, u16 SpiDeviceId);
void SPI_Write(XSpi *SpiPtr, unsigned char data);
void SPI_WriteWord(XSpi *SpiPtr, unsigned int data);


#endif
