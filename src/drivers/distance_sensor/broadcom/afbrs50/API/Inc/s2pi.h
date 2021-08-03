/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides an interface for the required S2PI module.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

#ifndef S2PI_H
#define S2PI_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	SPI SPI: Serial Peripheral Interface
 * @ingroup		driver
 * @brief		S2PI: SPI incl. GPIO Hardware Layer Module
 *
 * @details		Provides driver functionality for the S2PI interface module.
 *
 * 				The S2PI module consists of a standard SPI interface plus a
 *				single GPIO interrupt line. Furthermore, the SPI pins are
 *				accessible via GPIO control to allow a software emulation of
 *				additional protocols using the same pins.
 *
 * 				This module actually implements the #argus_s2pi interface that
 * 				is required for the Argus API. Refer to the module for more
 * 				information.
 *
 * @addtogroup 	SPI
 * @{
 *****************************************************************************/

#include "platform/argus_s2pi.h"


/*! Enables the SPI slaves that utilize a GPIO pin for chip select. */
#define S2PI_GPIO_SLAVES 0

/*! The S2PI slaves. */
enum S2PISlaves {
	/*! No SPI slave selected (all pins are disabled w/ high z state). */
	S2PI_NONE = 0,

	/*! The S2PI slave 1 (connected via adapter board). */
	S2PI_S1 = 1,

	/*! The S2PI slave 2 (connected via cable). */
	S2PI_S2 = 2,

#if S2PI_GPIO_SLAVES

	/*! The S2PI slave 3. */
	S2PI_S3 = 3,

	/*! The S2PI slave 4. */
	S2PI_S4 = 4,

	/*! The experimental S2PI slave 1 w/ GPIO CS
	 *  (connected via adapter board). */
	S2PI_S1_GPIO = 5,

	/*! The experimental S2PI slave 2 w/ GPIO CS
	 *  (connected via cable). */
	S2PI_S2_GPIO = 6,

#endif

	/*! No SPI slave selected (all pins go to low state). */
	S2PI_PINS_LOW = 0xFFU,
};


/*!***************************************************************************
 * @brief	Initializes the S2PI module.
 *
 * @details	Setup the board as a S2PI master, this also sets up up the S2PI pins.
 * 			The SPI interface is initialized with the corresponding default
 * 			SPI slave (i.e. CS and IRQ lines) and the default baud rate.
 *
 * @param	defaultSlave The default SPI slave to be addressed right after
 * 						 module initialization.
 * @param	baudRate_Bps The default SPI baud rate in bauds-per-second.
 *
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t S2PI_Init(s2pi_slave_t defaultSlave, uint32_t baudRate_Bps);

/*!***************************************************************************
 * @brief	Sets the SPI baud rate in bps.
 * @param	baudRate_Bps The default SPI baud rate in bauds-per-second.
 * @return 	Returns the \link #status_t status\endlink (#STATUS_OK on success).
 * 			- #STATUS_OK on success
 * 			- #ERROR_S2PI_INVALID_BAUD_RATE on invalid baud rate value.
 *****************************************************************************/
status_t S2PI_SetBaudRate(uint32_t baudRate_Bps);

#ifdef __cplusplus
}
#endif

/*! @} */
#endif // S2PI_H
