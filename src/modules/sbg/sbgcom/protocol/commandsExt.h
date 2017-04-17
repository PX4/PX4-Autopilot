/*!
 *	\file		commandsExt.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		25/01/10
 *
 *	\brief		Commands used to define Extrnal device parameters on IG-500E<br>
 *				Copyright 2007-2010 SBG Systems. All rights reserved.
 */
#ifndef __COMMANDS_EXT_H__
#define __COMMANDS_EXT_H__

#include "../sbgCommon.h"
#include "protocol.h"
#include "commands.h"

//----------------------------------------------------------------------//
//- External devices Types definition                                  -//   
//----------------------------------------------------------------------//

/*!
 * This enum defines all handeled external devices
 */
typedef enum _SbgExtDeviceType
{
	SBG_EXT_NONE			=	0x00,						/*!< No external device is attached */
	SBG_EXT_UBLOX4			=	0x01,						/*!< Ublox Antaris 4 GPS */
	SBG_EXT_UBLOX6			=	0x02,						/*!< Ublox 6 GPS */
	SBG_EXT_IG_DEVICE		=	0x03,						/*!< SBG Systems IG device */
	SBG_EXT_NMEA			=	0x04						/*!< Other device which uses standard NMEA protocol */
} SbgExtDeviceType;

//----------------------------------------------------------------------//
//- External devices definitions                                       -//
//----------------------------------------------------------------------//

#define SBG_EXT_PORT_RS232					(0x0000)		/*!< Set the external port in RS-232 mode (default) */
#define SBG_EXT_PORT_RS422					(0x0001)		/*!< Set the external port in RS-422 mode */
#define SBG_EXT_PORT_FAST_SLEW				(0x0000)		/*!< Fast slew rate, baudrate is not limited. (default) */
#define SBG_EXT_PORT_SLOW_SLEW				(0x0002)		/*!< Slow slew rate for EMI reduction. baudrate is limited to 230 400 bps. */

//----------------------------------------------------------------------//
//- External device commands                                           -//
//----------------------------------------------------------------------//

/*!
 *	Select the external device connected to the IG-500E and its UART configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	deviceType			The device type connected
 *  \param[in]	baudRate			The baud rate used to communicate with the device at initialization
 *  \param[in]	uartOptions			Some uart Options (bitwize ORed) needed to communicate with the device. Restart the device if changed:
 *									- SBG_EXT_PORT_RS232 or SBG_EXT_PORT_RS422
 *									- SBG_EXT_PORT_DIS_EMI_REDUCTION or SBG_EXT_PORT_EN_EMI_REDUCTION
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgSetExtDevice(SbgProtocolHandle handle, SbgExtDeviceType deviceType, uint32 baudRate, uint16 uartOptions);

/*!
 *	Get the external device connected to the IG-500E and its UART configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[out]	pDeviceType			The device type connected
 *  \param[out]	pBaudRate			The baud rate used to communicate with the device at initialization
 *  \param[out]	pUartOptions		Some uart Options (bitwize ORed) used to communicate with the device
 *									- SBG_EXT_PORT_RS232 or SBG_EXT_PORT_RS422
 *									- SBG_EXT_PORT_DIS_EMI_REDUCTION or SBG_EXT_PORT_EN_EMI_REDUCTION
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgGetExtDevice(SbgProtocolHandle handle, SbgExtDeviceType *pDeviceType, uint32 *pBaudRate, uint16 *pUartOptions);

/*!
 *	Send to the external device attached a specific configuration
 *	\param[in]	handle				A valid sbgCom library handle.
 *  \param[in]	pCommand			Command sent to the external device
 *  \param[in]	commandSize			Size of the command to send
 *  \param[out]	pAnswer				Answer of the external device
 *  \param[out]	pAnswerSize			Size of the device answer
 *  \param[in]	answerMaxSize		Maximum allowed size of the answer
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgExtDeviceConfig(SbgProtocolHandle handle, const uint8 *pCommand, uint16 commandSize, uint8 *pAnswer, uint16 *pAnswerSize, uint16 answerMaxSize);




#endif
